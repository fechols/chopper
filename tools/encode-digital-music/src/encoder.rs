// Ported from secondreality-opt/src/audio/flac_to_adpcm.rs (Unlicense).
use crate::format::*;
use crate::lloyd::train_1d_codebook_signed;
use anyhow::{bail, Context, Result};
pub struct EncodedDeltaQuant {
    pub blob: DeltaBlob,
    pub rate_actual: u32,
    pub n_samples: u32,
}

/// Train channels separately, preserving DLT2's even segment lengths, then
/// interleave codebooks and put one left/right nibble pair in each stream byte.
pub fn encode_stereo(pcm: &[Vec<i16>; 2], rate: u32) -> Result<EncodedDeltaQuant> {
    if pcm[0].len() != pcm[1].len() {
        bail!("stereo channel lengths differ");
    }
    let left = encode_pcm_to_delta_quant(&pcm[0], rate, 16, 521, 4)?;
    let right = encode_pcm_to_delta_quant(&pcm[1], rate, 16, 521, 4)?;
    let mut codebook_bytes = Vec::with_capacity(left.blob.codebook_bytes.len() * 2);
    for (l, r) in left
        .blob
        .codebook_bytes
        .chunks_exact(48)
        .zip(right.blob.codebook_bytes.chunks_exact(48))
    {
        codebook_bytes.extend_from_slice(l);
        codebook_bytes.extend_from_slice(r);
    }
    let stream: Vec<u8> = (0..left.n_samples as usize)
        .map(|frame| {
            let shift = (frame & 1) * 4;
            let l = (left.blob.stream[frame / 2] >> shift) & 15;
            let r = (right.blob.stream[frame / 2] >> shift) & 15;
            l | (r << 4)
        })
        .collect();
    Ok(EncodedDeltaQuant {
        rate_actual: rate,
        n_samples: left.n_samples,
        blob: DeltaBlob {
            compact_entries: false,
            header: DeltaQuantHeader {
                stream_bytes: stream.len() as u32,
                channels: 2,
                ..left.blob.header
            },
            codebook_bytes,
            stream,
        },
    })
}
pub fn encode_pcm_to_delta_quant(
    pcm: &[i16],
    rate: u32,
    codebook_size: usize,
    num_codebooks: usize,
    bits_per_sample: u8,
) -> Result<EncodedDeltaQuant> {
    if rate == 0 {
        bail!("audio sample rate must be > 0");
    }
    if !matches!(bits_per_sample, 2 | 3 | 4 | 8) {
        bail!(
            "bits_per_sample must be 2, 3, 4, or 8, got {}",
            bits_per_sample
        );
    }
    let max_entries = 1usize << bits_per_sample;
    if codebook_size < 2 || codebook_size > max_entries {
        bail!(
            "codebook_size must be 2..={} at {} bits/sample, got {}",
            max_entries,
            bits_per_sample,
            codebook_size
        );
    }
    if num_codebooks == 0 || num_codebooks > 1024 {
        bail!("num_codebooks must be 1..=1024, got {}", num_codebooks);
    }
    if pcm.is_empty() {
        bail!("cannot delta-encode empty PCM");
    }

    // Open-loop deltas for codebook training: pcm[i] - pcm[i-1] with
    // pcm[-1] = 0. i32 because deltas can exceed i16 range.
    let mut deltas: Vec<i32> = Vec::with_capacity(pcm.len());
    let mut prev: i32 = 0;
    for &s in pcm {
        let d = s as i32 - prev;
        deltas.push(d);
        prev = s as i32;
    }

    // Split samples into `num_codebooks` equal-length segments; the
    // last segment absorbs the remainder. `segment_for(sample_idx)`
    // gives the segment that sample belongs to — must match the target
    // decoder's per-sample boundary check exactly.
    let total_samples = pcm.len();
    let sample_count: u32 = total_samples
        .try_into()
        .context("audio exceeds u32 sample count")?;
    let num_codebooks =
        delta_codebook_count(sample_count, num_codebooks as u16, bits_per_sample) as usize;
    let samples_per_segment =
        delta_samples_per_segment(sample_count, num_codebooks as u16, bits_per_sample) as usize;
    let segment_for = |sample_idx: usize| -> usize {
        if num_codebooks <= 1 {
            0
        } else {
            (sample_idx / samples_per_segment).min(num_codebooks - 1)
        }
    };

    // Train one codebook per segment on that segment's delta
    // distribution. Each segment's deltas are the open-loop deltas
    // from the samples in that segment (reusing `deltas` sliced by
    // segment index).
    let mut cbs_i16: Vec<Vec<i16>> = Vec::with_capacity(num_codebooks);
    let mut codebook_bytes: Vec<u8> =
        Vec::with_capacity(num_codebooks * codebook_size * DELTA_QUANT_ENTRY_SIZE);
    for seg in 0..num_codebooks {
        let start = seg * samples_per_segment;
        let end = if seg == num_codebooks - 1 {
            total_samples
        } else {
            start + samples_per_segment
        };
        let seg_deltas = &deltas[start..end];
        let centroids_i32 = train_1d_codebook_signed(seg_deltas, codebook_size, 20);
        assert_eq!(centroids_i32.len(), codebook_size);
        let mut cb_i16: Vec<i16> = Vec::with_capacity(codebook_size);
        for &c in &centroids_i32 {
            let entry = encode_delta_entry(c);
            codebook_bytes.extend_from_slice(&entry);
            cb_i16.push(decode_delta_entry_i16(&entry));
        }
        cbs_i16.push(cb_i16);
    }

    // Closed-loop encoding: walk PCM with a running accumulator that
    // mirrors the target's reconstruction. For each sample, pick the
    // entry from the active segment's codebook whose resulting accum
    // is closest to the target sample. Prevents drift; closed-loop
    // matches open-loop quality for well-trained codebooks but is
    // strictly safer.
    // Per-sample closed-loop pick — same for 4-bit and 8-bit. We
    // collect the raw indices first and pack afterward (or not, for
    // 8-bit).
    let mut indices: Vec<u8> = Vec::with_capacity(pcm.len());
    let mut accum: i16 = 0;
    for (sample_idx, &target) in pcm.iter().enumerate() {
        let cb_i16 = &cbs_i16[segment_for(sample_idx)];
        let mut best = 0u8;
        let mut best_dist = i64::MAX;
        for (i, &delta) in cb_i16.iter().enumerate() {
            let cand = (accum as i32 + delta as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            let d = (target as i64 - cand as i64).abs();
            if d < best_dist {
                best_dist = d;
                best = i as u8;
            }
        }
        let chosen = cb_i16[best as usize];
        accum = (accum as i32 + chosen as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        indices.push(best);
    }
    let n_samples = indices.len() as u32;

    // Stream packing. Samples/byte = 8 / bits_per_sample. Packing
    // order puts the earliest sample in the **low** bits so the
    // target can peel samples off with `byte & mask` then `>>= bits`
    // — matches the IMA ADPCM low-nibble-first convention used
    // elsewhere in this codebase.
    //   2-bit: s0 = b[1:0], s1 = b[3:2], s2 = b[5:4], s3 = b[7:6]
    //   4-bit: s0 = b[3:0],                           s1 = b[7:4]
    //   8-bit: s0 = b[7:0]
    let stream: Vec<u8> = if bits_per_sample == 3 {
        pack_codes(&indices, 3)
    } else if bits_per_sample == 2 {
        let mut packed = Vec::with_capacity(indices.len().div_ceil(4));
        let mut i = 0;
        while i < indices.len() {
            let s0 = indices[i] & 0x03;
            let s1 = if i + 1 < indices.len() {
                indices[i + 1] & 0x03
            } else {
                0
            };
            let s2 = if i + 2 < indices.len() {
                indices[i + 2] & 0x03
            } else {
                0
            };
            let s3 = if i + 3 < indices.len() {
                indices[i + 3] & 0x03
            } else {
                0
            };
            packed.push(s0 | (s1 << 2) | (s2 << 4) | (s3 << 6));
            i += 4;
        }
        packed
    } else if bits_per_sample == 4 {
        let mut packed = Vec::with_capacity(indices.len().div_ceil(2));
        let mut i = 0;
        while i < indices.len() {
            let lo = indices[i] & 0x0F;
            let hi = if i + 1 < indices.len() {
                indices[i + 1] & 0x0F
            } else {
                // Odd-sample tail: pad with index 0 (≈ silent delta
                // for a Lloyd-trained codebook; never played back
                // anyway since the target stops at n_samples).
                0
            };
            packed.push(lo | (hi << 4));
            i += 2;
        }
        packed
    } else {
        indices
    };

    let header = DeltaQuantHeader {
        rate_hz: rate,
        n_samples,
        stream_bytes: stream.len() as u32,
        codebook_entries: codebook_size as u16,
        num_codebooks: num_codebooks as u16,
        bits_per_sample,
        channels: 1,
    };

    Ok(EncodedDeltaQuant {
        rate_actual: rate,
        n_samples,
        blob: DeltaBlob {
            compact_entries: false,
            header,
            codebook_bytes,
            stream,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generic_decode_honors_two_three_and_eight_bit_widths() {
        for bits in [2, 3, 8] {
            let pcm: Vec<_> = (0..39).map(|i| (i * 117 - 2048) as i16).collect();
            let encoded = encode_pcm_to_delta_quant(&pcm, 16000, 1 << bits, 3, bits).unwrap();
            let mut accumulator = 0i16;
            let stride = delta_samples_per_segment(39, encoded.blob.header.num_codebooks, bits);
            let mut expected = Vec::new();
            for index in 0..39 {
                let bit = index * bits as usize;
                let code = (0..bits as usize).fold(0usize, |code, shift| {
                    code | (((encoded.blob.stream[(bit + shift) / 8] >> ((bit + shift) % 8)) & 1)
                        as usize)
                        << shift
                });
                let segment = (index as u32 / stride)
                    .min(encoded.blob.header.num_codebooks as u32 - 1)
                    as usize;
                let at = (segment * (1 << bits) + code) * 3;
                accumulator = accumulator.saturating_add(decode_delta_entry_i16(
                    &encoded.blob.codebook_bytes[at..at + 3],
                ));
                expected.push(accumulator);
            }
            assert_eq!(encoded.blob.decode(), expected);
        }
    }

    #[test]
    fn stereo_matches_two_independent_encoders_including_odd_tail() {
        let pcm = [
            (0..1045)
                .map(|i| ((i * 137) % 20000 - 10000) as i16)
                .collect(),
            (0..1045).map(|i| ((i * 47) % 6000 - 3000) as i16).collect(),
        ];
        let stereo = encode_stereo(&pcm, 24_000).unwrap();
        let left = encode_pcm_to_delta_quant(&pcm[0], 24_000, 16, 521, 4).unwrap();
        let right = encode_pcm_to_delta_quant(&pcm[1], 24_000, 16, 521, 4).unwrap();
        let expected: Vec<_> = left
            .blob
            .decode()
            .into_iter()
            .zip(right.blob.decode())
            .flat_map(|(l, r)| [l, r])
            .collect();
        assert_eq!(stereo.blob.decode(), expected);
        assert_eq!(stereo.blob.stream.len(), 1045);
        assert_eq!(stereo.blob.header.num_codebooks, 521);
        assert_eq!(
            stereo.blob.to_bytes(),
            encode_stereo(&pcm, 24_000).unwrap().blob.to_bytes()
        );
        assert!(stereo.blob.decode().chunks_exact(2).any(|f| f[0] != f[1]));
    }
}
