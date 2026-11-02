//! Fixed-cost three-bit profiles. All adaptation happens in the host encoder.
use crate::{format::*, lloyd::train_1d_codebook_signed, normalize, resample};
use anyhow::{bail, Context, Result};
use std::{fs, path::Path};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Training {
    #[default]
    Baseline,
    BeamRefine,
    Som,
    Combined,
}

impl Training {
    pub fn parse(name: &str) -> Result<Self> {
        match name {
            "baseline" => Ok(Self::Baseline),
            "beam-refine" => Ok(Self::BeamRefine),
            "som" => Ok(Self::Som),
            "combined" => Ok(Self::Combined),
            _ => bail!("unknown training strategy {name}"),
        }
    }
    fn name(self) -> &'static str {
        match self {
            Self::Baseline => "baseline",
            Self::BeamRefine => "beam-refine",
            Self::Som => "som",
            Self::Combined => "combined",
        }
    }
    fn retraining_passes(self) -> usize {
        if matches!(self, Self::BeamRefine | Self::Combined) {
            2
        } else {
            0
        }
    }
    fn som(self) -> bool {
        matches!(self, Self::Som | Self::Combined)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Options {
    pub training: Training,
    pub beam_width: usize,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            training: Training::Baseline,
            beam_width: 32,
        }
    }
}

impl Options {
    pub fn validate(self) -> Result<()> {
        if ![32, 64, 128].contains(&self.beam_width) {
            bail!("beam width must be 32, 64, or 128");
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Profile {
    Stereo20k3,
    Mono16k3,
}

impl Profile {
    pub fn parse(name: &str) -> Result<Self> {
        match name {
            "stereo20k3" => Ok(Self::Stereo20k3),
            "mono16k3" => Ok(Self::Mono16k3),
            _ => bail!("unknown profile {name}; expected stereo20k3 or mono16k3"),
        }
    }
    fn name(self) -> &'static str {
        match self {
            Self::Stereo20k3 => "stereo20k3",
            Self::Mono16k3 => "mono16k3",
        }
    }
    fn rate(self) -> u32 {
        match self {
            Self::Stereo20k3 => 20_000,
            Self::Mono16k3 => 16_000,
        }
    }
    fn channels(self) -> usize {
        match self {
            Self::Stereo20k3 => 2,
            Self::Mono16k3 => 1,
        }
    }
}

fn prepare(pcm: &[Vec<i16>; 2], source_rate: u32, profile: Profile) -> Vec<Vec<f64>> {
    if profile == Profile::Mono16k3 {
        let mono: Vec<_> = pcm[0]
            .iter()
            .zip(&pcm[1])
            .map(|(&left, &right)| (f64::from(left) + f64::from(right)) * 0.5)
            .collect();
        vec![resample::resample_f64(&mono, source_rate, profile.rate())]
    } else {
        pcm.iter()
            .map(|channel| resample::resample_pcm(channel, source_rate, profile.rate()))
            .collect()
    }
}

#[derive(Debug, PartialEq, Eq)]
struct Segment {
    codes: Vec<u8>,
    residuals: Vec<i32>,
    end: i16,
    error: u64,
}

fn encode_segment(pcm: &[i16], book: &[i16], mut previous: i16) -> Segment {
    let mut codes = Vec::with_capacity(pcm.len());
    let mut residuals = Vec::with_capacity(pcm.len());
    let mut error = 0;
    for &target in pcm {
        residuals.push(i32::from(target) - i32::from(previous));
        let mut best = 0;
        let mut best_error = i32::MAX;
        for (index, &delta) in book.iter().enumerate() {
            let difference = (i32::from(target) - i32::from(previous.saturating_add(delta))).abs();
            if difference < best_error {
                best = index;
                best_error = difference;
            }
        }
        previous = previous.saturating_add(book[best]);
        let difference = i64::from(target) - i64::from(rounded_output(previous));
        error += (difference * difference) as u64;
        codes.push(best as u8);
    }
    Segment {
        codes,
        residuals,
        end: previous,
        error,
    }
}

fn train(residuals: &[i32]) -> Vec<i16> {
    train_1d_codebook_signed(residuals, 8, 20)
        .into_iter()
        .map(|delta| decode_delta_entry_i16(&encode_delta_entry(delta)))
        .collect()
}

#[derive(Clone, Copy, Default)]
struct Node {
    sample: i16,
    error: u64,
    parent: u8,
    code: u8,
}

impl Node {
    fn key(&self) -> (u64, i16, u8, u8) {
        (self.error, self.sample, self.parent, self.code)
    }
}

struct BeamScratch {
    positions: Vec<u16>,
    candidates: Vec<Node>,
}

impl BeamScratch {
    fn new() -> Self {
        Self {
            positions: vec![u16::MAX; 65536],
            candidates: Vec::with_capacity(1024),
        }
    }
}

/// Keep the same ordered best distinct states as sorting every child and then
/// deduplicating. Deduplicating before partial selection makes wider searches
/// affordable without changing error or tie rules.
fn encode_segment_beam<const WIDTH: usize>(
    pcm: &[i16],
    book: &[i16],
    previous: i16,
    scratch: &mut BeamScratch,
) -> Segment {
    let mut beam = vec![Node {
        sample: previous,
        ..Node::default()
    }];
    let mut paths = Vec::<[(u8, u8); WIDTH]>::with_capacity(pcm.len());
    for &target in pcm {
        let candidates = &mut scratch.candidates;
        candidates.clear();
        for (parent, node) in beam.iter().enumerate() {
            for (code, &delta) in book.iter().enumerate() {
                let sample = node.sample.saturating_add(delta);
                let difference = i64::from(target) - i64::from(rounded_output(sample));
                let node = Node {
                    sample,
                    error: node.error + (difference * difference) as u64,
                    parent: parent as u8,
                    code: code as u8,
                };
                let position = &mut scratch.positions[sample as u16 as usize];
                if *position == u16::MAX {
                    *position = candidates.len() as u16;
                    candidates.push(node);
                } else if node.key() < candidates[*position as usize].key() {
                    candidates[*position as usize] = node;
                }
            }
        }
        for node in candidates.iter() {
            scratch.positions[node.sample as u16 as usize] = u16::MAX;
        }
        if candidates.len() > WIDTH {
            candidates.select_nth_unstable_by_key(WIDTH, Node::key);
            candidates.truncate(WIDTH);
        }
        candidates.sort_unstable_by_key(Node::key);
        beam.clear();
        beam.extend_from_slice(candidates);
        let mut path = [(0, 0); WIDTH];
        for (index, node) in beam.iter().enumerate() {
            path[index] = (node.parent, node.code);
        }
        paths.push(path);
    }
    let mut codes = vec![0; pcm.len()];
    let mut index = 0;
    for frame in (0..pcm.len()).rev() {
        let (parent, code) = paths[frame][index];
        codes[frame] = code;
        index = parent as usize;
    }
    let mut residuals = Vec::with_capacity(pcm.len());
    let mut predictor = previous;
    for (&target, &code) in pcm.iter().zip(&codes) {
        residuals.push(i32::from(target) - i32::from(predictor));
        predictor = predictor.saturating_add(book[code as usize]);
    }
    Segment {
        codes,
        residuals,
        end: predictor,
        error: beam[0].error,
    }
}

fn search(
    pcm: &[i16],
    book: &[i16],
    previous: i16,
    width: usize,
    scratch: &mut BeamScratch,
) -> Segment {
    match width {
        32 => encode_segment_beam::<32>(pcm, book, previous, scratch),
        64 => encode_segment_beam::<64>(pcm, book, previous, scratch),
        128 => encode_segment_beam::<128>(pcm, book, previous, scratch),
        _ => unreachable!("validated beam width"),
    }
}

#[cfg(test)]
pub fn encode(pcm: &[Vec<i16>], profile: Profile, requested_books: u16) -> Result<DeltaBlob> {
    encode_with_options(pcm, profile, requested_books, Options::default())
}

pub fn encode_with_options(
    pcm: &[Vec<i16>],
    profile: Profile,
    requested_books: u16,
    options: Options,
) -> Result<DeltaBlob> {
    options.validate()?;
    let channels = profile.channels();
    if pcm.len() != channels || pcm[0].is_empty() || pcm.iter().any(|c| c.len() != pcm[0].len()) {
        bail!("profile requires nonempty, equal-length channels");
    }
    if requested_books == 0 {
        bail!("codebook count must be 1..=65535");
    }
    let frames: u32 = pcm[0].len().try_into()?;
    let segments = BalancedSegments::new(frames, requested_books, channels as u8);
    let books = segments.books;
    let mut predictors = [0i16; 2];
    let mut scratch = BeamScratch::new();
    let mut codes = vec![0u8; pcm[0].len() * channels];
    let mut codebook_bytes = Vec::with_capacity(books as usize * channels * 8 * COMPACT_ENTRY_SIZE);
    for segment in 0..books as usize {
        let range = segments.range(segment as u16);
        let (start, end) = (range.start, range.end);
        for channel in 0..channels {
            let source = &pcm[channel][start..end];
            let mut source_previous = if start == 0 {
                0
            } else {
                pcm[channel][start - 1]
            };
            let deltas: Vec<_> = source
                .iter()
                .map(|&sample| {
                    let delta = i32::from(sample) - i32::from(source_previous);
                    source_previous = sample;
                    delta
                })
                .collect();
            let mut book = train(&deltas);
            let mut best = encode_segment(source, &book, predictors[channel]);
            // Retrain on the actual closed-loop residuals, retaining an update
            // only when the final rounded DAC output has lower squared error.
            for _ in 0..3 {
                let candidate_book = train(&best.residuals);
                let candidate = encode_segment(source, &candidate_book, predictors[channel]);
                if candidate.error >= best.error {
                    break;
                }
                book = candidate_book;
                best = candidate;
            }
            let searched = search(
                source,
                &book,
                predictors[channel],
                options.beam_width,
                &mut scratch,
            );
            if searched.error < best.error {
                best = searched;
            }
            if options.training.som() {
                let candidate_book: Vec<_> = crate::som::train(&best.residuals)
                    .into_iter()
                    .map(|delta| decode_delta_entry_i16(&encode_delta_entry(delta)))
                    .collect();
                let candidate = search(
                    source,
                    &candidate_book,
                    predictors[channel],
                    options.beam_width,
                    &mut scratch,
                );
                if candidate.error < best.error {
                    book = candidate_book;
                    best = candidate;
                }
            }
            for _ in 0..options.training.retraining_passes() {
                let candidate_book = train(&best.residuals);
                let candidate = search(
                    source,
                    &candidate_book,
                    predictors[channel],
                    options.beam_width,
                    &mut scratch,
                );
                if candidate.error >= best.error {
                    break;
                }
                book = candidate_book;
                best = candidate;
            }
            for &delta in &book {
                codebook_bytes.extend_from_slice(&delta.to_le_bytes());
            }
            for (index, &code) in best.codes.iter().enumerate() {
                codes[(start + index) * channels + channel] = code;
            }
            predictors[channel] = best.end;
        }
    }
    let stream = pack_codes(&codes, 3);
    Ok(DeltaBlob {
        compact_entries: true,
        header: DeltaQuantHeader {
            rate_hz: profile.rate(),
            n_samples: frames,
            stream_bytes: stream.len().try_into()?,
            codebook_entries: 8,
            num_codebooks: books,
            bits_per_sample: 3,
            channels: channels as u8,
        },
        codebook_bytes,
        stream,
    })
}

fn interleave(pcm: &[Vec<i16>]) -> Vec<i16> {
    (0..pcm[0].len())
        .flat_map(|frame| pcm.iter().map(move |channel| channel[frame]))
        .collect()
}

fn read_reference_wav(
    bytes: &[u8],
    rate: u32,
    channels: usize,
    frames: usize,
) -> Result<Vec<Vec<i16>>> {
    if bytes.len() < 12 || &bytes[..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        bail!("reference must be a PCM WAV");
    }
    let riff_size = u32::from_le_bytes(bytes[4..8].try_into().unwrap()) as usize;
    if riff_size.checked_add(8) != Some(bytes.len()) {
        bail!("reference WAV has a truncated or extended RIFF extent");
    }
    let (mut format, mut data) = (None, None);
    let mut at = 12;
    while at + 8 <= bytes.len() {
        let size = u32::from_le_bytes(bytes[at + 4..at + 8].try_into().unwrap()) as usize;
        let end = (at + 8)
            .checked_add(size)
            .context("reference WAV chunk overflow")?;
        if end > bytes.len() {
            bail!("truncated reference WAV chunk");
        }
        match &bytes[at..at + 4] {
            b"fmt " => {
                if format.is_some() || size < 16 {
                    bail!("invalid reference WAV format chunk");
                }
                format = Some(&bytes[at + 8..end]);
            }
            b"data" => {
                if data.is_some() {
                    bail!("duplicate reference WAV data chunk");
                }
                data = Some(&bytes[at + 8..end]);
            }
            _ => {}
        }
        at = end
            .checked_add(size % 2)
            .context("reference WAV padding overflow")?;
    }
    if at != bytes.len() {
        bail!("invalid reference WAV trailing bytes");
    }
    let format = format.context("reference WAV missing format")?;
    let u16_at = |at| u16::from_le_bytes(format[at..at + 2].try_into().unwrap());
    let u32_at = |at| u32::from_le_bytes(format[at..at + 4].try_into().unwrap());
    if u16_at(0) != 1
        || usize::from(u16_at(2)) != channels
        || u32_at(4) != rate
        || u32_at(8) != rate * channels as u32 * 2
        || usize::from(u16_at(12)) != channels * 2
        || u16_at(14) != 16
    {
        bail!("reference WAV must match profile rate/channels and signed16 PCM");
    }
    let data = data.context("reference WAV missing PCM")?;
    if frames == 0 || frames.checked_mul(channels * 2) != Some(data.len()) {
        bail!("reference WAV frame count differs from metadata");
    }
    let mut result = vec![Vec::with_capacity(frames); channels];
    for frame in data.chunks_exact(channels * 2) {
        for channel in 0..channels {
            result[channel].push(i16::from_le_bytes([
                frame[channel * 2],
                frame[channel * 2 + 1],
            ]));
        }
    }
    Ok(result)
}

fn load_reference(
    profile: Profile,
    books: u16,
    input_hash: &str,
    wav: &Path,
    metadata: &Path,
) -> Result<(Vec<Vec<i16>>, serde_json::Value)> {
    let metadata: serde_json::Value = serde_json::from_slice(&fs::read(metadata)?)?;
    let reference = validate_reference(profile, books, input_hash, &fs::read(wav)?, &metadata)?;
    Ok((reference, metadata))
}

fn validate_reference(
    profile: Profile,
    books: u16,
    input_hash: &str,
    wav: &[u8],
    metadata: &serde_json::Value,
) -> Result<Vec<Vec<i16>>> {
    if metadata["profile"].as_str() != Some(profile.name())
        || metadata["sample_rate"].as_u64() != Some(u64::from(profile.rate()))
        || metadata["channels"].as_u64() != Some(profile.channels() as u64)
        || metadata["codebook_count"].as_u64() != Some(u64::from(books))
        || metadata["flac_sha256"].as_str() != Some(input_hash)
    {
        bail!("reference metadata does not match profile, codebook count, or authenticated source FLAC");
    }
    let frames: usize = metadata["sample_count"]
        .as_u64()
        .context("reference metadata missing frame count")?
        .try_into()?;
    let reference = read_reference_wav(wav, profile.rate(), profile.channels(), frames)?;
    if metadata["reference_pcm_sha256"].as_str()
        != Some(&crate::hash(&crate::pcm_bytes(&interleave(&reference))))
    {
        bail!("reference PCM hash differs from metadata");
    }
    for field in ["normalization_gain_db", "normalization_initial_gain_db"] {
        if !metadata[field].as_f64().is_some_and(f64::is_finite) {
            bail!("reference metadata has invalid {field}");
        }
    }
    if metadata["channel_metrics"].as_array().map(Vec::len) != Some(profile.channels()) {
        bail!("reference metadata has wrong channel metric count");
    }
    Ok(reference)
}

fn sums(reference: &[i16], decoded: &[i16]) -> (u128, u128) {
    reference
        .iter()
        .zip(decoded)
        .fold((0, 0), |(signal, error), (&a, &b)| {
            let a = i128::from(a);
            let difference = a - i128::from(b);
            (
                signal + (a * a) as u128,
                error + (difference * difference) as u128,
            )
        })
}

pub fn run(
    profile: Profile,
    requested_books: u16,
    options: Options,
    frozen: Option<(&Path, &Path)>,
    input: &Path,
    output: &Path,
    preview: &Path,
) -> Result<()> {
    options.validate()?;
    let input_hash = crate::hash(&fs::read(input)?);
    let frozen = frozen
        .map(|(wav, metadata)| load_reference(profile, requested_books, &input_hash, wav, metadata))
        .transpose()?;
    let pcm = if frozen.is_none() {
        let (source, source_rate) = crate::read_source(input)?;
        prepare(&source, source_rate, profile)
    } else {
        Vec::new()
    };
    let channels = profile.channels();
    let peak = pcm
        .iter()
        .map(|c| normalize::unclipped_peak(c))
        .fold(0.0, f64::max);
    let initial_gain = if let Some((_, metadata)) = &frozen {
        10f64.powf(metadata["normalization_initial_gain_db"].as_f64().unwrap() / 20.0)
    } else if peak == 0.0 {
        1.0
    } else {
        (32768.0 * 10f64.powf(-1.0 / 20.0)).floor() / peak
    };
    let mut gain = frozen.as_ref().map_or(initial_gain, |(_, metadata)| {
        10f64.powf(metadata["normalization_gain_db"].as_f64().unwrap() / 20.0)
    });
    let mut accepted = None;
    for attempt in 1..=normalize::MAX_ATTEMPTS {
        let reference: Vec<Vec<i16>> = if let Some((reference, _)) = &frozen {
            reference.clone()
        } else {
            pcm.iter()
                .map(|channel| {
                    channel
                        .iter()
                        .map(|&sample| {
                            (sample * gain)
                                .round()
                                .clamp(i16::MIN as f64, i16::MAX as f64)
                                as i16
                        })
                        .collect()
                })
                .collect()
        };
        let encoded = encode_with_options(&reference, profile, requested_books, options)?;
        let decoded: Vec<_> = encoded.decode().into_iter().map(rounded_output).collect();
        // Include a real loop boundary, not only the final transition to silence.
        let mut meter_pcm = decoded.clone();
        meter_pcm.extend_from_slice(
            &decoded[..decoded.len().min(profile.rate() as usize / 10 * channels)],
        );
        let meter = crate::measure_profile(&meter_pcm, profile.rate(), channels as u16)?;
        let maximum = meter
            .true_peak_dbtp
            .iter()
            .copied()
            .fold(normalize::db(normalize::peak(&decoded) / 32768.0), f64::max);
        if maximum <= normalize::TARGET_DB {
            accepted = Some((encoded, reference, decoded, meter, attempt));
            break;
        }
        if frozen.is_some() {
            bail!("FROZEN_REFERENCE_PEAK_LIMIT: decoded peak {maximum:.6} dB exceeds -1 dBTP; frozen gain retained and outputs were not replaced");
        }
        let reduction = maximum - normalize::TARGET_DB + 0.1;
        eprintln!("Normalization attempt {attempt}: decoded peak {maximum:.4} dB; reducing gain by {reduction:.4} dB");
        gain *= 10f64.powf(-reduction / 20.0);
    }
    let Some((encoded, reference, decoded, meter, attempts)) = accepted else {
        bail!(
            "decoded peak exceeds -1 dBTP after {} attempts; outputs were not replaced",
            normalize::MAX_ATTEMPTS
        );
    };
    let mut bytes = encoded.to_bytes();
    let stream_offset = 32 + encoded.codebook_bytes.len();
    let crc_offset = bytes.len();
    let mut crc = 0;
    for chunk in encoded.stream.chunks(1024) {
        for &byte in chunk {
            crc = crc_byte(crc, byte);
        }
        bytes.push(crc);
    }
    let channel_metrics: Vec<_> =
        (0..channels)
            .map(|channel| {
                let samples: Vec<_> = decoded
                    .iter()
                    .skip(channel)
                    .step_by(channels)
                    .copied()
                    .collect();
                format!(
                    concat!("    {{\"channel\": \"{}\", \"input_sample_peak_dbfs\": {}, ",
            "\"normalized_sample_peak_dbfs\": {}, \"decoded_sample_peak_dbfs\": {}, ",
            "\"decoded_true_peak_upper_bound_dbtp\": {}, \"matched_level_snr_db\": {}}}"),
                    if channels == 1 {
                        "mono"
                    } else if channel == 0 {
                        "left"
                    } else {
                        "right"
                    },
                    if let Some((_, metadata)) = &frozen {
                        metadata["channel_metrics"][channel]["input_sample_peak_dbfs"].to_string()
                    } else {
                        crate::json_number(normalize::db(
                            normalize::unclipped_peak(&pcm[channel]) / 32768.0,
                        ))
                    },
                    crate::json_number(normalize::db(
                        normalize::peak(&reference[channel]) / 32768.0
                    )),
                    crate::json_number(normalize::db(normalize::peak(&samples) / 32768.0)),
                    crate::json_number(meter.true_peak_dbtp[channel]),
                    crate::json_number(crate::snr(&reference[channel], &samples))
                )
            })
            .collect();
    let reference = interleave(&reference);
    let metadata = format!(concat!(
        "{{\n  \"format\": \"DLT5 with cumulative CRC-8 per 1024 stream bytes\",\n",
        "  \"profile\": \"{}\",\n  \"sample_rate\": {},\n  \"sample_count\": {},\n",
        "  \"channels\": {},\n  \"bits_per_sample\": 3,\n  \"output_bits_per_sample\": 8,\n",
        "  \"codebook_entries\": 8,\n  \"codebook_count\": {},\n  \"samples_per_segment\": {},\n  \"long_segment_count\": {},\n  \"codebook_entry_bytes\": 2,\n",
        "  \"header_bytes\": 32,\n  \"codebook_bytes\": {},\n  \"stream_offset\": {},\n",
        "  \"stream_bytes\": {},\n  \"crc_offset\": {},\n  \"crc_count\": {},\n  \"asset_bytes\": {},\n",
        "  \"header_checksum\": {},\n  \"codebook_checksum\": {},\n  \"crc_table_checksum\": {},\n",
        "  \"asset_checksum\": {},\n  \"asset_sha256\": \"{}\",\n  \"flac_sha256\": \"{}\",\n",
        "  \"reference_pcm_sha256\": \"{}\",\n  \"decoded_pcm_sha256\": \"{}\",\n",
        "  \"matched_level_snr_db\": {},\n  \"normalization_target_dbfs\": -1.0,\n",
        "  \"decoded_true_peak_limit_dbtp\": -1.0,\n  \"normalization_initial_gain_db\": {},\n",
        "  \"normalization_gain_db\": {},\n  \"normalization_attempts\": {},\n",
        "  \"decoded_integrated_lufs\": {},\n  \"closed_loop_training_passes\": 3,\n  \"encoder_beam_width\": 32,\n",
        "  \"true_peak_meter\": \"FFmpeg EBU R128; loop seam included; upper bounds include 0.0005 linear metadata rounding margin\",\n",
        "  \"channel_metrics\": [\n{}\n  ]\n}}\n"),
        profile.name(), profile.rate(), encoded.header.n_samples, channels, encoded.header.num_codebooks,
        BalancedSegments::new(encoded.header.n_samples, encoded.header.num_codebooks, channels as u8).samples_per_segment,
        BalancedSegments::new(encoded.header.n_samples, encoded.header.num_codebooks, channels as u8).long_segment_count,
        encoded.codebook_bytes.len(), stream_offset, encoded.stream.len(), crc_offset, bytes.len() - crc_offset, bytes.len(),
        checksum(&bytes[..32]), checksum(&encoded.codebook_bytes), checksum(&bytes[crc_offset..]), checksum(&bytes),
        crate::hash(&bytes), input_hash, crate::hash(&crate::pcm_bytes(&reference)), crate::hash(&crate::pcm_bytes(&decoded)),
        crate::json_number(crate::snr(&reference, &decoded)), crate::json_number(normalize::db(initial_gain)), crate::json_number(normalize::db(gain)),
        attempts, crate::json_number(meter.integrated_lufs), channel_metrics.join(",\n"));
    let mut metadata: serde_json::Value = serde_json::from_str(&metadata)?;
    metadata["encoder_training"] = options.training.name().into();
    metadata["encoder_beam_width"] = options.beam_width.into();
    metadata["beam_retraining_passes"] = options.training.retraining_passes().into();
    metadata["som_trainer"] = if options.training.som() {
        crate::som::TRAINER.into()
    } else {
        serde_json::Value::Null
    };
    metadata["fixed_reference"] = frozen.is_some().into();
    metadata["encoding_attempts"] = attempts.into();
    if let Some((_, original)) = &frozen {
        for field in [
            "normalization_initial_gain_db",
            "normalization_gain_db",
            "normalization_attempts",
        ] {
            metadata[field] = original[field].clone();
        }
        for field in [
            "normalization_initial_gain_linear",
            "normalization_gain_linear",
        ] {
            if let Some(value) = original.get(field) {
                metadata[field] = value.clone();
            }
        }
    } else {
        metadata["normalization_initial_gain_linear"] = initial_gain.into();
        metadata["normalization_gain_linear"] = gain.into();
    }
    let (signal, error) = sums(&reference, &decoded);
    metadata["signal_sum_squares"] = serde_json::json!(signal);
    metadata["error_sum_squares"] = serde_json::json!(error);
    for channel in 0..channels {
        let channel_reference: Vec<_> = reference
            .iter()
            .skip(channel)
            .step_by(channels)
            .copied()
            .collect();
        let channel_decoded: Vec<_> = decoded
            .iter()
            .skip(channel)
            .step_by(channels)
            .copied()
            .collect();
        let (signal, error) = sums(&channel_reference, &channel_decoded);
        metadata["channel_metrics"][channel]["signal_sum_squares"] = serde_json::json!(signal);
        metadata["channel_metrics"][channel]["error_sum_squares"] = serde_json::json!(error);
    }
    let metadata = serde_json::to_string_pretty(&metadata)? + "\n";
    fs::write(output, &bytes)?;
    crate::write_profile_wav(preview, &decoded, profile.rate(), channels as u16)?;
    fs::write(output.with_extension("json"), &metadata)?;
    // Reference WAV distinguishes coding error from downmix/resampling changes.
    crate::write_profile_wav(
        &preview.with_extension("reference.wav"),
        &reference,
        profile.rate(),
        channels as u16,
    )?;
    if channels == 1 {
        let delay = profile.rate() as usize / 100;
        let delayed: Vec<_> = (0..decoded.len() + delay)
            .flat_map(|frame| {
                [
                    decoded.get(frame).copied().unwrap_or(0),
                    frame
                        .checked_sub(delay)
                        .and_then(|at| decoded.get(at))
                        .copied()
                        .unwrap_or(0),
                ]
            })
            .collect();
        crate::write_profile_wav(
            &preview.with_extension("delayed.wav"),
            &delayed,
            profile.rate(),
            2,
        )?;
    }
    print!("{metadata}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Original full-sort/deduplicate algorithm, kept independent of the optimized
    // selection to verify all states, ties, paths, and reconstructed residuals.
    fn reference_beam(pcm: &[i16], book: &[i16], previous: i16, width: usize) -> Segment {
        let mut beam = vec![Node {
            sample: previous,
            ..Node::default()
        }];
        let mut paths = Vec::new();
        for &target in pcm {
            let mut candidates = Vec::new();
            for (parent, node) in beam.iter().enumerate() {
                for (code, &delta) in book.iter().enumerate() {
                    let sample = node.sample.saturating_add(delta);
                    let difference = i64::from(target) - i64::from(rounded_output(sample));
                    candidates.push(Node {
                        sample,
                        error: node.error + (difference * difference) as u64,
                        parent: parent as u8,
                        code: code as u8,
                    });
                }
            }
            candidates.sort_unstable_by_key(Node::key);
            beam.clear();
            for node in candidates {
                if beam.iter().all(|other| other.sample != node.sample) {
                    beam.push(node);
                    if beam.len() == width {
                        break;
                    }
                }
            }
            paths.push(
                beam.iter()
                    .map(|node| (node.parent, node.code))
                    .collect::<Vec<_>>(),
            );
        }
        let mut codes = vec![0; pcm.len()];
        let mut index = 0;
        for frame in (0..pcm.len()).rev() {
            let (parent, code) = paths[frame][index];
            codes[frame] = code;
            index = usize::from(parent);
        }
        let mut residuals = Vec::new();
        let mut predictor = previous;
        for (&target, &code) in pcm.iter().zip(&codes) {
            residuals.push(i32::from(target) - i32::from(predictor));
            predictor = predictor.saturating_add(book[usize::from(code)]);
        }
        Segment {
            codes,
            residuals,
            end: predictor,
            error: beam[0].error,
        }
    }

    #[test]
    fn optimized_beam_matches_original_sort_for_all_widths_ties_and_saturation() {
        let books = [
            [-32768, -17000, -257, -1, 0, 1, 17000, 32767],
            [0; 8],
            [-256, -256, -128, 0, 0, 128, 256, 256],
            [-2048, -1024, -512, -256, 256, 512, 1024, 2048],
        ];
        let mut scratch = BeamScratch::new();
        for width in [32, 64, 128] {
            for book in &books {
                for previous in [i16::MIN, -128, 0, 128, i16::MAX] {
                    let pcm: Vec<_> = (0..41)
                        .map(|i| {
                            if i % 7 == 0 {
                                i16::MAX
                            } else if i % 11 == 0 {
                                i16::MIN
                            } else {
                                ((i * 23497 + 8193) % 65536) as u16 as i16
                            }
                        })
                        .collect();
                    assert_eq!(
                        search(&pcm, book, previous, width, &mut scratch),
                        reference_beam(&pcm, book, previous, width)
                    );
                }
            }
        }
        assert!(scratch
            .positions
            .iter()
            .all(|&position| position == u16::MAX));
    }

    #[test]
    fn experimental_modes_are_deterministic_and_keep_format_extents() {
        for profile in [Profile::Mono16k3, Profile::Stereo20k3] {
            let pcm: Vec<Vec<_>> = (0..profile.channels())
                .map(|channel| {
                    (0..113)
                        .map(|i| ((i * 1237 + channel * 2749) % 32768) as i16 - 16384)
                        .collect()
                })
                .collect();
            let baseline = encode(&pcm, profile, 5).unwrap();
            for training in [
                Training::Baseline,
                Training::BeamRefine,
                Training::Som,
                Training::Combined,
            ] {
                for beam_width in [32, 64, 128] {
                    let options = Options {
                        training,
                        beam_width,
                    };
                    let result = encode_with_options(&pcm, profile, 5, options).unwrap();
                    assert_eq!(
                        result.to_bytes(),
                        encode_with_options(&pcm, profile, 5, options)
                            .unwrap()
                            .to_bytes()
                    );
                    assert_eq!(result.to_bytes().len(), baseline.to_bytes().len());
                    assert_eq!(result.decode().len(), 113 * profile.channels());
                    if training == Training::Baseline && beam_width == 32 {
                        assert_eq!(result.to_bytes(), baseline.to_bytes());
                    }
                }
            }
        }
        assert!(Options {
            beam_width: 256,
            ..Options::default()
        }
        .validate()
        .is_err());
    }

    #[test]
    fn exact_error_totals_include_both_signed_extrema() {
        assert_eq!(
            sums(&[-32768, 32767], &[32767, -32768]),
            (32768u128.pow(2) + 32767u128.pow(2), 2 * 65535u128.pow(2))
        );
    }

    fn wav_bytes(pcm: &[i16], rate: u32, channels: u16) -> Vec<u8> {
        let data = crate::pcm_bytes(pcm);
        let mut bytes = b"RIFF".to_vec();
        bytes.extend_from_slice(&(36 + data.len() as u32).to_le_bytes());
        bytes.extend_from_slice(b"WAVEfmt ");
        bytes.extend_from_slice(&16u32.to_le_bytes());
        bytes.extend_from_slice(&1u16.to_le_bytes());
        bytes.extend_from_slice(&channels.to_le_bytes());
        bytes.extend_from_slice(&rate.to_le_bytes());
        bytes.extend_from_slice(&(rate * u32::from(channels) * 2).to_le_bytes());
        bytes.extend_from_slice(&(channels * 2).to_le_bytes());
        bytes.extend_from_slice(&16u16.to_le_bytes());
        bytes.extend_from_slice(b"data");
        bytes.extend_from_slice(&(data.len() as u32).to_le_bytes());
        bytes.extend(data);
        bytes
    }

    #[test]
    fn frozen_reference_requires_exact_wav_shape_and_length() {
        let pcm = [-32768, 32767, 0, 256];
        let bytes = wav_bytes(&pcm, 20000, 2);
        assert_eq!(
            read_reference_wav(&bytes, 20000, 2, 2).unwrap(),
            vec![vec![-32768, 0], vec![32767, 256]]
        );
        assert!(read_reference_wav(&bytes, 16000, 2, 2).is_err());
        assert!(read_reference_wav(&bytes, 20000, 1, 4).is_err());
        assert!(read_reference_wav(&bytes, 20000, 2, 3).is_err());
        assert!(read_reference_wav(&bytes[..bytes.len() - 1], 20000, 2, 2).is_err());
        let mut extended = bytes.clone();
        extended.push(0);
        assert!(read_reference_wav(&extended, 20000, 2, 2).is_err());
    }

    #[test]
    fn frozen_reference_authenticates_pcm_source_profile_and_gain_metadata() {
        let pcm = [-1234, 4567, 0, 256];
        let wav = wav_bytes(&pcm, 16000, 1);
        let metadata = serde_json::json!({
            "profile": "mono16k3", "sample_rate": 16000, "channels": 1,
            "sample_count": 4, "codebook_count": 1, "flac_sha256": "source-hash",
            "reference_pcm_sha256": crate::hash(&crate::pcm_bytes(&pcm)),
            "normalization_gain_db": 1.25, "normalization_initial_gain_db": 2.0,
            "channel_metrics": [{"channel": "mono", "input_sample_peak_dbfs": -4.0}],
        });
        let validate = |wav: &[u8], metadata: &serde_json::Value| {
            validate_reference(Profile::Mono16k3, 1, "source-hash", wav, metadata)
        };
        assert_eq!(validate(&wav, &metadata).unwrap(), vec![pcm.to_vec()]);
        for (field, replacement) in [
            ("profile", serde_json::json!("stereo20k3")),
            ("sample_rate", serde_json::json!(20000)),
            ("channels", serde_json::json!(2)),
            ("sample_count", serde_json::json!(5)),
            ("codebook_count", serde_json::json!(2)),
            ("flac_sha256", serde_json::json!("different-source")),
            (
                "reference_pcm_sha256",
                serde_json::json!("different-reference"),
            ),
            ("normalization_gain_db", serde_json::Value::Null),
            ("channel_metrics", serde_json::json!([])),
        ] {
            let mut corrupted = metadata.clone();
            corrupted[field] = replacement;
            assert!(
                validate(&wav, &corrupted).is_err(),
                "accepted corrupt {field}"
            );
        }
        let mut corrupted = wav.clone();
        *corrupted.last_mut().unwrap() ^= 1;
        assert!(validate(&corrupted, &metadata).is_err());
    }

    #[test]
    fn beam_search_finds_the_best_two_sample_path_without_changing_decoder() {
        let book = [-2000, -1000, -500, -100, 100, 500, 1000, 2000];
        let pcm = [780, -1400];
        let result = encode_segment_beam::<32>(&pcm, &book, 0, &mut BeamScratch::new());
        let mut exhaustive = u64::MAX;
        for &first in &book {
            for &second in &book {
                let a = i64::from(pcm[0]) - i64::from(rounded_output(first));
                let b = i64::from(pcm[1]) - i64::from(rounded_output(first.saturating_add(second)));
                exhaustive = exhaustive.min((a * a + b * b) as u64);
            }
        }
        assert_eq!(result.error, exhaustive);
    }

    #[test]
    fn downmix_keeps_fractional_samples_before_filtering() {
        let pcm = [vec![0, 32767, -32768], vec![1, -32768, 32767]];
        assert_eq!(
            prepare(&pcm, 16000, Profile::Mono16k3)[0],
            [0.5, -0.5, -0.5]
        );
    }

    #[test]
    fn independent_bit_oracle_matches_both_profiles_and_partial_tails() {
        for profile in [Profile::Mono16k3, Profile::Stereo20k3] {
            for frames in [1, 3, 7, 8, 9, 31, 113] {
                let pcm: Vec<Vec<_>> = (0..profile.channels())
                    .map(|c| {
                        (0..frames)
                            .map(|n| (((n * (137 + c * 193)) % 16000) as i16) - 8000)
                            .collect()
                    })
                    .collect();
                let blob = encode(&pcm, profile, 3).unwrap();
                let mut accum = [0i16; 2];
                let mut expected = Vec::new();
                // Independent partition oracle: distribute complete packing groups,
                // then clip only the final group to the exact source length.
                let alignment = 8 / profile.channels();
                let groups = frames.div_ceil(alignment);
                let books = blob.header.num_codebooks as usize;
                let mut boundaries = Vec::with_capacity(books);
                let mut boundary = 0;
                for book in 0..books {
                    boundary += (groups / books + usize::from(book < groups % books)) * alignment;
                    boundaries.push(boundary.min(frames));
                }
                for frame in 0..frames {
                    let segment = boundaries.iter().position(|&end| frame < end).unwrap();
                    for channel in 0..profile.channels() {
                        let bit = (frame * profile.channels() + channel) * 3;
                        let code = (0..3).fold(0usize, |code, shift| {
                            code | (((blob.stream[(bit + shift) / 8] >> ((bit + shift) % 8)) & 1)
                                as usize)
                                << shift
                        });
                        let at = ((segment * profile.channels() + channel) * 8 + code) * 2;
                        let delta = i16::from_le_bytes([
                            blob.codebook_bytes[at],
                            blob.codebook_bytes[at + 1],
                        ]) as i32;
                        accum[channel] =
                            (i32::from(accum[channel]) + delta).clamp(-32768, 32767) as i16;
                        expected.push(accum[channel]);
                    }
                }
                assert_eq!(blob.decode(), expected);
                assert_eq!(&blob.to_bytes()[..4], b"DLT5");
                assert_eq!(blob.codebook_bytes.len(), books * profile.channels() * 16);
                assert_eq!(
                    blob.stream.len(),
                    (frames * profile.channels() * 3).div_ceil(8)
                );
                let used = frames * profile.channels() * 3 % 8;
                if used != 0 {
                    assert_eq!(blob.stream.last().unwrap() >> used, 0);
                }
            }
        }
    }

    #[test]
    fn accepts_more_than_1024_books_and_caps_at_partial_packing_groups() {
        let pcm = vec![vec![0; 8193]];
        let blob = encode(&pcm, Profile::Mono16k3, u16::MAX).unwrap();
        assert_eq!(blob.header.num_codebooks, 1025);
        assert_eq!(blob.codebook_bytes.len(), 1025 * 16);
        assert_eq!(blob.decode(), vec![0; 8193]);
        assert!(encode(&pcm, Profile::Mono16k3, 0).is_err());
        assert_eq!(
            blob.to_bytes(),
            encode(&pcm, Profile::Mono16k3, u16::MAX)
                .unwrap()
                .to_bytes()
        );
    }
}
