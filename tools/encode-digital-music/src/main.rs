//! Reproducible normalized DLT3 stereo, using Second Reality's quantizer.
mod dlt4;
mod encoder;
mod format;
mod lloyd;
mod normalize;
mod resample;
mod som;

use anyhow::{bail, Context, Result};
use sha2::{Digest, Sha256};
use std::{
    env, fs,
    io::Write,
    path::Path,
    process::{Command, Stdio},
};

fn hash(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn read_source(path: &Path) -> Result<([Vec<i16>; 2], u32)> {
    let probe = Command::new("ffprobe")
        .args([
            "-v",
            "error",
            "-select_streams",
            "a:0",
            "-show_entries",
            "stream=sample_rate,channels",
            "-of",
            "csv=p=0",
        ])
        .arg(path)
        .output()
        .context("run ffprobe")?;
    if !probe.status.success() {
        bail!("ffprobe failed: {}", String::from_utf8_lossy(&probe.stderr));
    }
    let text = String::from_utf8(probe.stdout)?;
    let values: Vec<u32> = text
        .trim()
        .split(',')
        .map(str::parse)
        .collect::<Result<_, _>>()?;
    if values.len() != 2 || !(1..=2).contains(&values[1]) {
        bail!("expected mono or stereo FLAC");
    }
    let output = Command::new("ffmpeg")
        .args(["-v", "error", "-i"])
        .arg(path)
        .args(["-map", "0:a:0", "-f", "s16le", "-c:a", "pcm_s16le", "-"])
        .output()
        .context("decode FLAC through ffmpeg")?;
    if !output.status.success() {
        bail!(
            "FLAC decode failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
    let channels = values[1] as usize;
    if output.stdout.len() % (channels * 2) != 0 {
        bail!("truncated PCM frame");
    }
    let mut pcm = [Vec::new(), Vec::new()];
    for frame in output.stdout.chunks_exact(channels * 2) {
        let left = i16::from_le_bytes([frame[0], frame[1]]);
        let right = if channels == 2 {
            i16::from_le_bytes([frame[2], frame[3]])
        } else {
            left
        };
        pcm[0].push(left);
        pcm[1].push(right);
    }
    Ok((pcm, values[0]))
}

fn read_flac(path: &Path) -> Result<[Vec<f64>; 2]> {
    let (pcm, rate) = read_source(path)?;
    Ok(pcm
        .each_ref()
        .map(|channel| resample::resample_pcm(channel, rate, 24_000)))
}

fn pcm_bytes(samples: &[i16]) -> Vec<u8> {
    samples
        .iter()
        .flat_map(|sample| sample.to_le_bytes())
        .collect()
}

fn interleave(pcm: &[Vec<i16>; 2]) -> Vec<i16> {
    pcm[0]
        .iter()
        .zip(&pcm[1])
        .flat_map(|(&l, &r)| [l, r])
        .collect()
}

fn write_wav(path: &Path, samples: &[i16]) -> Result<()> {
    write_profile_wav(path, samples, 24_000, 2)
}

fn write_profile_wav(path: &Path, samples: &[i16], rate: u32, channels: u16) -> Result<()> {
    let mut out = fs::File::create(path)?;
    let size = (samples.len() * 2) as u32;
    out.write_all(b"RIFF")?;
    out.write_all(&(36 + size).to_le_bytes())?;
    out.write_all(b"WAVEfmt ")?;
    out.write_all(&16u32.to_le_bytes())?;
    out.write_all(&1u16.to_le_bytes())?;
    out.write_all(&channels.to_le_bytes())?;
    out.write_all(&rate.to_le_bytes())?;
    out.write_all(&(rate * channels as u32 * 2).to_le_bytes())?;
    out.write_all(&(channels * 2).to_le_bytes())?;
    out.write_all(&16u16.to_le_bytes())?;
    out.write_all(b"data")?;
    out.write_all(&size.to_le_bytes())?;
    out.write_all(&pcm_bytes(samples))?;
    Ok(())
}

fn measure(samples: &[i16]) -> Result<normalize::Meter> {
    measure_profile(samples, 24_000, 2)
}

fn measure_profile(samples: &[i16], rate: u32, channels: u16) -> Result<normalize::Meter> {
    let rate_arg = rate.to_string();
    let channels_arg = channels.to_string();
    let mut child = Command::new("ffmpeg")
        .args([
            "-hide_banner",
            "-nostats",
            "-v",
            "error",
            "-f",
            "s16le",
            "-ar",
            &rate_arg,
            "-ac",
            &channels_arg,
            "-i",
            "pipe:0",
            "-af",
            "ebur128=peak=true:metadata=1,ametadata=print:file=-",
            "-f",
            "null",
            "-",
        ])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .context("run FFmpeg EBU R128 true-peak meter")?;
    let mut stdin = child.stdin.take().unwrap();
    let mut bytes = pcm_bytes(samples);
    // Finish the last 100 ms metadata block and include reconstruction to silence.
    bytes.resize(bytes.len() + (rate / 10 * channels as u32 * 2) as usize, 0);
    let writer = std::thread::spawn(move || stdin.write_all(&bytes));
    let output = child.wait_with_output()?;
    writer
        .join()
        .map_err(|_| anyhow::anyhow!("PCM meter writer panicked"))??;
    if !output.status.success() {
        bail!(
            "true-peak meter failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
    parse_profile_meter(&String::from_utf8(output.stdout)?, channels)
}

#[cfg(test)]
fn parse_meter(text: &str) -> Result<normalize::Meter> {
    parse_profile_meter(text, 2)
}

fn parse_profile_meter(text: &str, channels: u16) -> Result<normalize::Meter> {
    let mut peaks = [None::<f64>; 2];
    let mut loudness = None;
    for line in text.lines() {
        for (channel, peak) in peaks.iter_mut().enumerate() {
            if let Some(value) = line.strip_prefix(&format!("lavfi.r128.true_peaks_ch{channel}=")) {
                let value: f64 = value.parse()?;
                *peak = Some(peak.unwrap_or(0.0).max(value));
            }
        }
        if let Some(value) = line.strip_prefix("lavfi.r128.I=") {
            loudness = Some(value.parse()?);
        }
    }
    let mut true_peak_dbtp = [0.0; 2];
    for channel in 0..channels as usize {
        let peak = peaks[channel].context("FFmpeg returned no per-channel true-peak metadata")?;
        // FFmpeg emits three decimal places for these linear amplitudes. Account
        // for half a unit of rounding, so a rounded-down value cannot pass falsely.
        true_peak_dbtp[channel] = normalize::db(if peak == 0.0 { 0.0 } else { peak + 0.0005 });
    }
    if channels == 1 {
        true_peak_dbtp[1] = true_peak_dbtp[0];
    }
    Ok(normalize::Meter {
        true_peak_dbtp,
        integrated_lufs: loudness.context("FFmpeg returned no integrated loudness")?,
    })
}

fn json_number(value: f64) -> String {
    if value.is_finite() {
        format!("{value:.6}")
    } else {
        "null".to_string()
    }
}

fn snr(reference: &[i16], decoded: &[i16]) -> f64 {
    let signal: f64 = reference.iter().map(|&s| (s as f64).powi(2)).sum();
    let noise: f64 = reference
        .iter()
        .zip(decoded)
        .map(|(&a, &b)| (a as f64 - b as f64).powi(2))
        .sum();
    10.0 * (signal / noise).log10()
}

fn main() -> Result<()> {
    let arguments: Vec<_> = env::args_os().collect();
    if arguments.get(1).is_some_and(|arg| arg == "--profile") {
        let usage = "usage: encode-digital-music --profile stereo20k3|mono16k3 [--codebooks COUNT] [--training baseline|beam-refine|som|combined] [--beam-width 32|64|128] [--reference-wav PATH --reference-metadata PATH] input.flac output.dlt preview.wav";
        if arguments.len() < 6 {
            bail!("{usage}");
        }
        let profile = dlt4::Profile::parse(&arguments[2].to_string_lossy())?;
        let mut books = 521;
        let mut options = dlt4::Options::default();
        let (mut reference_wav, mut reference_metadata) = (None, None);
        let mut first_path = 3;
        let mut seen = std::collections::HashSet::new();
        while first_path + 3 < arguments.len() {
            let option = arguments[first_path].to_string_lossy();
            if !seen.insert(option.to_string()) || first_path + 1 >= arguments.len() - 3 {
                bail!("{usage}");
            }
            let value = &arguments[first_path + 1];
            match option.as_ref() {
                "--codebooks" => {
                    books = value
                        .to_string_lossy()
                        .parse::<u16>()
                        .context("codebook count must be 1..=65535")?
                }
                "--training" => options.training = dlt4::Training::parse(&value.to_string_lossy())?,
                "--beam-width" => {
                    options.beam_width = value
                        .to_string_lossy()
                        .parse()
                        .context("invalid beam width")?
                }
                "--reference-wav" => reference_wav = Some(Path::new(value)),
                "--reference-metadata" => reference_metadata = Some(Path::new(value)),
                _ => bail!("{usage}"),
            }
            first_path += 2;
        }
        if first_path + 3 != arguments.len()
            || reference_wav.is_some() != reference_metadata.is_some()
            || books == 0
        {
            bail!("{usage}");
        }
        options.validate()?;
        return dlt4::run(
            profile,
            books,
            options,
            reference_wav.zip(reference_metadata),
            Path::new(&arguments[first_path]),
            Path::new(&arguments[first_path + 1]),
            Path::new(&arguments[first_path + 2]),
        );
    }
    if arguments.len() != 4 {
        bail!("usage: encode-digital-music input.flac output.dlt preview.wav");
    }
    let input = Path::new(&arguments[1]);
    let output = Path::new(&arguments[2]);
    let preview = Path::new(&arguments[3]);
    let pcm = read_flac(input)?;
    // No deliverable is opened until all normalization attempts and metering pass.
    let normalized = normalize::encode_normalized(&pcm, measure)?;
    let encoded = &normalized.encoded;
    let mut bytes = encoded.blob.to_bytes();
    let stream_offset = 32 + encoded.blob.codebook_bytes.len();
    let crc_offset = bytes.len();
    let mut crc = 0;
    for chunk in encoded.blob.stream.chunks(1024) {
        for &byte in chunk {
            crc = format::crc_byte(crc, byte);
        }
        bytes.push(crc);
    }
    let reference = interleave(&normalized.reference);
    let channel_metadata: Vec<_> =
        (0..2)
            .map(|channel| {
                let decoded: Vec<_> = normalized
                    .decoded
                    .iter()
                    .skip(channel)
                    .step_by(2)
                    .copied()
                    .collect();
                format!(
                    concat!("    {{\"channel\": \"{}\", \"input_sample_peak_dbfs\": {}, ",
            "\"normalized_sample_peak_dbfs\": {}, \"decoded_sample_peak_dbfs\": {}, ",
            "\"decoded_true_peak_upper_bound_dbtp\": {}, \"matched_level_snr_db\": {}}}"),
                    if channel == 0 { "left" } else { "right" },
                    json_number(normalize::db(
                        normalize::unclipped_peak(&pcm[channel]) / 32768.0
                    )),
                    json_number(normalize::db(
                        normalize::peak(&normalized.reference[channel]) / 32768.0
                    )),
                    json_number(normalize::db(normalize::peak(&decoded) / 32768.0)),
                    json_number(normalized.meter.true_peak_dbtp[channel]),
                    json_number(snr(&normalized.reference[channel], &decoded))
                )
            })
            .collect();
    let metadata = format!(concat!(
        "{{\n  \"format\": \"DLT3 with cumulative CRC-8 per 1024 stream bytes\",\n",
        "  \"sample_rate\": {},\n  \"sample_count\": {},\n  \"channels\": 2,\n  \"bits_per_sample\": 4,\n",
        "  \"codebook_entries\": 16,\n  \"codebook_count\": {},\n  \"samples_per_segment\": {},\n",
        "  \"header_bytes\": 32,\n  \"codebook_bytes\": {},\n  \"stream_offset\": {},\n",
        "  \"stream_bytes\": {},\n  \"crc_offset\": {},\n  \"crc_count\": {},\n  \"asset_bytes\": {},\n",
        "  \"header_checksum\": {},\n  \"codebook_checksum\": {},\n  \"crc_table_checksum\": {},\n",
        "  \"asset_checksum\": {},\n  \"asset_sha256\": \"{}\",\n  \"flac_sha256\": \"{}\",\n",
        "  \"reference_pcm_sha256\": \"{}\",\n  \"decoded_pcm_sha256\": \"{}\",\n",
        "  \"matched_level_snr_db\": {},\n  \"normalization_target_dbfs\": -1.0,\n",
        "  \"decoded_true_peak_limit_dbtp\": -1.0,\n  \"normalization_initial_gain_db\": {},\n",
        "  \"normalization_gain_db\": {},\n  \"normalization_attempts\": {},\n",
        "  \"decoded_integrated_lufs\": {},\n",
        "  \"true_peak_meter\": \"FFmpeg EBU R128; upper bounds include 0.0005 linear metadata rounding margin\",\n",
        "  \"channel_metrics\": [\n{}\n  ]\n}}\n"),
        encoded.rate_actual, encoded.n_samples, encoded.blob.header.num_codebooks,
        format::delta_samples_per_segment(encoded.n_samples, encoded.blob.header.num_codebooks, 4),
        encoded.blob.codebook_bytes.len(), stream_offset, encoded.blob.stream.len(), crc_offset,
        bytes.len() - crc_offset, bytes.len(), format::checksum(&bytes[..32]),
        format::checksum(&encoded.blob.codebook_bytes), format::checksum(&bytes[crc_offset..]),
        format::checksum(&bytes), hash(&bytes), hash(&fs::read(input)?), hash(&pcm_bytes(&reference)),
        hash(&pcm_bytes(&normalized.decoded)), json_number(snr(&reference, &normalized.decoded)),
        json_number(normalize::db(normalized.initial_gain)), json_number(normalize::db(normalized.gain)),
        normalized.attempts, json_number(normalized.meter.integrated_lufs), channel_metadata.join(",\n"));
    fs::write(output, &bytes)?;
    write_wav(preview, &normalized.decoded)?;
    fs::write(output.with_extension("json"), &metadata)?;
    print!("{metadata}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn true_peak_metadata_uses_each_channel_maximum_and_rounding_margin() {
        let meter = parse_meter(concat!(
            "lavfi.r128.true_peaks_ch0=0.891\n",
            "lavfi.r128.true_peaks_ch1=0.100\nlavfi.r128.I=-14.125\n",
            "lavfi.r128.true_peaks_ch0=0.890\nlavfi.r128.true_peaks_ch1=0.200\n"
        ))
        .unwrap();
        assert!(meter.true_peak_dbtp[0] > -1.0);
        assert_eq!(meter.true_peak_dbtp[1], normalize::db(0.2005));
        assert_eq!(meter.integrated_lufs, -14.125);
        assert!(parse_meter("").is_err());
        assert_eq!(json_number(f64::NEG_INFINITY), "null");
    }

    #[test]
    fn mono_meter_requires_only_its_one_real_channel() {
        let text = "lavfi.r128.true_peaks_ch0=0.500\nlavfi.r128.I=-20.0\n";
        let meter = parse_profile_meter(text, 1).unwrap();
        assert_eq!(meter.true_peak_dbtp, [normalize::db(0.5005); 2]);
        assert!(parse_profile_meter(text, 2).is_err());
    }
}
