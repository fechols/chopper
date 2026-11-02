//! One track-wide gain keeps the source stereo balance and musical dynamics.
use crate::encoder::{encode_stereo, EncodedDeltaQuant};
use anyhow::{bail, Result};

pub const TARGET_DB: f64 = -1.0;
pub const MAX_ATTEMPTS: usize = 5;

pub struct Meter {
    /// Conservative upper bounds for FFmpeg's rounded linear true-peak readings.
    pub true_peak_dbtp: [f64; 2],
    pub integrated_lufs: f64,
}

pub struct NormalizedEncoding {
    pub encoded: EncodedDeltaQuant,
    pub reference: [Vec<i16>; 2],
    pub decoded: Vec<i16>,
    pub initial_gain: f64,
    pub gain: f64,
    pub attempts: usize,
    pub meter: Meter,
}

pub fn db(amplitude: f64) -> f64 {
    20.0 * amplitude.log10()
}

pub fn peak(samples: &[i16]) -> f64 {
    samples
        .iter()
        .map(|&s| (s as i32).unsigned_abs())
        .max()
        .unwrap_or(0) as f64
}

pub fn unclipped_peak(samples: &[f64]) -> f64 {
    samples.iter().map(|s| s.abs()).fold(0.0, f64::max)
}

pub fn shared_gain(pcm: &[Vec<f64>; 2]) -> f64 {
    let source_peak = unclipped_peak(&pcm[0]).max(unclipped_peak(&pcm[1]));
    if source_peak == 0.0 {
        1.0
    } else {
        // Round down the target so integer PCM cannot cross -1 dBFS.
        (32_768.0 * 10f64.powf(TARGET_DB / 20.0)).floor() / source_peak
    }
}

pub fn apply_gain(pcm: &[Vec<f64>; 2], gain: f64) -> [Vec<i16>; 2] {
    pcm.each_ref().map(|channel| {
        channel
            .iter()
            .map(|&sample| {
                (sample * gain)
                    .round()
                    .clamp(i16::MIN as f64, i16::MAX as f64) as i16
            })
            .collect()
    })
}

pub fn encode_normalized(
    pcm: &[Vec<f64>; 2],
    mut measure: impl FnMut(&[i16]) -> Result<Meter>,
) -> Result<NormalizedEncoding> {
    let initial_gain = shared_gain(pcm);
    let mut gain = initial_gain;
    for attempts in 1..=MAX_ATTEMPTS {
        let reference = apply_gain(pcm, gain);
        let encoded = encode_stereo(&reference, 24_000)?;
        let decoded = encoded.blob.decode();
        let meter = measure(&decoded)?;
        let maximum = meter.true_peak_dbtp[0]
            .max(meter.true_peak_dbtp[1])
            .max(db(peak(&decoded) / 32_768.0));
        if maximum <= TARGET_DB {
            return Ok(NormalizedEncoding {
                encoded,
                reference,
                decoded,
                initial_gain,
                gain,
                attempts,
                meter,
            });
        }
        let reduction = maximum - TARGET_DB + 0.1;
        eprintln!("Normalization attempt {attempts}: decoded peak {maximum:.4} dB; reducing gain by {reduction:.4} dB");
        gain *= 10f64.powf(-reduction / 20.0);
    }
    bail!("decoded peak exceeds -1 dBTP after {MAX_ATTEMPTS} normalization attempts; outputs were not replaced")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_gain_preserves_channel_ratio_and_handles_signed_extrema() {
        let pcm = [vec![1000.0, -2000.0, 500.0], vec![-250.0, 500.0, -125.0]];
        let gain = shared_gain(&pcm);
        assert!(gain > 10.0);
        let scaled = apply_gain(&pcm, gain);
        assert!(db(peak(&scaled[0]) / 32768.0) <= TARGET_DB);
        for (&l, &r) in scaled[0].iter().zip(&scaled[1]) {
            assert!((l as i32 + 4 * r as i32).abs() <= 2);
        }
        let extremes = [vec![i16::MIN as f64], vec![i16::MAX as f64]];
        assert!(shared_gain(&extremes) < 1.0);
        let scaled = apply_gain(&extremes, shared_gain(&extremes));
        assert_eq!(scaled[0][0], -29204);
        assert!(peak(&scaled[1]) <= 29204.0);
    }

    #[test]
    fn silence_stays_silent_and_uses_unity_gain() {
        let pcm = [vec![0.0; 16], vec![0.0; 16]];
        let result = encode_normalized(&pcm, |_| {
            Ok(Meter {
                true_peak_dbtp: [f64::NEG_INFINITY; 2],
                integrated_lufs: -70.0,
            })
        })
        .unwrap();
        assert_eq!(result.gain, 1.0);
        assert_eq!(result.attempts, 1);
        assert!(result.decoded.iter().all(|&s| s == 0));
    }

    #[test]
    fn codec_overshoot_reduces_shared_gain_and_reencodes() {
        let pcm = [vec![1000.0; 16], vec![-250.0; 16]];
        let mut calls = 0;
        let result = encode_normalized(&pcm, |_| {
            calls += 1;
            Ok(Meter {
                true_peak_dbtp: if calls == 1 {
                    [-0.5, -8.0]
                } else {
                    [-1.1, -8.6]
                },
                integrated_lufs: -14.0,
            })
        })
        .unwrap();
        assert_eq!(calls, 2);
        assert!((db(result.gain / result.initial_gain) + 0.6).abs() < 1e-10);
        assert!(result.decoded.iter().all(|&s| (s as i32).abs() < 29204));
    }

    #[test]
    fn persistent_overshoot_fails_after_five_attempts() {
        let pcm = [vec![1000.0; 16], vec![-250.0; 16]];
        let mut calls = 0;
        let result = encode_normalized(&pcm, |_| {
            calls += 1;
            Ok(Meter {
                true_peak_dbtp: [0.0, -10.0],
                integrated_lufs: -14.0,
            })
        });
        assert!(result.is_err());
        assert_eq!(calls, MAX_ATTEMPTS);
    }
}
