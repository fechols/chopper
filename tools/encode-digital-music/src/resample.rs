// Ported from secondreality-opt/src/audio/flac_to_adpcm.rs (Unlicense).
// Sequential iteration preserves the original per-sample arithmetic.
pub fn resample_pcm(src: &[i16], src_rate: u32, dst_rate: u32) -> Vec<f64> {
    resample_f64(
        &src.iter().map(|&sample| sample as f64).collect::<Vec<_>>(),
        src_rate,
        dst_rate,
    )
}

/// Floating-point input preserves half-sample precision when downmixing stereo.
pub fn resample_f64(src: &[f64], src_rate: u32, dst_rate: u32) -> Vec<f64> {
    assert!(src_rate > 0 && dst_rate > 0, "sample rates must be nonzero");
    if src_rate == dst_rate || src.is_empty() {
        return src.to_vec();
    }
    const PHASES: usize = 512;
    let taps = resampler_tap_count(src_rate, dst_rate);
    let cutoff = 0.45 * src_rate.min(dst_rate) as f64 / src_rate as f64;
    let filters: Vec<Vec<f64>> = (0..=PHASES)
        .map(|phase| build_lpf_taps(taps, cutoff, phase as f64 / PHASES as f64))
        .collect();
    let half = (taps / 2) as isize;
    let n_out = (src.len() as u64 * dst_rate as u64).div_ceil(src_rate as u64) as usize;
    (0..n_out)
        .into_iter()
        .map(|i| {
            // Integer time arithmetic prevents long-song phase drift.
            let position = i as u64 * src_rate as u64;
            let center = (position / dst_rate as u64) as isize;
            let phase_position = (position % dst_rate as u64) * PHASES as u64;
            let phase = (phase_position / dst_rate as u64) as usize;
            let blend = (phase_position % dst_rate as u64) as f64 / dst_rate as f64;
            let a = &filters[phase];
            let b = &filters[phase + 1];
            let first = center - half;
            let lo = (-first).max(0) as usize;
            let hi = (src.len() as isize - first).min(taps as isize).max(0) as usize;
            let mut sum = 0.0;
            for k in lo..hi {
                let weight = a[k] + blend * (b[k] - a[k]);
                sum += src[(first + k as isize) as usize] * weight;
            }
            // Keep filter overshoot and fractional precision until the shared
            // normalization gain is applied; clipping here would be permanent.
            sum
        })
        .collect()
}

fn resampler_tap_count(src_rate: u32, dst_rate: u32) -> usize {
    let transition = 0.1 * src_rate.min(dst_rate) as f64 / src_rate as f64;
    // Kaiser estimate for 100 dB stopband attenuation; reserve additional
    // taps so the tested 80 dB specification has a comfortable margin.
    let order = ((100.0 - 8.0) / (2.285 * std::f64::consts::TAU * transition)).ceil() as usize;
    (order + 9) | 1
}

fn build_lpf_taps(n: usize, normalized_cutoff: f64, phase: f64) -> Vec<f64> {
    const BETA: f64 = 0.1102 * (100.0 - 8.7);
    let half = (n / 2) as f64;
    let normalizer = bessel_i0(BETA);
    let mut taps: Vec<f64> = (0..n)
        .map(|i| {
            let x = i as f64 - half - phase;
            if x.abs() > half {
                return 0.0;
            }
            let sinc = if x.abs() < 1e-12 {
                2.0 * normalized_cutoff
            } else {
                (std::f64::consts::TAU * normalized_cutoff * x).sin() / (std::f64::consts::PI * x)
            };
            let window = bessel_i0(BETA * (1.0 - (x / half).powi(2)).max(0.0).sqrt()) / normalizer;
            sinc * window
        })
        .collect();
    let sum: f64 = taps.iter().sum();
    for tap in &mut taps {
        *tap /= sum;
    }
    taps
}

fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    for k in 1..=64 {
        term *= (x * x * 0.25) / (k * k) as f64;
        sum += term;
        if term < sum * 1e-16 {
            break;
        }
    }
    sum
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn production_conversion_preserves_duration_and_dc_gain() {
        let samples = vec![10_000; 4800];
        let output = resample_pcm(&samples, 48_000, 24_000);
        assert_eq!(output.len(), 2400);
        assert!(output[100..2300]
            .iter()
            .all(|&value| (value - 10_000.0).abs() <= 2.0));
    }

    #[test]
    fn production_filter_rejects_frequencies_above_output_nyquist() {
        let input: Vec<i16> = (0..4800)
            .map(|i| {
                (10_000.0 * (std::f64::consts::TAU * 16_000.0 * i as f64 / 48_000.0).sin()) as i16
            })
            .collect();
        let output = resample_pcm(&input, 48_000, 24_000);
        let interior = &output[100..output.len() - 100];
        let rms = (interior.iter().map(|&x| (x as f64).powi(2)).sum::<f64>()
            / interior.len() as f64)
            .sqrt();
        assert!(
            rms < 1.0,
            "out-of-band signal leaked into decoded audio: {rms}"
        );
    }

    #[test]
    fn normalization_receives_unclipped_filter_overshoot() {
        let mut source = vec![i16::MIN; 480];
        source[240..].fill(i16::MAX);
        let filtered = resample_pcm(&source, 48_000, 24_000);
        assert!(filtered
            .iter()
            .any(|&s| s < i16::MIN as f64 || s > i16::MAX as f64));
        let pair = [filtered.clone(), filtered];
        let normalized = crate::normalize::apply_gain(&pair, crate::normalize::shared_gain(&pair));
        assert_eq!(crate::normalize::peak(&normalized[0]), 29204.0);
        assert!(normalized[0].iter().any(|&s| s > 0 && s < 29000));
    }
}
