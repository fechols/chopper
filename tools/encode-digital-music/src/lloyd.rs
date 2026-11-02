//! 1-D Lloyd's algorithm (k-means on scalar values), signed version.
//!
//! Used by the audio delta-quantization encoder to learn a 256-entry
//! codebook of signed deltas tailored to the current song. Built
//! offline during `build-video`; not perf-critical, but the histogram
//! bucketing makes it sub-second anyway (6.5 M samples → ~5-20 k
//! distinct delta values after bucketing).
//!
//! Algorithm:
//! 1. Bucket samples into a frequency histogram (distinct value → count).
//! 2. Seed `k` centroids via equal-count quantiles over the sorted
//!    distinct values.
//! 3. Assignment: for each (value, count) bucket, find the nearest
//!    centroid; accumulate weighted sum and count into that cell.
//! 4. Update: centroid = Σ(v·count) / Σ(count) for its cell.
//! 5. Split-the-largest rescue for any empty cell.
//! 6. Repeat until centroids are stable, or `max_iters` hit.

use std::collections::HashMap;

/// Train a `k`-entry codebook on signed i32 samples via Lloyd's
/// algorithm. Returns the centroid values sorted ascending.
pub fn train_1d_codebook_signed(samples: &[i32], k: usize, max_iters: usize) -> Vec<i32> {
    assert!(k > 0);
    if samples.is_empty() {
        return vec![0i32; k];
    }

    // Histogram bucketing: distinct value → count.
    let mut hist: HashMap<i32, u64> = HashMap::with_capacity(samples.len() / 100);
    for &s in samples {
        *hist.entry(s).or_insert(0) += 1;
    }
    let mut keys: Vec<i32> = hist.keys().copied().collect();
    keys.sort();

    if keys.len() <= k {
        // Fewer distinct values than cells — return them directly,
        // padding with repeats of the last value.
        let mut out = keys.clone();
        while out.len() < k {
            out.push(*keys.last().unwrap());
        }
        return out;
    }

    // Seed centroids via equal-count quantiles over the sorted keys.
    // Each centroid i gets the key at the (i + 0.5) / k fractional
    // count position — spreads seeds by cumulative mass, not by range,
    // so a long-tail histogram still gets useful coverage near the
    // common values instead of one seed per empty extreme.
    let total_count: u64 = keys.iter().map(|v| hist[v]).sum();
    let mut centroids: Vec<i32> = Vec::with_capacity(k);
    let mut cum: u64 = 0;
    let mut key_idx = 0usize;
    for i in 0..k {
        let target = ((i as u64 * 2 + 1) * total_count) / (2 * k as u64);
        while key_idx + 1 < keys.len() {
            let next_cum = cum + hist[&keys[key_idx]];
            if next_cum > target {
                break;
            }
            cum = next_cum;
            key_idx += 1;
        }
        centroids.push(keys[key_idx]);
    }
    dedupe_adjacent_and_spread(&mut centroids, &keys);

    // Lloyd iteration.
    for _ in 0..max_iters {
        let mut num: Vec<i128> = vec![0i128; k];
        let mut den: Vec<u64> = vec![0u64; k];

        for &v in &keys {
            let c = hist[&v];
            let idx = nearest_centroid_signed(&centroids, v);
            num[idx] = num[idx].saturating_add(v as i128 * c as i128);
            den[idx] = den[idx].saturating_add(c);
        }

        let mut changed = false;
        for i in 0..k {
            if den[i] > 0 {
                let new_c = (num[i] / den[i] as i128) as i32;
                if new_c != centroids[i] {
                    centroids[i] = new_c;
                    changed = true;
                }
            }
        }

        // Split-the-largest rescue: any empty cell steals from the
        // highest-count cell by nudging its centroid +/-1.
        for i in 0..k {
            if den[i] == 0 {
                let heavy = den
                    .iter()
                    .enumerate()
                    .max_by_key(|(_, d)| *d)
                    .map(|(idx, _)| idx)
                    .unwrap_or(0);
                centroids[i] = centroids[heavy].saturating_add(1);
                changed = true;
            }
        }

        if !changed {
            break;
        }
    }

    centroids.sort();
    centroids
}

/// Linear scan for the nearest centroid to a signed value. Sorted
/// input would allow binary search, but k=256 with a tight inner loop
/// is already fast enough and this is called from a histogram loop
/// with at most ~20 k iterations.
pub fn nearest_centroid_signed(centroids: &[i32], value: i32) -> usize {
    let mut best = 0usize;
    let mut best_dist = i64::MAX;
    for (i, &c) in centroids.iter().enumerate() {
        let d = (value as i64 - c as i64).abs();
        if d < best_dist {
            best_dist = d;
            best = i;
        }
    }
    best
}

/// If the quantile seeding produced duplicate adjacent centroids
/// (common when a single delta value dominates the histogram), nudge
/// duplicates to unique nearby keys so Lloyd iteration has room to
/// move. No-op when all seeds are unique.
fn dedupe_adjacent_and_spread(centroids: &mut Vec<i32>, keys: &[i32]) {
    let mut seen = std::collections::HashSet::new();
    for i in 0..centroids.len() {
        if !seen.insert(centroids[i]) {
            // Find the nearest key we haven't taken yet.
            let mut offset = 1usize;
            loop {
                let up_idx = keys
                    .binary_search(&centroids[i])
                    .unwrap_or_else(|x| x)
                    .saturating_add(offset);
                if up_idx < keys.len() && seen.insert(keys[up_idx]) {
                    centroids[i] = keys[up_idx];
                    break;
                }
                let down_idx = keys
                    .binary_search(&centroids[i])
                    .unwrap_or_else(|x| x)
                    .saturating_sub(offset);
                if down_idx < keys.len() && seen.insert(keys[down_idx]) {
                    centroids[i] = keys[down_idx];
                    break;
                }
                offset += 1;
                if offset > keys.len() {
                    // Give up — cluster keys aren't sufficiently
                    // varied for full k-way uniqueness. Lloyd will
                    // still converge; duplicates just waste cells.
                    break;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_input_returns_zero_codebook() {
        let out = train_1d_codebook_signed(&[], 16, 20);
        assert_eq!(out.len(), 16);
        assert!(out.iter().all(|&v| v == 0));
    }

    #[test]
    fn single_value_input_returns_that_value() {
        let out = train_1d_codebook_signed(&[1234i32; 100], 16, 20);
        assert_eq!(out.len(), 16);
        // With only one distinct value, all centroids end up there.
        assert!(out.iter().all(|&v| v == 1234));
    }

    #[test]
    fn bimodal_distribution_finds_both_modes() {
        let mut samples: Vec<i32> = Vec::new();
        for _ in 0..500 {
            samples.push(-10_000);
        }
        for _ in 0..500 {
            samples.push(10_000);
        }
        // Add a little spread so centroids can actually migrate.
        for i in 0..50 {
            samples.push(-10_000 + i);
            samples.push(10_000 - i);
        }
        let out = train_1d_codebook_signed(&samples, 2, 50);
        assert_eq!(out.len(), 2);
        // Output is sorted ascending.
        assert!(out[0] < 0, "expected negative centroid, got {}", out[0]);
        assert!(out[1] > 0, "expected positive centroid, got {}", out[1]);
        // Both centroids within 2 % of the modes.
        assert!((out[0] - (-10_000)).abs() < 200, "got {}", out[0]);
        assert!((out[1] - 10_000).abs() < 200, "got {}", out[1]);
    }

    #[test]
    fn small_distinct_count_pads_codebook() {
        let samples = vec![1i32, 2, 3];
        let out = train_1d_codebook_signed(&samples, 8, 20);
        assert_eq!(out.len(), 8);
        // First 3 entries are the distinct values (sorted).
        assert!(out.contains(&1));
        assert!(out.contains(&2));
        assert!(out.contains(&3));
    }

    #[test]
    fn nearest_centroid_picks_closest() {
        let c = [-1000i32, -10, 0, 100, 5000];
        assert_eq!(nearest_centroid_signed(&c, -1001), 0);
        assert_eq!(nearest_centroid_signed(&c, -6), 1);
        assert_eq!(nearest_centroid_signed(&c, 1), 2);
        assert_eq!(nearest_centroid_signed(&c, 49), 2); // 49 closer to 0 than 100
        assert_eq!(nearest_centroid_signed(&c, 60), 3); // 60 closer to 100 than 0
        assert_eq!(nearest_centroid_signed(&c, 9999), 4);
    }
}
