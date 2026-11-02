//! Deterministic offline scalar SOM, used only to propose alternative books.
use std::collections::BTreeMap;

pub const TRAINER: &str = "batch-triangular-v1";

pub fn train(samples: &[i32]) -> Vec<i32> {
    let mut histogram = BTreeMap::<i32, u64>::new();
    for &sample in samples {
        *histogram.entry(sample).or_default() += 1;
    }
    let points: Vec<_> = histogram.into_iter().collect();
    if points.is_empty() {
        return vec![0; 8];
    }
    if points.len() <= 8 {
        let mut nodes: Vec<_> = points.iter().map(|&(value, _)| value).collect();
        nodes.resize(8, *nodes.last().unwrap());
        return nodes;
    }
    let low = i64::from(points[0].0);
    let span = i64::from(points.last().unwrap().0) - low;
    let mut nodes: Vec<_> = (0..8).map(|i| (low + span * i / 7) as i32).collect();
    for radius in [3usize, 2, 1] {
        for _ in 0..4 {
            let mut sums = [0i128; 8];
            let mut weights = [0u64; 8];
            for &(value, count) in &points {
                let winner = nearest(&nodes, value);
                for node in 0usize..8 {
                    let weight = (radius + 1).saturating_sub(node.abs_diff(winner)) as u64 * count;
                    sums[node] += i128::from(value) * i128::from(weight);
                    weights[node] += weight;
                }
            }
            for node in 0..8 {
                if weights[node] != 0 {
                    nodes[node] = (sums[node] / i128::from(weights[node])) as i32;
                }
            }
        }
    }
    // Seeded Lloyd refinement drops the topology constraint before evaluation.
    for _ in 0..20 {
        let mut sums = [0i128; 8];
        let mut counts = [0u64; 8];
        for &(value, count) in &points {
            let winner = nearest(&nodes, value);
            sums[winner] += i128::from(value) * i128::from(count);
            counts[winner] += count;
        }
        let mut next = nodes.clone();
        for node in 0..8 {
            if counts[node] != 0 {
                next[node] = (sums[node] / i128::from(counts[node])) as i32;
            } else {
                // First highest-count cell wins a tie; this is deterministic.
                let mut heavy = 0;
                for other in 1..8 {
                    if counts[other] > counts[heavy] {
                        heavy = other;
                    }
                }
                next[node] = nodes[heavy].saturating_add(1);
            }
        }
        if next == nodes {
            break;
        }
        nodes = next;
    }
    nodes.sort_unstable();
    nodes
}

fn nearest(nodes: &[i32], value: i32) -> usize {
    (0..nodes.len())
        .min_by_key(|&i| ((i64::from(nodes[i]) - i64::from(value)).abs(), i))
        .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_histogram_training_handles_order_duplicates_and_extrema() {
        let samples: Vec<_> = (0..113).map(|i| (i * 1199 % 131071) - 65535).collect();
        let expected = train(&samples);
        let reversed: Vec<_> = samples.iter().rev().copied().collect();
        assert_eq!(train(&reversed), expected);
        assert_eq!(train(&[samples.clone(), samples].concat()), expected);
        assert_eq!(expected.len(), 8);
        assert!(expected.windows(2).all(|pair| pair[0] <= pair[1]));
        assert!(expected.iter().all(|&v| (-65535..=65535).contains(&v)));
        assert_eq!(train(&[i32::MIN, i32::MAX])[..2], [i32::MIN, i32::MAX]);
    }

    #[test]
    fn silence_and_short_inputs_have_complete_books() {
        assert_eq!(train(&[]), vec![0; 8]);
        assert_eq!(train(&[0; 100]), vec![0; 8]);
        assert_eq!(train(&[-7, 4, -7]), vec![-7, 4, 4, 4, 4, 4, 4, 4]);
    }
}
