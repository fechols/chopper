# Digital music encoder

This standalone tool extends the DLT2 quantizer, Lloyd training, and band-limited
resampler from `secondreality-opt`, under the accompanying Unlicense. It needs
FFmpeg and FFprobe on PATH. Normal game builds use checked-in encoded assets.

## Profiles

| Invocation | Format | Source processing | Stored codes | DAC output |
|---|---|---|---|---|
| `--profile stereo20k3` | DLT5 | Preserve stereo, resample to 20 kHz | 3 bits/channel/frame | Rounded signed 8-bit stereo |
| `--profile mono16k3` | DLT5 | Floating-point `(L+R)/2`, resample to 16 kHz | 3 bits/frame | Rounded signed 8-bit mono |
| No profile option | Legacy DLT3 | Preserve stereo, resample to 24 kHz | 4 bits/channel/frame | Signed 16-bit stereo |

Hammond Rocker uses `stereo20k3`. Galactic Dash, Mission Failed, and all eighteen
stage recordings use `mono16k3`, with the player's 10 ms right-channel delay.
Galactic Dash has one canonical mono asset; the temporary duplicate is retired.

## Reproduction

Preserve the original M4As under `assets/music/source/`; convert them to signed
16-bit FLAC at their original sample rate and channel count. Do not downmix or
resample in FFmpeg: the encoder owns those operations.

```powershell
New-Item -ItemType Directory -Force -Path target/digital-music | Out-Null
ffmpeg -v error -y -i 'assets/music/source/Galactic Dash.m4a' -map 0:a:0 -vn -sample_fmt s16 -c:a flac -threads 1 target/digital-music/galactic-dash-source.flac
ffmpeg -v error -y -i 'assets/music/source/Hammond Rocker.m4a' -map 0:a:0 -vn -sample_fmt s16 -c:a flac -threads 1 target/digital-music/hammond-rocker-source.flac
cargo run --release --manifest-path tools/encode-digital-music/Cargo.toml --target-dir target/digital-music/tool-build -- --profile mono16k3 --codebooks 7341 target/digital-music/galactic-dash-source.flac target/digital-music/galactic-dash.dlt target/digital-music/galactic-dash-dlt5.wav
cargo run --release --manifest-path tools/encode-digital-music/Cargo.toml --target-dir target/digital-music/tool-build -- --profile stereo20k3 --codebooks 10036 target/digital-music/hammond-rocker-source.flac target/digital-music/hammond-rocker.dlt target/digital-music/hammond-rocker-dlt5.wav
```

Regenerating an asset also requires updating its provenance and regenerating the
soundtrack catalog. The positional legacy interface remains
`input.flac output.dlt preview.wav`; it reproduces the previous DLT3 bytes.
Profile encoding accepts `--codebooks COUNT` after the profile;
the default is 521. Counts must be 1 through 65,535 and are capped at the number
of packing groups in short inputs. The soundtrack's explicit catalog counts
allocate the expanded REU music region at approximately 20.92 ms per book.
Use the catalog regeneration tool for installed per-track settings; the commands
above demonstrate the unchanged baseline encoder and stage their outputs.

## Reconstruction and encoder search

The sequential 512-phase Kaiser-windowed sinc resampler retains floating-point
precision and filter overshoot until normalization. Mono downmix retains half
sample values. One constant track-wide gain preserves dynamics and, for stereo,
left/right balance.

DLT5 starts with eight-entry books trained from source deltas using twenty Lloyd
iterations. Up to three closed-loop retraining passes are retained only when
they improve squared error after final DAC rounding. A 32-trajectory beam search
then selects sample codes across each segment; it is retained only when it
improves on the greedy encoding. This search adds no target CPU work or bits.

### Optional offline quality experiments

All options precede the three positional paths. Existing invocations retain the
original baseline encoding, including its tie rules and encoded bytes.

| Option | Values and behavior |
|---|---|
| `--training` | `baseline` (default), `beam-refine`, `som`, or `combined` |
| `--beam-width` | `32` (default), `64`, or `128` predictor trajectories |
| `--reference-wav` / `--reference-metadata` | Paired paths to an authenticated, already-normalized reference and its baseline metadata |

`beam-refine` adds up to two closed-loop retraining passes using residuals from
the selected beam path. `som` proposes a second eight-entry book trained from
those residuals by a deterministic scalar self-organizing map. `combined` tries
the SOM candidate and then beam-residual retraining. Candidates replace the
current segment result only when their exact rounded-output squared error is
lower. Segment-end predictors still affect later segments, so acceptance of an
asset requires a separate complete-track comparison.

The SOM uses eight nodes initialized uniformly between residual extrema. Sorted
histogram data drive integer batch updates with triangular neighborhood radii
3, 2, and 1, four epochs each, followed by twenty seeded Lloyd iterations. It uses
no random seed or target neural-network code. Books are converted to the actual
signed-16 deltas before decoder evaluation. The trainer identifier is
`batch-triangular-v1`.

Frozen-reference mode validates the WAV rate, channels, frame count and PCM hash,
the requested book count, and the input FLAC hash against the reference metadata.
It encodes the exact reference samples without another downmix, resampling, or
gain adjustment. A peak failure exits with `FROZEN_REFERENCE_PEAK_LIMIT` before
writing any deliverable. This makes comparisons share one reference instead of
letting normalization changes affect the apparent SNR gain.

Metadata includes the actual training strategy and beam width, configured beam
retraining passes, SOM identifier, fixed-reference flag, and exact integer
`signal_sum_squares` / `error_sum_squares` for each channel and the full track.
Ordinary encoding also records full-precision linear gains; frozen encoding
preserves those when present and retains the original gain metadata otherwise.
The supplied reference PCM hash is authoritative: rounded historical dB values
are not used to reconstruct its samples.

Beam search deduplicates predictor states before selecting the best trajectories.
It preserves the original ordering by error, sample, parent and code; tests
compare it against the original full-sort algorithm at all supported widths.

Each channel keeps its own signed-16 saturating predictor across codebook changes.
Only publication is rounded:

```text
signed8 = clamp(floor(sample / 256 + 0.5), -128, 127)
```

WAV files expand these values by 256 into signed-16 containers, so they describe
the actual 8-bit DAC waveform. The predictor itself is never rounded to 8 bits.

The decoded sample peak and FFmpeg EBU R128 true-peak upper bounds must remain at
or below -1 dB. Metering includes a loop seam and the transition to silence. If
necessary, reduce gain by the excess plus 0.1 dB and retrain; fail after five
attempts without replacing deliverables. The true-peak bound includes 0.0005
linear amplitude for FFmpeg's rounded metadata.

## DLT5 bytes

The 32-byte little-endian header retains the existing field offsets: `DLT5` magic,
rate at 4, exact frame count at 8, packed byte count at 12, eight entries at 16,
book count at 18, three bits at 20, and one or two channels at 21. Bytes 22–31 are
zero. A frame means one mono sample or one simultaneous stereo pair.

Each segment stores eight little-endian signed-16 deltas per channel, left
before right for stereo: 16 bytes for mono or 32 bytes for stereo. This is the
same effective delta precision as DLT4, without its redundant sign byte.
Predictors do not reset at segment boundaries.

Segments distribute packing groups evenly. Let `A` be eight mono frames or four
stereo frames, `G = ceil(frames / A)`, `q = G / books`, and `r = G % books` using
integer division. The first `r` segments contain `(q + 1) * A` frames; the rest
contain `q * A`. Only the final segment is clipped to the exact frame count.
Metadata records `samples_per_segment = q * A` and `long_segment_count = r`.
This avoids an oversized final segment when the soundtrack uses thousands of
books. DLT5 changes both the entry format and partitioning; old DLT4 assets must
be regenerated and packaged with the matching player.

Codes form a continuous least-significant-bit-first stream. Stereo alternates
left and right codes. Eight codes occupy three bytes; the final unused bits are
zero and never decoded. The stream contains `ceil(frames * channels * 3 / 8)`
bytes. Both predictors and the stream CRC reset at the exact song loop.

The entire stream is followed by one cumulative CRC-8 byte per 1,024 packed bytes,
including a final short chunk. Polynomial is `0x07`, initial value zero, without
resetting between chunks.

## Evidence and listening

The JSON records extents, checksums, hashes, profile, normalization, decoded peaks,
and SNR. **SNR is relative to the normalized reference at the selected profile's
rate and channel count.** It does not measure the loss of bandwidth or stereo
information caused by profile selection; do not directly equate it with the
legacy profile's SNR.

For each DLT5 preview, a `.reference.wav` contains the normalized uncompressed
profile reference. Mono additionally writes `.delayed.wav`: left is the direct
waveform, right begins 160 samples later, with a silent prefix and retained final
tail. Compare these with the original stereo recording and legacy decode using
matched listening levels. Check attacks, cymbals, sustained tones, quiet parts,
loop seams, and mono fold-down. Numerical checks do not establish comparable
perceived quality.

```text
cargo test --manifest-path tools/encode-digital-music/Cargo.toml --target-dir target/digital-music/tool-build
```

Tests include independent bit-level decoding, balanced partitions and partial
tails, more than 1,024 books, book tables beyond 64 KiB, segment continuity,
signed extrema, all signed-16 rounding inputs, stereo predictor separation,
fractional downmix, beam-search correctness, unchanged legacy wire bytes,
normalization and filter response.
