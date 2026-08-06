# Draft Results

## 4.X Experimental Configuration

Phase F compared Edge and Ground processing over three independent NS-3 RNG runs per mode. The UAV target pose, 1 Hz relay rate, 60-second official measurement interval, 0.25 detector confidence threshold, and revised startup-stabilization procedure were held constant. Ground used JPEG quality 5. Statistics were calculated per run over metadata-defined official windows and then summarized across the three runs; individual frames were not treated as independent repetitions.

## 4.X Detection Accuracy under Compression

The D4 evaluation used 61 manually labelled frames per representation. Edge/raw achieved precision 0.878, recall 0.811, F1 0.843, and exact-count rate 0.557. Ground/JPEG q5 achieved precision 1.000, recall 0.172, F1 0.293, and exact-count rate 0.426. Phase F detection counts are not accuracy measurements and are therefore not used to infer precision or recall.

## 4.X Processed-Frame Reliability

Edge local pipeline completion was 100.00% ± 0.00% across the three runs. Ground complete compressed-frame reception and processing averaged 76.67% ± 7.27%. Ground completion by session was RNG 1: 81.67% (49/60), RNG 2: 80.00% (48/60), RNG 3: 68.33% (41/60). The resulting Edge advantage in mean processed-frame completion was 23.33 percentage points.

## 4.X End-to-End Pipeline Latency

The mean of the three run-level median latencies was 182.41 ± 19.22 ms for Edge and 507.86 ± 4.34 ms for Ground. Ground therefore exceeded Edge by 325.45 ms and was 2.78 times the Edge value. Mean p95 latency was 421.25 ± 170.22 ms for Edge and 562.97 ± 5.43 ms for Ground. Median and p95 values describe different parts of the latency distribution, and official-window tail observations were retained.

## 4.X Inference and Image-Processing Overhead

Mean YOLO inference time was similar between modes: 54.52 ± 3.12 ms for Edge and 51.57 ± 4.92 ms for Ground. Ground JPEG compression averaged 2.51 ± 0.05 ms and decoding averaged 2.14 ± 0.15 ms, giving a combined mean overhead of 4.64 ms.

## 4.X CPU and Memory Utilization

Mean monitored CPU utilization on the simulation host was 56.22% ± 2.56% for Edge and 42.44% ± 1.47% for Ground. Edge was higher by 13.78 percentage points. Mean RSS was 952.32 ± 5.11 MB for Edge and 963.34 ± 2.58 MB for Ground, a Ground-minus-Edge difference of 11.02 MB.

## 4.X Summary of Edge-versus-Ground Results

Under the tested configuration, Edge provided complete local frame processing and lower run-level median latency, while Ground used less host CPU but processed fewer complete frames. D4 further showed lower recall, F1, and exact-count rate for JPEG-quality-5 imagery than for raw imagery. These measurements describe separate accuracy and live-system evaluations and are not pooled into a single accuracy measure.
