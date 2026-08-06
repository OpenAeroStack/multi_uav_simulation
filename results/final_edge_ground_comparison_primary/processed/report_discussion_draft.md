# Draft Discussion

## Reliability and latency

Edge processing completed every locally published official frame under the tested configuration. This is consistent with the image stream remaining within the local Edge pipeline and avoiding complete-image delivery across the emulated wireless path. Ground processing achieved a mean completion ratio of 76.7%, indicating that some compressed frames were not received and processed completely. The longer Ground inter-arrival gaps are therefore treated as delivery-performance observations rather than provenance failures.

Ground's mean of run median latencies was 2.78 times the Edge value. JPEG compression and decoding together averaged only 4.64 ms, which suggests that these operations alone do not explain the 325.45 ms median-latency difference. The result is consistent with additional transport, scheduling, middleware, and pipeline waiting contributions. The available `wireless_transit_ms` field is not guaranteed to isolate pure radio propagation delay, so the latency gap should not be assigned to a single component.

## Detection accuracy and computation

YOLO inference time remained similar between modes, indicating that the inference workload itself was broadly comparable after an image reached the detector. However, D4 showed that JPEG quality 5 reduced recall from 0.811 to 0.172 and F1 from 0.843 to 0.293. The Ground precision of 1.000 occurred alongside low recall and does not indicate superior overall detection performance; it reflects few false positives among a much smaller set of detected positives.

## Reliability-resource trade-off

Edge required 13.78 percentage points more mean CPU on the simulation host. This indicates a compute-cost trade-off for its higher local completion and lower latency, but it is not a measurement of physical UAV energy use. Ground mean RSS exceeded Edge by only 11.02 MB, suggesting relatively similar memory demand compared with the larger reliability and latency differences.

Overall, the results support Edge processing for time-sensitive perception under the tested configuration. Ground processing may remain suitable where aerial compute is constrained, communication conditions reliably support complete-frame delivery, sufficient image quality can be maintained, or centralized processing offers operational benefits. These findings do not establish universal superiority for either architecture.
