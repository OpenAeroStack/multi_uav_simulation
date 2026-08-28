# Limitations Notes

- The received-power raw CSV retains only three relevant packets from one configured distance window. Multi-distance R², trend, and historical fit metrics are not independently verifiable.
- The available network traffic logs are five fixed-2-Mbit/s runs, not an offered-load sweep.
- Historical distance labels in raw filenames and the processed B4 summary are inconsistent; labels are preserved rather than silently corrected.
- Ground transports JPEG ROS 2 `CompressedImage` messages over DDS, not a production video codec or RTP stream.
- Perception evaluation is count-based and cannot support bounding-box IoU, AP, or mAP claims.
- Rate-sweep latency includes only results received at the GCS and therefore has survivorship bias, especially for Ground 2 Hz.
- Ground 2 Hz has 5–7 successful results per run; inference, latency, and CPU summaries are unstable and conditional on delivery.
- CPU measurements describe host processes in simulation and are neither onboard embedded CPU nor electrical power.
- TAP byte counts include all interface traffic.
- The rate case study used a single controlled pose and one UAV vision stream.
- No completed conference multi-UAV scaling result set was available when this package was frozen.
- No sufficient raw Raspberry Pi/HITL dataset was found.
- Edge P95 latency has large run-to-run variability and is not used as the headline latency result.
