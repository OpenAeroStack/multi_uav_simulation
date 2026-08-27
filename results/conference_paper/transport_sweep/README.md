# Ground JPEG Transport Feasibility Sweep

This experiment measures complete-frame transport reliability for Ground processing at JPEG qualities 5, 10, 20, 30, 40, and 50. It retains the existing platform configuration: 1 Hz relay rate, BEST_EFFORT/KEEP_LAST/depth-1 compressed-image QoS, current DDS defaults, current 1500-byte network MTUs, current socket-buffer defaults, and the existing NS-3 channel and PHY configuration.

The existing Gazebo, namespace, and NS-3 infrastructure must be running with the requested RNG run and the UAV at the fixed Phase F test pose. Run one quality with:

```bash
bash results/conference_paper/transport_sweep/run_transport_once.sh RUN_ID RNG_RUN JPEG_QUALITY
```

The runner waits for the compressed-image DDS endpoints, uses a fixed settling period, then measures exactly 60 new relay publications. It does not require any successfully received or processed frame, so a zero-delivery run remains valid.

Optional instrumentation is enabled only for these runs. The relay places a monotonically increasing sequence ID after the preserved send timestamp in `CompressedImage.header.frame_id` as `<send_timestamp>|<sequence_id>`. The Ground detector separates the two values, continues using the timestamp for latency, and writes callback, decode, inference, and result-publication events keyed by sequence ID.

The result builder starts with all 60 official relay-publication events and left-joins detector events by sequence ID. A frame that never reaches the GCS therefore remains in the final trace with `gcs_callback_received=false` and blank/false downstream fields rather than disappearing.

Raw run artifacts are stored under `raw/<run_id>/`. Processed outputs are `transport_trace_<run_id>.csv`, `transport_summary_<run_id>.csv`, and `tap_deltas_<run_id>.csv`. Existing paths are never overwritten.

No DDS fragmentation, reliability, QoS depth, socket buffer, MTU, or NS-3 network parameter is changed by this experiment.
