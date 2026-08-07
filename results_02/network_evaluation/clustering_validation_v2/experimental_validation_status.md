# Experimental Validation Status

| Feature | Implemented in code | Triggered in corrected trials | Dedicated test found | Experimentally validated | Report wording |
|---|---|---|---|---|---|
| Initial election | Yes | Yes (stored early events/states) | No separate test | Yes, within dynamic runs | A valid initial primary and backup were established. |
| Primary-head retention | Yes | Yes | Dynamic trials | Yes | The primary remained unchanged in each corrected trial. |
| Backup-head selection | Yes | Yes | Dynamic trials | Yes | A valid backup was selected in each trial. |
| Backup-head reselection | Yes | Yes | Dynamic trials | Yes | Canonical backup transitions were 2, 4 and 3. |
| Controlled primary handover | Yes | No | No | No | Implemented in the contemporaneous clustering node, but not triggered; trials validate stability, not handover. |
| Emergency primary failover | Partially: threshold-driven best-candidate replacement, not guaranteed backup promotion | No | No | No | A partial primary-link-failure path existed, but emergency backup promotion/failover was not experimentally validated. |
| Cluster assignment publication | Yes | Yes | Dynamic trials | Yes | Complete assignment states were published and recorded. |
| Cluster event publication | Yes | Yes | Dynamic trials | Yes for observed election/reselection events | Change events were published; event count is not the canonical transition count. |
