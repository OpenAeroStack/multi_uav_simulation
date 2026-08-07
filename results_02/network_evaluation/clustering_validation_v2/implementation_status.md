# Clustering Implementation Status

The contemporaneous implementation is available in repository commit `09cff0a` but its source file is absent from the current checkout. Static classifications therefore cite that committed snapshot; stored bags provide runtime evidence.

| Feature | Function | Constants | Implementation | Trial observation | Final classification | Evidence |
|---|---|---|---|---|---|---|
| initial election | `election_callback` | period=2.0 s | implemented | yes | implemented_and_observed | lines 701,715-718 choose first eligible candidate |
| candidate scoring | `calculate_score` | weights=.40/.30/.20/.10; candidate SNR>=3 dB | implemented | yes | implemented_and_observed | lines 399-504 calculate score and eligibility |
| primary selection | `election_callback` | candidate sort by score | implemented | yes | implemented_and_observed | lines 689-702 select highest score |
| backup selection/reselection | `election_callback` | next eligible candidate | implemented | yes | implemented_and_observed | lines 770-795 select backup and label reselection |
| periodic election | `__init__/election_callback` | 2.0 s | implemented | yes | implemented_and_observed | lines 41,71-73,220-223 schedule callback |
| controlled primary switching | `election_callback` | margin=.12; wins=3; hold=10 s | implemented | no | implemented_but_not_triggered | lines 725-768 compare scores, enforce hold/wins, assign primary |
| switching margin | `election_callback` | 0.12 | implemented | no | implemented_but_not_triggered | lines 44,80-82,742-745 use margin in decision |
| consecutive-epoch requirement | `election_callback` | 3 wins | implemented | no | implemented_but_not_triggered | lines 45,83-85,747-758 count challenger wins |
| minimum holding period | `election_callback` | 10.0 s | implemented | no | implemented_but_not_triggered | lines 43,77-79,737-740 enforce hold |
| stale-measurement detection | `metrics_ready` | timeout=5.0 s | implemented | not identified | implemented_but_not_tested | lines 331-361 reject missing/stale global metrics |
| GCS-SNR failure threshold | `election_callback` | -2.0 dB | implemented | no | implemented_but_not_tested | lines 49,90-92,707-721 detect current primary link failure |
| immediate backup promotion | `election_callback` | n/a | not_found | no | not_implemented | failure path lines 719-723 assigns proposed_primary, not stored backup |
| new backup after primary change | `election_callback` | next eligible candidate | implemented | no | implemented_but_not_tested | lines 770-780 recompute backup after primary decision |
| assignment/role/score publication | `publish_state` | transient-local reliable state QoS | implemented | yes | implemented_and_observed | lines 589-644 publish assignment, scores, primary, backup |
| event publication | `publish_state` | only when changed | implemented | yes | implemented_and_observed | lines 646-668 publish reason and old/new IDs |

Controlled primary switching is fully present and reachable in the committed control flow but was not triggered. The failure-threshold path is only a partial match for the report's claimed emergency backup promotion: it selects the current best eligible candidate rather than explicitly promoting the stored backup, and global stale metrics stop election instead of triggering failover. No dedicated handover/failover test was found.
