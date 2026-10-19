# PSN-PATH

ns-3 simulation code for PSN-PATH. The simulation entry point is
[`scratch/network-load-balance.cc`](scratch/network-load-balance.cc).

## Prepare the input files

Create the following files:

| File | Contents |
| --- | --- |
| `config.txt` | Input/output paths, network settings, and recovery settings, based on the [configuration template](templates/config.txt.template). |
| `topology.txt` | Hosts, switches, links, link rates, propagation delays, and packet error probabilities. |
| `flow.txt` | Source and destination hosts, priority group, transfer size, and start time for each flow. |

From the repository root, create the run directory and copy the template:

```sh
cd ns3/ns3
mkdir -p mix/input mix/output/1
cp templates/config.txt.template mix/output/1/config.txt
```

Place the topology at `mix/input/topology.txt` and the traffic at
`mix/input/flow.txt`. Run the remaining commands from `ns3/ns3`.

### Topology format

Use whitespace-separated records:

```text
<NODE_COUNT> <SWITCH_COUNT> <LINK_COUNT>
<SWITCH_NODE_IDS>
<SOURCE_NODE_ID> <DESTINATION_NODE_ID> <RATE_WITH_UNIT> <DELAY_WITH_UNIT> <ERROR_PROBABILITY>
```

List exactly `SWITCH_COUNT` distinct switch IDs, followed by `LINK_COUNT` link
records. Node IDs are zero-based; nodes not listed as switches are hosts. Each
link record creates a bidirectional link. For host access links, write the host
ID first and the switch ID second. All hosts must be connected, with routes
between the traffic endpoints.

Use ns-3 rate strings with units, such as `Gb/s`. Each link delay must match the
source variable `one_hop_delay`, written with the `ns` suffix. A positive link
error probability overrides `ERROR_RATE_PER_LINK`; a nonpositive value uses that
global setting.

### Traffic format

```text
<FLOW_COUNT>
<SOURCE_HOST_ID> <DESTINATION_HOST_ID> <PRIORITY_GROUP> <FLOW_SIZE_BYTES> <START_TIME_SECONDS>
```

Add exactly `FLOW_COUNT` flow records, sorted by nondecreasing start time. Source
and destination IDs must identify hosts in the topology. Use a supported priority
group and a positive transfer size in bytes. Start times are absolute simulation
times in seconds. Set the monitoring and simulation time window to cover the
traffic.

The angle-bracketed fields above describe the file structure. Replace them with
values, repeat the records as required, and write the files without column names
or comments.

## Fill in the configuration

Replace all `<...>` placeholders in `mix/output/1/config.txt` with the settings
for the run. Optional settings that do not apply to the selected mode can be
omitted. Use supported configuration keys and whitespace-separated values, without
comments or whitespace in file paths.

Set the input paths:

```text
TOPOLOGY_FILE mix/input/topology.txt
FLOW_FILE mix/input/flow.txt
```

Set the output paths to the run directory:

```text
FLOW_INPUT_FILE mix/output/1/flow-record.txt
FCT_OUTPUT_FILE mix/output/1/fct.txt
PFC_OUTPUT_FILE mix/output/1/pfc.txt
CNP_OUTPUT_FILE mix/output/1/cnp.txt
QLEN_MON_FILE mix/output/1/qlen.txt
VOQ_MON_FILE mix/output/1/voq.txt
VOQ_MON_DETAIL_FILE mix/output/1/voq-detail.txt
UPLINK_MON_FILE mix/output/1/uplink.txt
CONN_MON_FILE mix/output/1/connection.txt
EST_ERROR_MON_FILE mix/output/1/estimation-error.txt
```

All relative paths are resolved from the working directory of the launch command.
Create the parent directories of all configured outputs before running.
`FLOW_INPUT_FILE` is a flow-record output; traffic is read from `FLOW_FILE`.
Use a separate output directory for each run to preserve existing results.

Fill in network and algorithm settings consistently with the input files:

- Cover every switch-port link rate in `KMIN_MAP`, `KMAX_MAP`, and `PMAX_MAP`.
  Each map starts with an entry count, followed by that many rate/value pairs.
  Map rates are integer bit/s; queue thresholds are decimal KB.
- Set the recovery switches for the selected model. See
  [recovery settings](#recovery-models) for their combinations and scope.
- For PSN-PATH, supply `PSN_PATH_K`. Supply the loss penalty, thresholds, and
  evaluation interval required by the selected scoring and switching mode. See
  [path scoring](#psn-path-mapping-and-scoring).

Parameter units are listed in the [configuration reference](#configuration-reference).

## Configuration reference

Configuration keys override the corresponding source initializers. Parameters
without a parser key remain source settings. In particular, the driver sets the
IRN RTOs and the topology delay constraint in source.

### Network settings

| Key | Meaning and units |
| --- | --- |
| `FLOWGEN_START_TIME`, `FLOWGEN_STOP_TIME` | Monitoring/completion-check time settings, in seconds; traffic start times come from the flow file. |
| `SW_MONITORING_INTERVAL` | Positive monitoring interval, in nanoseconds. |
| `BUFFER_SIZE` | Switch buffer capacity, in MiB. |
| `PACKET_PAYLOAD_SIZE` | Positive packet payload length, in bytes. |
| `RANDOM_SEED` | ns-3 random seed. |
| `ERROR_RATE_PER_LINK` | Global packet error probability; a positive per-link probability overrides it. |
| `ENABLE_PFC`, `ENABLE_QCN`, `USE_DYNAMIC_PFC_THRESHOLD` | PFC, congestion feedback, and dynamic PFC threshold switches. |
| `KMIN_MAP`, `KMAX_MAP` | Entry count followed by rate/threshold pairs; rate in integer bit/s, threshold in decimal KB. |
| `PMAX_MAP` | Entry count followed by rate/marking-probability pairs; rate in integer bit/s. |

The three ECN maps must cover all switch-port link rates. For each rate, the
minimum threshold must not exceed the maximum threshold.

### Congestion control and load balancing

| Key | Meaning and units |
| --- | --- |
| `CC_MODE` | Congestion-control mode: DCQCN = `1`, HPCC = `3`, TIMELY = `7`. |
| `LB_MODE` | Load balancing: flow ECMP = `0`, DRILL = `2`, Conga = `3`, Letflow = `6`, ConWeave = `9`. |
| `RATE_AI`, `RATE_HAI`, `MIN_RATE`, `DCTCP_RATE_AI` | Rates with ns-3 units. |
| `ALPHA_RESUME_INTERVAL`, `RP_TIMER`, `RATE_DECREASE_INTERVAL` | Time intervals, in microseconds. |
| `EWMA_GAIN`, `FAST_RECOVERY_TIMES`, `MI_THRESH`, `U_TARGET` | Gain, recovery count, increase threshold, and target utilization for the selected congestion controller. |
| `CLAMP_TARGET_RATE`, `HAS_WIN`, `VAR_WIN`, `FAST_REACT`, `MULTI_RATE`, `SAMPLE_FEEDBACK`, `RATE_BOUND` | Rate, window, and feedback switches. |
| `GLOBAL_T` | Use the global maximum RTT/BDP when enabled; otherwise use the endpoint-pair RTT/BDP. |
| `INT_MULTI` | INT fixed-point encoding multiplier. |
| `L2_CHUNK_SIZE`, `L2_ACK_INTERVAL` | Chunk size and cumulative ACK interval, in bytes. |
| `L2_BACK_TO_ZERO` | Chunk-based rollback switch; requires a positive chunk size when enabled. |

Boolean switches use `0` and `1`. Mode IDs are program identifiers. ConWeave has
additional settings in the driver and is supported there with DCQCN or HPCC.


## Recovery models

The simulator models transport recovery for the following methods. These are
packet-level recovery models, rather than complete NIC hardware implementations.

### Select a recovery model

Use the recovery switches consistently with the selected model. For a standalone
baseline, disable PSN-PATH, path selection, path-aware recovery, and the other
baseline recovery switches.

| Model | Recovery switches |
| --- | --- |
| GBN | Disable `ENABLE_IRN`, `ENABLE_BITMAP_RETRANS`, `ENABLE_FALCON`, and `ENABLE_ORNIC`. |
| IRN selective recovery | Enable `ENABLE_IRN`; disable `ENABLE_RX_OOO_NACK` and `ENABLE_TX_NACK_GOBACK`. |
| Traditional Bitmap | Enable `ENABLE_BITMAP_RETRANS`. |
| Falcon OOO-D | Enable `ENABLE_FALCON`. |
| ORNIC OTD-based model | Enable `ENABLE_ORNIC`. |

The direct out-of-order NACK and NACK-triggered GoBack switches select additional
behaviors. Keep them disabled for standalone IRN selective recovery. PSN-PATH
configuration is described under [path mapping and scoring](#psn-path-mapping-and-scoring).

### GBN and IRN

GBN rolls back to the cumulative ACK on NACK or timeout and retransmits the suffix.
IRN retains selective acknowledgment state and skips received packets during
recovery. The first recovery retransmission starts at the cumulative ACK;
subsequent inferred holes require a higher selectively acknowledged packet.
Unproven tail losses remain subject to timeout recovery. Recovery ends when the
cumulative ACK covers the original recovery boundary.

IRN's BDP-FC bounds outstanding data. `IRN_BDP` is a byte limit. If omitted or
zero, the driver uses the maximum BDP computed from the loaded topology and
routes. A positive configured value overrides that computation. The effective
limit must be positive and fit the simulator's field. Retransmissions do not
consume new sequence space.

BDP-FC and selective recovery are described in
[IRN, Sections 3.1–3.2](https://arxiv.org/html/1806.08159).

### Falcon OOO-D recovery

The modeled early-retransmission rule follows the out-of-order distance mechanism
in the [OCP Falcon Transport Protocol specification, Sections 9.1.1 and 9.1.4](https://www.opencompute.org/documents/falcon-spec-v1-1-pdf-1):

```text
highest selectively acknowledged PSN - missing PSN > distance threshold
age since the packet's latest transmission > recency interval
```

`FALCON_OOO_DISTANCE_THRESHOLD` sets the PSN-distance threshold within the modeled
ACK bitmap. `FALCON_RETRANS_RTT_K` scales the RTT-based recency interval and must
be finite and at least one. Without an RTT estimate, the model uses ordinary
timeout recovery. Deferred retransmissions recheck acknowledgment state and the
latest send time before entering the transmit queue.

This is the OOO-D recovery model. The
[Falcon SIGCOMM 2025 paper](https://praveenk.io/papers/falcon.pdf) also describes
RACK-TLP recovery; selecting this model does not enable RACK-TLP.

### ORNIC OTD-based recovery

The [ORNIC paper, Section 3.3](https://www.mdpi.com/2079-9292/14/1/88)
defines OTD from the maximum multipath delay difference, send bandwidth, and
packet length, and supports per-QP OTD configuration.

```text
MDD = (RTT_max - RTT_min) / 2
OTD = MDD * B / packet_length
```

Use the slowest and fastest paths of the selected multipath strategy for
`RTT_max` and `RTT_min`. With RTTs in seconds and sending bandwidth `B` in bit/s,
`packet_length` is in bits and OTD is a distance in packet sequence numbers.

Set `ORNIC_OTD` explicitly when `ENABLE_ORNIC` is enabled. It must be finite and
nonnegative; a missing value is an error. The driver uses this value to initialize
each receive QP's OTD. The receiver triggers loss recovery when the PSN distance
exceeds that QP's configured OTD. Sender timeout recovery covers losses without
subsequent packets, as described in Section 3.3.

### Traditional Bitmap

`BITMAP_RETRANS_SIZE` controls the receive bitmap. The receiver records
out-of-order packets and returns cumulative ACKs. Overflow or the optional gap
feedback timer requests recovery of the earliest hole.
`BITMAP_RETRANS_TIMEOUT_NS` configures that receiver timer; zero disables it.
The feedback does not carry a full selective-ACK bitmap to the sender.

This model follows the traditional Bitmap method in the
[RDMA reference repository](https://github.com/tomyu872-jpg/RDMA/tree/7df1d53e7818b0909a0948e44f5aa4f78951b141).

### Timing in the simulation

Sender timeout recovery is available for losses without later feedback, including
tail losses. Control packets enter the NIC queue and experience modeled queueing,
serialization, and propagation delays.

Falcon uses packet timestamp metadata for its RTT-based recency check. The metadata
is an ns-3 packet tag rather than an additional serialized header. HPCC INT fields
are interpreted according to their header mode.

## PSN-PATH mapping and scoring

### Path mapping and recovery

Original packets use logical lanes selected by `PSN % K`. Each lane maintains its
own source-port offset and mapping boundary. Replacing a degraded path changes
that lane's source port while retaining the lane count and the other mappings.
A distinct source port is a routing input; the topology and ECMP hash determine
its physical route.

The receiver tracks the lane, mapping generation, and source port. Observing a
new mapping does not establish loss on its predecessor. Retransmissions retain
the original mapping boundary and may use the lane's current port. Such rerouted
retransmissions are not ordering evidence for the new path. Same-path evidence
supports loss detection; sender timeout recovery handles gaps without later
same-path evidence.

The simulator carries control metadata in packet tags and accounts for the
PSN-PATH control-word length in packet serialization. The receiver separates that
length from the data payload.

### Scoring

```text
raw_score = ACK - beta * NACK
normalized_score = (ACK - beta * NACK) / (ACK + NACK)
```

The sender attributes observations to original transmissions and their original
paths. Cumulative ACKs account for each covered packet. Repeated feedback is
deduplicated, and retransmitted packets are excluded from successful-original
counts. Losses are counted once. An evaluation window with no observations keeps
the previous score and does not independently trigger replacement.

Set `PSN_PATH_NORMALIZE_SCORE` to select normalization. For finite nonnegative
`beta`, normalized scores lie in `[-beta, 1]`. Thresholds must satisfy
`-beta <= remove <= explore <= 1`. Threshold comparisons are strict.
Normalization is an optional scoring extension; thresholds are supplied directly
in the chosen score scale.

### Configuration

| Key | Meaning |
| --- | --- |
| `ENABLE_PSN_PATH` | Enable path selection and path-aware recovery. |
| `ENABLE_PATH_SELECT`, `ENABLE_PATH_AWARE_RETRANS` | Control those capabilities separately. |
| `ENABLE_PATH_SWITCH` | Enable periodic path evaluation and replacement; requires path selection. |
| `PSN_PATH_K` | Positive logical lane count, required when PSN-PATH, path selection, or path-aware recovery is enabled. |
| `PSN_PATH_BASE_PORT` | Optional base UDP source port; otherwise the QP's original source port is used. |
| `PSN_PATH_O` | Optional initial offset. |
| `PSN_PATH_NORMALIZE_SCORE` | Select normalized or raw scores. |
| `PSN_PATH_BETA` | Explicit loss penalty, required for normalization or switching. |
| `PSN_PATH_NORMALIZED_T_EXPLORE`, `PSN_PATH_NORMALIZED_T_REMOVE` | Explicit thresholds when normalization is enabled. |
| `PSN_PATH_T_EXPLORE`, `PSN_PATH_T_REMOVE` | Explicit raw-score thresholds when switching is enabled without normalization. |
| `PSN_PATH_EVAL_INTERVAL_US` | Positive evaluation interval, in microseconds, required for switching. |

The implemented switching mode replaces degraded paths per lane. Configure equal
explore and remove thresholds for this mode. Fixed mappings are selected by
leaving switching disabled. Candidate exploration and gradual migration are not
implemented by this mode.

Packet sequence numbers use the configured payload length.

## Build and run

Use a compatible ns-3/waf environment and a Python interpreter compatible with the
bundled waf. Configure and build from `ns3/ns3`:

```sh
./waf configure --build-profile=optimized --enable-modules=applications --disable-python
./waf
```

Run the simulation:

```sh
./waf --run 'scratch/network-load-balance mix/output/1/config.txt'
```

To save console output, use:

```sh
./waf --run 'scratch/network-load-balance mix/output/1/config.txt' > mix/output/1/config.log 2>&1
```
## Reference
Please cite PSN-PATH in your publication if it helps your research:

```bibtex
@inproceedings{li2026PSN,
  title={{PSN-PATH: When Multipath RDMA Meets Lossy Networks}},
  author={Li, Zhexiong and Wei, Shugui and Zhao, Puyu and Wang, Feifan and Li, Yuepeng and Gu, Lin and Zeng, Deze and Wang, Xiaoliang and Zhao, Laiping},
  booktitle={Proceedings of the ACM SIGCOMM Conference (SIGCOMM)},
  pages={1215-1227},
  year={2026}
}
```
