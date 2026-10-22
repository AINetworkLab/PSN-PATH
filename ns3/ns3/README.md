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
- For PSN-PATH, supply `PSN_PATH_K`. Supply the loss penalty and thresholds required by the selected
  scoring and switching mode. See
  [PSN-PATH configuration](#psn-path-configuration).

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
configuration is described under [PSN-PATH configuration](#psn-path-configuration).

## PSN-PATH configuration

| Key | Meaning |
| --- | --- |
| `ENABLE_PSN_PATH` | Enable path selection and path-aware recovery. |
| `ENABLE_PATH_SELECT`, `ENABLE_PATH_AWARE_RETRANS` | Control those capabilities separately. |
| `ENABLE_PATH_SWITCH` | Enable DT-Switch path evaluation, candidate exploration, and migration; requires path selection. |
| `PSN_PATH_K` | Positive logical lane count, required when PSN-PATH, path selection, or path-aware recovery is enabled. |
| `PSN_PATH_BASE_PORT` | Optional base UDP source port; otherwise the QP's original source port is used. |
| `PSN_PATH_O` | Optional initial offset. |
| `PSN_PATH_NORMALIZE_SCORE` | Select normalized or raw scores. |
| `PSN_PATH_BETA` | Explicit loss penalty, required for normalization or switching. |
| `PSN_PATH_NORMALIZED_T_EXPLORE`, `PSN_PATH_NORMALIZED_T_REMOVE` | Explicit thresholds when normalization is enabled. |
| `PSN_PATH_T_EXPLORE`, `PSN_PATH_T_REMOVE` | Explicit raw-score thresholds when switching is enabled without normalization. |
| `PSN_PATH_PROBE_FREQUENCY` | Explicit initial step `f`: send one candidate packet for every `f` original-path packets in the same lane. Required when the explore threshold exceeds the remove threshold; must fit Explore's positive seven-bit step field. |

DT-Switch evaluates each logical lane independently. A score below the removal
threshold triggers a source-port search starting at the lane's current offset
plus one, skipping mappings still in use. A score between the removal and
exploration thresholds starts candidate exploration,
including when all original lanes are active. Candidate and original paths use
the same selected score formula; a candidate with a score no lower than the
original path enters gradual migration. A lower-scoring candidate is discarded;
later evaluation windows may start another probe. Each `f` successful candidate
ACKs reduces `f` by one until the lane moves fully to the candidate. A candidate
NACK cancels exploration or migration and retains the original mapping. The sender
RTO also recovers packets whose loss has no later same-path witness.

During exploration and gradual migration, packets carry the switching status,
step, candidate offset, and phase-start PSN. FPSN identifies the first original
packet of a physical mapping and remains unchanged as the migration step changes.
The phase-start PSN is a simulation metadata extension for deriving the alternating
schedule. After completion or cancellation, this extension is retained until a
cumulative ACK covers the first original packet under the final schedule; steady
packets then omit it. Retransmissions retain the optional fields of their original
transmission, even if the lane has since changed state.

Packet lengths include the seven-byte path control word and, only when present,
the two-byte Explore field and four-byte phase-start extension. The receiver uses
the same per-packet layout to recover the data payload length. Configure the step
and score parameters directly in `config.txt`; the template contains placeholders.
Leave switching disabled for fixed mappings.

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
