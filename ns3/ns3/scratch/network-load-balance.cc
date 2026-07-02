/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2023 NUS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Chahwan Song <skychahwan@gmail.com>
 */

#include <ns3/assert.h>
#include <ns3/rdma-client-helper.h>
#include <ns3/rdma-client.h>
#include <ns3/rdma-driver.h>
#include <ns3/rdma.h>
#include <ns3/sim-setting.h>
#include <ns3/switch-node.h>
#include <time.h>

#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unistd.h>

#include "ns3/applications-module.h"
#include "ns3/broadcom-node.h"
#include "ns3/conga-routing.h"
#include "ns3/conweave-voq.h"
#include "ns3/core-module.h"
#include "ns3/error-model.h"
#include "ns3/global-route-manager.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/letflow-routing.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/qbb-helper.h"
#include "ns3/qbb-net-device.h"
#include "ns3/rdma-hw.h"
#include "ns3/settings.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("GENERIC_SIMULATION");

/*------Load balancing parameters-----*/
// mode for load balancer, 0: flow ECMP, 2: DRILL, 3: Conga, 6: Letflow, 9: ConWeave
uint32_t lb_mode = 0;

// Conga params (based on paper recommendation)
Time conga_flowletTimeout = MicroSeconds(100);  // 100us
Time conga_dreTime = MicroSeconds(50);
Time conga_agingTime = MicroSeconds(500);
uint32_t conga_quantizeBit = 3;
double conga_alpha = 0.2;

// Letflow params
Time letflow_flowletTimeout = MicroSeconds(100);  // 100us
Time letflow_agingTime = MilliSeconds(2);  // just to clear the unused map entries for simul speed

// Conweave params
Time conweave_extraReplyDeadline = MicroSeconds(4);       // additional term to reply deadline
Time conweave_pathPauseTime = MicroSeconds(8);            // time to send packets to congested path
Time conweave_txExpiryTime = MicroSeconds(1000);          // waiting time for CLEAR
Time conweave_extraVOQFlushTime = MicroSeconds(32);       // extra for uncertainty
Time conweave_defaultVOQWaitingTime = MicroSeconds(500);  // default flush timer if no history
bool conweave_pathAwareRerouting = true;

/*------------------------ simulation variables -----------------------------*/
uint64_t one_hop_delay = 1000;  // nanoseconds
uint32_t cc_mode = 1;           // mode for congestion control, 1: DCQCN
bool enable_qcn = true, enable_pfc = true, use_dynamic_pfc_threshold = true;
uint32_t packet_payload_size = 1000, l2_chunk_size = 0, l2_ack_interval = 0;
double pause_time = 5;  // PFC pause, microseconds
double flowgen_start_time = 2.0, flowgen_stop_time = 2.5, simulator_extra_time = 0.1;
// queue length monitoring time is not used in this simulator
// uint32_t qlen_dump_interval = 100000000, qlen_mon_interval = 1000;  // ns
uint64_t qlen_mon_start;               // ns
uint64_t qlen_mon_end;                 // ns
uint32_t switch_mon_interval = 10000;  // ns
uint64_t cnp_mon_start;                // ns
uint64_t cnp_monitor_bucket = 100000;  // ns
uint64_t irn_mon_start;                // ns
uint64_t irn_monitor_bucket = 100000;  // ns

FILE *pfc_file = NULL;
FILE *fct_output = NULL;
FILE *flow_input_stream = NULL;
FILE *cnp_output = NULL;
FILE *est_error_output = NULL;
FILE *voq_output = NULL;
FILE *voq_detail_output = NULL;
FILE *uplink_output = NULL;
FILE *conn_output = NULL;

std::string data_rate, link_delay, topology_file, flow_file;
std::string flow_input_file = "flow.txt";
std::string fct_output_file = "fct.txt";
std::string pfc_output_file = "pfc.txt";
std::string cnp_output_file = "cnp.txt";
std::string qlen_mon_file = "qlen.txt";
std::string voq_mon_file = "voq.txt";
std::string voq_mon_detail_file = "voq_detail.txt";
std::string uplink_mon_file = "uplink.txt";
std::string conn_mon_file = "conn.txt";
std::string est_error_output_file = "est_error.txt";
std::string burst_loss_schedule = "";
std::string burst_loss_log_file = "burst_loss.txt";
uint64_t burst_stat_start_ns = 0;
uint64_t burst_stat_end_ns = 0;
uint64_t burst_stat_interval_ns = 0;
std::string burst_goodput_output_file = "";
std::string burst_redundancy_output_file = "";

// CC params
double alpha_resume_interval = 55, rp_timer = 300, ewma_gain = 1 / 16;
double rate_decrease_interval = 4;
uint32_t fast_recovery_times = 1;
std::string rate_ai, rate_hai, min_rate = "100Mb/s";
std::string dctcp_rate_ai = "1000Mb/s";

bool clamp_target_rate = false, l2_back_to_zero = false;
double error_rate_per_link = 0.0;
uint32_t has_win = 1;
uint32_t global_t = 1;
uint32_t mi_thresh = 5;
bool var_win = false, fast_react = true;
bool multi_rate = true;
bool sample_feedback = false;
double u_target = 0.95;
uint32_t int_multi = 1;
bool rate_bound = true;
bool hpcc_trace = false;
bool int_trace = false;
uint32_t hpcc_trace_interval = 128;
uint32_t enable_console_log = 1;
std::string console_log_psns = "";
unordered_map<uint64_t, uint32_t> rate2kmax, rate2kmin;
unordered_map<uint64_t, double> rate2pmax;
unordered_map<uint32_t, Ptr<SwitchNode>> idxNodeToR;  // Id -> Ptr

// config of link-down scenario, ACK priority, and buffer
uint64_t link_down_time = 0;
uint32_t link_down_A = 0, link_down_B = 0;
uint32_t buffer_size = 0;  // 0 to set buffer size automatically

// Added from Here
double load = 10.0;
int enable_irn = 0;
int enable_psn_path = 0;
int enable_path_select = 0;
int enable_path_switch = 0;
int enable_path_aware_retrans = 0;
int enable_rx_ooo_nack = 0;
int enable_tx_nack_goback = 0;
uint64_t tx_nack_retrans_interval_ns = 0;
int enable_bitmap_retrans = 0;
int enable_falcon = 0;
int enable_ornic = 0;
double falcon_retrans_rtt_k = 0.0;
uint32_t bitmap_retrans_size = BITMAP_SIZE;
uint64_t bitmap_retrans_timeout_ns = 0;
uint64_t falcon_rx_send_delay_ns = 0;
uint64_t ornic_rx_send_delay_ns = 0;
double ornic_bw_gbps = 0.0;
uint32_t psn_path_k = 4;
double psn_path_beta = 1.0;
uint64_t psn_path_gap_timeout_ns = 0;
int random_seed = 1;  // change this randomly if you want random expt

uint64_t maxRtt, maxBdp;

struct BurstLossPeriod {
    uint64_t start_ns;
    uint64_t end_ns;
    double loss_rate;
};

struct BurstStatBucket {
    uint64_t rx_data_bytes{0};
    uint64_t tx_data_bytes{0};
    uint64_t tx_data_pkts{0};
    uint64_t retrans_data_pkts{0};
};

std::vector<BurstLossPeriod> burst_loss_periods;
std::vector<Ptr<RateErrorModel>> burst_loss_error_models;
std::set<RateErrorModel *> burst_loss_error_model_seen;
FILE *burst_loss_log_output = NULL;
std::map<std::pair<uint32_t, uint32_t>, std::vector<BurstStatBucket>> burst_flow_stats;

// app parameters
struct Interface {
    uint32_t idx;
    bool up;
    uint64_t delay;
    uint64_t bw;

    Interface() : idx(0), up(false) {}
};
map<Ptr<Node>, map<Ptr<Node>, Interface>> nbr2if;
// Mapping destination to next hop for each node: <node, <dest, <nexthop0, ...> > >
map<Ptr<Node>, map<Ptr<Node>, vector<Ptr<Node>>>> nextHop;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairDelay;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairTxDelay;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairBw;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairBdp;
map<Ptr<Node>, map<Ptr<Node>, uint64_t>> pairRtt;

// for uplink/Downlink monitoring at TOR switches (load balance performance)
std::map<uint32_t, std::vector<uint32_t>> torId2UplinkIf;
std::map<uint32_t, std::vector<uint32_t>> torId2DownlinkIf;

// input files
std::ifstream topof, flowf;
NodeContainer n;                         // node container
std::vector<Ipv4Address> serverAddress;  // server address

// flow generator
std::unordered_map<uint32_t, uint32_t> flows_per_host;
uint32_t flow_id = 0;
std::unordered_map<uint32_t, uint16_t> portNumber;
std::unordered_map<uint32_t, uint16_t> dportNumber;
uint16_t *port_per_host;

// Scheduling input flows from flow.txt
struct FlowInput {
    uint32_t src, dst, pg, maxPacketCount, port;
    double start_time;
    uint32_t idx;
};
FlowInput flow_input = {0};  // global variable
uint32_t flow_num;

namespace {
void ProgressLog(const char *fmt, ...);
extern const char *g_routeDumpPhase;
std::string NodeName(Ptr<Node> node);
std::string HostIpString(Ptr<Node> node);
std::string JoinNodeNames(const std::vector<Ptr<Node>> &nodes);
void DumpHostRoutingState(const char *phase);
void DumpSwitchViewToHosts(const char *phase);
}

void ParseBurstLossSchedule() {
    burst_loss_periods.clear();
    if (burst_loss_schedule.empty()) {
        return;
    }

    std::stringstream schedule(burst_loss_schedule);
    std::string item;
    while (std::getline(schedule, item, ',')) {
        if (item.empty()) {
            continue;
        }
        std::stringstream period_stream(item);
        std::string start, end, rate;
        if (!std::getline(period_stream, start, ':') ||
            !std::getline(period_stream, end, ':') ||
            !std::getline(period_stream, rate, ':')) {
            NS_FATAL_ERROR("Invalid BURST_LOSS_SCHEDULE segment: " << item);
        }
        std::string extra;
        if (std::getline(period_stream, extra, ':')) {
            NS_FATAL_ERROR("Invalid BURST_LOSS_SCHEDULE segment: " << item);
        }

        BurstLossPeriod period;
        period.start_ns = std::stoull(start);
        period.end_ns = std::stoull(end);
        period.loss_rate = std::stod(rate);
        if (period.end_ns <= period.start_ns) {
            NS_FATAL_ERROR("BURST_LOSS_SCHEDULE end_ns must be greater than start_ns: " << item);
        }
        if (period.loss_rate < 0.0 || period.loss_rate > 1.0) {
            NS_FATAL_ERROR("BURST_LOSS_SCHEDULE loss_rate must be in [0, 1]: " << item);
        }
        burst_loss_periods.push_back(period);
    }
}

void RegisterBurstLossErrorModel(Ptr<RateErrorModel> rem) {
    RateErrorModel *raw = PeekPointer(rem);
    if (burst_loss_error_model_seen.insert(raw).second) {
        burst_loss_error_models.push_back(rem);
    }
}

void ApplyBurstLossRate(double rate, const char *event) {
    for (auto rem : burst_loss_error_models) {
        rem->SetAttribute("ErrorRate", DoubleValue(rate));
    }
    if (burst_loss_log_output != NULL) {
        fprintf(burst_loss_log_output, "%lu %.12g %s\n", Simulator::Now().GetNanoSeconds(), rate,
                event);
        fflush(burst_loss_log_output);
    }
}

void ScheduleBurstLossEvents() {
    if (burst_loss_periods.empty()) {
        return;
    }
    burst_loss_log_output = fopen(burst_loss_log_file.c_str(), "w");
    if (burst_loss_log_output == NULL) {
        NS_FATAL_ERROR("Cannot open BURST_LOSS_LOG_FILE: " << burst_loss_log_file);
    }
    for (const auto &period : burst_loss_periods) {
        Simulator::Schedule(NanoSeconds(period.start_ns), &ApplyBurstLossRate, period.loss_rate,
                            "burst_start");
        Simulator::Schedule(NanoSeconds(period.end_ns), &ApplyBurstLossRate, error_rate_per_link,
                            "burst_end");
    }
}

void RecordBurstTxDataPacket(Ipv4Address sip, Ipv4Address dip, uint32_t bytes, bool is_retrans) {
    if (burst_stat_interval_ns == 0) {
        return;
    }
    uint64_t now_ns = Simulator::Now().GetNanoSeconds();
    if (now_ns < burst_stat_start_ns) {
        return;
    }
    if (burst_stat_end_ns != 0 && now_ns >= burst_stat_end_ns) {
        return;
    }
    uint64_t bucket = (now_ns - burst_stat_start_ns) / burst_stat_interval_ns;
    auto key = std::make_pair(Settings::ip_to_node_id(sip), Settings::ip_to_node_id(dip));
    auto &buckets = burst_flow_stats[key];
    if (buckets.size() <= bucket) {
        buckets.resize(bucket + 1);
    }
    buckets[bucket].tx_data_bytes += bytes;
    buckets[bucket].tx_data_pkts += 1;
    if (is_retrans) {
        buckets[bucket].retrans_data_pkts += 1;
    }
}

void RecordBurstRxDataPacket(Ipv4Address sip, Ipv4Address dip, uint32_t bytes) {
    if (burst_stat_interval_ns == 0) {
        return;
    }
    uint64_t now_ns = Simulator::Now().GetNanoSeconds();
    if (now_ns < burst_stat_start_ns) {
        return;
    }
    if (burst_stat_end_ns != 0 && now_ns >= burst_stat_end_ns) {
        return;
    }
    uint64_t bucket = (now_ns - burst_stat_start_ns) / burst_stat_interval_ns;
    auto key = std::make_pair(Settings::ip_to_node_id(sip), Settings::ip_to_node_id(dip));
    auto &buckets = burst_flow_stats[key];
    if (buckets.size() <= bucket) {
        buckets.resize(bucket + 1);
    }
    buckets[bucket].rx_data_bytes += bytes;
}

void WriteBurstStatOutputs() {
    if (burst_stat_interval_ns == 0) {
        return;
    }
    if (burst_goodput_output_file.empty() && burst_redundancy_output_file.empty()) {
        return;
    }

    size_t bucket_count = 0;
    for (const auto &flow : burst_flow_stats) {
        bucket_count = std::max(bucket_count, flow.second.size());
    }
    if (burst_stat_end_ns != 0) {
        uint64_t stat_duration_ns = burst_stat_end_ns - burst_stat_start_ns;
        bucket_count = (stat_duration_ns + burst_stat_interval_ns - 1) / burst_stat_interval_ns;
    }

    if (!burst_goodput_output_file.empty()) {
        std::ofstream out(burst_goodput_output_file.c_str());
        if (!out.is_open()) {
            NS_FATAL_ERROR("Cannot open BURST_GOODPUT_OUTPUT_FILE: " << burst_goodput_output_file);
        }
        out << std::setprecision(12);
        for (const auto &flow : burst_flow_stats) {
            out << flow.first.first << " " << flow.first.second;
            for (size_t i = 0; i < bucket_count; ++i) {
                double goodput_gbps = 0.0;
                if (i < flow.second.size()) {
                    goodput_gbps =
                        flow.second[i].rx_data_bytes * 8.0 / burst_stat_interval_ns;
                }
                out << " " << goodput_gbps;
            }
            out << "\n";
        }
    }

    if (!burst_redundancy_output_file.empty()) {
        std::ofstream out(burst_redundancy_output_file.c_str());
        if (!out.is_open()) {
            NS_FATAL_ERROR("Cannot open BURST_REDUNDANCY_OUTPUT_FILE: "
                           << burst_redundancy_output_file);
        }
        out << std::setprecision(12);
        for (const auto &flow : burst_flow_stats) {
            out << flow.first.first << " " << flow.first.second;
            for (size_t i = 0; i < bucket_count; ++i) {
                double redundancy_rate = 0.0;
                if (i < flow.second.size() && flow.second[i].tx_data_pkts > 0) {
                    redundancy_rate =
                        static_cast<double>(flow.second[i].retrans_data_pkts) /
                        flow.second[i].tx_data_pkts;
                }
                out << " " << redundancy_rate;
            }
            out << "\n";
        }
    }
}

/**
 * Read flow input from file "flowf"
 */
void ReadFlowInput() {
    if (flow_input.idx < flow_num) {
        flowf >> flow_input.src >> flow_input.dst >> flow_input.pg >> flow_input.maxPacketCount >>
            flow_input.start_time;

        if (flow_input.src >= Settings::node_num || flow_input.dst >= Settings::node_num) {
            NS_FATAL_ERROR("Invalid flow endpoint in " << flow_file << ": src="
                                                         << flow_input.src << " dst="
                                                         << flow_input.dst << " node_num="
                                                         << Settings::node_num);
        }

        Ptr<Node> srcNode = n.Get(flow_input.src);
        Ptr<Node> dstNode = n.Get(flow_input.dst);
        if (srcNode == 0 || dstNode == 0 || srcNode->GetNodeType() != 0 ||
            dstNode->GetNodeType() != 0) {
            NS_FATAL_ERROR("Flow endpoints must be hosts in " << flow_file << ": src="
                                                                 << flow_input.src << " dst="
                                                                 << flow_input.dst);
        }
    } else {
        std::cout << "*** input flow is over the prefixed number -- flow number : " << flow_num
                  << std::endl;
        std::cout << "*** flow_input.idx : " << flow_input.idx << std::endl;
        std::cout << "*** THIS IS THE LAST FLOW TO SEND :) " << std::endl;
    }
}

/**
 * Scheduling flows given in /config/L_XX....txt file
 */
void ScheduleFlowInputs(FILE *infile) {
    NS_LOG_DEBUG("ScheduleFlowInputs at " << Simulator::Now());
    while (flow_input.idx < flow_num && Seconds(flow_input.start_time) == Simulator::Now()) {
        uint32_t pg, src, dst, sport, dport, maxPacketCount, target_len;
        pg = flow_input.pg;
        src = flow_input.src;
        dst = flow_input.dst;

        // src port
        sport = portNumber[src];  // get a new port number
        portNumber[src] = portNumber[src] + 1;

        // dst port
        dport = dportNumber[dst];
        dportNumber[dst] = dportNumber[dst] + 1;

        target_len = flow_input.maxPacketCount;  // this is actually not packet-count, but bytes
        if (target_len == 0) {
            target_len = 1;
        }
        assert(n.Get(src)->GetNodeType() == 0 && n.Get(dst)->GetNodeType() == 0);

        /**
         * Turn on if you want to record all input streams into output file for logging.
         * But, the input stream can be found in config. We do not recommend to do this
         * as it consumes storage resource, redundantly.
         */
        if (0) {  // logging input streams to "XXXX_out_in.txt"
            /************************
             * record flow's 4-tuple
             ************************/
            fprintf(infile, "%u %u %u %u %u %lu\n", src, dst, sport, dport, target_len,
                    (uint64_t)(flow_input.start_time * (uint64_t)1000000000));
            fflush(infile);

            /***********    FCT Tracking    **************/
            UdpServerHelper server0(dport);
            server0.SetAttribute("FlowSize", UintegerValue(target_len));
            server0.SetAttribute("irn", BooleanValue(enable_irn));
            server0.SetAttribute("StatHostSrc", UintegerValue(src));
            server0.SetAttribute("StatHostDst", UintegerValue(dst));
            server0.SetAttribute("StatRxLen", UintegerValue(target_len));
            server0.SetAttribute("StatFlowID", UintegerValue(flow_input.idx));
            server0.SetAttribute("Port", UintegerValue(dport));

            ApplicationContainer apps0s = server0.Install(n.Get(dst));  // DST
            apps0s.Start(Seconds(Time(0)));
            apps0s.Stop(Seconds(100.0));
        }  // end of logging input streams

        if (pairRtt.find(n.Get(src)) == pairRtt.end() ||
            pairRtt[n.Get(src)].find(n.Get(dst)) == pairRtt[n.Get(src)].end()) {
            std::cerr << "pairRtt src: " << src << " -> dst: " << dst
                      << " ==> cannot be found from database" << std::endl;
            assert(false);
        }

        RdmaClientHelper clientHelper(
            pg, serverAddress[src], serverAddress[dst], sport, dport, target_len,
            has_win ? (global_t == 1 ? maxBdp : pairBdp[n.Get(src)][n.Get(dst)]) : 0,
            global_t == 1 ? maxRtt : pairRtt[n.Get(src)][n.Get(dst)]);
        clientHelper.SetAttribute("StatFlowID", IntegerValue(flow_input.idx));

        ApplicationContainer appCon = clientHelper.Install(n.Get(src));  // SRC
        appCon.Start(Seconds(Time(0)));
        appCon.Stop(Seconds(100.0));

        flow_input.idx++;
        ReadFlowInput();
    }

    // schedule the next time to run this function
    if (flow_input.idx < flow_num) {
        Simulator::Schedule(Seconds(flow_input.start_time) - Simulator::Now(), &ScheduleFlowInputs,
                            infile);
    } else {  // no more flows
        ProgressLog("[PROGRESS] Flow file fully consumed. Scheduled flows=%u\n", flow_input.idx);
    }
}

/**
 * @brief CNP frequency monitoring (timestamp nodeId ECN OoO Total)
 */
void cnp_freq_monitoring(FILE *fout, Ptr<RdmaHw> rdmahw) {
    if (rdmahw->cnp_total > 0) {
        // flush
        fprintf(fout, "%lu %u %u %u %u\n", Simulator::Now().GetNanoSeconds(),
                rdmahw->m_node->GetId(), rdmahw->cnp_by_ecn, rdmahw->cnp_by_ooo, rdmahw->cnp_total);
        fflush(fout);

        // initialize
        rdmahw->cnp_by_ecn = 0;
        rdmahw->cnp_by_ooo = 0;
        rdmahw->cnp_total = 0;
    }

    // recursive callback
    Simulator::Schedule(NanoSeconds(cnp_monitor_bucket), &cnp_freq_monitoring, fout, rdmahw);
}

/**
 * @brief TOR Switch monitoring
 * - VOQ number and uplink throughput at switches
 * - the number of active connections at RNICS
 */
void periodic_monitoring(FILE *fout_voq, FILE *fout_voq_detail, FILE *fout_uplink, FILE *fout_conn,
                         uint32_t *lb_mode) {
    uint32_t lb_mode_val = *lb_mode;
    uint64_t now = Simulator::Now().GetNanoSeconds();
    for (const auto &tor2If : torId2UplinkIf) {  // for each TOR switches
        Ptr<Node> node = n.Get(tor2If.first);    // tor id
        auto swNode = DynamicCast<SwitchNode>(node);
        assert(swNode->m_isToR == true);  // sanity check

        if (lb_mode_val == 9) {  // Conweave
            // monitor VOQ number per switch <time, ToRId, #VOQ, #Pkts>
            uint32_t nVOQ = swNode->m_mmu->m_conweaveRouting.GetNumVOQ();
            uint32_t nVolumeVOQ = swNode->m_mmu->m_conweaveRouting.GetVolumeVOQ();
            fprintf(fout_voq, "%lu,%u,%u,%u\n", now, tor2If.first, nVOQ, nVolumeVOQ);

            // monitor VOQ per destination IP <time, dstip, #VOQ, #Pkts>
            std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>> dip_to_nvoq_npkt;
            for (auto voq : swNode->m_mmu->m_conweaveRouting.GetVOQMap()) {
                auto &nvoq_npkt = dip_to_nvoq_npkt[voq.second.getDIP()];
                nvoq_npkt.first += 1;
                nvoq_npkt.second += voq.second.getQueueSize();
            }
            for (auto x : dip_to_nvoq_npkt) {
                fprintf(fout_voq_detail, "%lu,%u,%u,%u\n", now, x.first, x.second.first,
                        x.second.second);
            }
        }

        // common: monitor TOR's uplink to measure load balancing performance
        for (const auto &iface : tor2If.second) {
            // monitor uplink txBytes <time, ToRId, OutDev, Bytes>
            uint64_t uplink_txbyte = swNode->GetTxBytesOutDev(iface);
            fprintf(fout_uplink, "%lu,%u,%u,%lu\n", now, tor2If.first, iface, uplink_txbyte);
        }
    }

    // common: get number of concurrent connections at each server
    for (uint32_t i = 0; i < Settings::node_num; i++) {
        if (n.Get(i)->GetNodeType() == 0) {  // is server
            Ptr<Node> server = n.Get(i);
            Ptr<RdmaDriver> rdmaDriver = server->GetObject<RdmaDriver>();
            Ptr<RdmaHw> rdmaHw = rdmaDriver->m_rdma;
            // monitor total/active QP number <time, serverId, #ExistingQP, #ActiveQP>
            uint64_t nQP = rdmaHw->m_qpMap.size();
            uint64_t nActiveQP = 0;
            for (auto qp : rdmaHw->m_qpMap) {
                if (qp.second->GetBytesLeft() > 0) {  // conns with bytes left
                    nActiveQP++;
                }
            }
            fprintf(fout_conn, "%lu,%u,%lu,%lu\n", now, i, nQP, nActiveQP);
        }
    }

    if (Simulator::Now() < Seconds(flowgen_stop_time + 0.05)) {
        // recursive callback
        Simulator::Schedule(NanoSeconds(switch_mon_interval), &periodic_monitoring, fout_voq,
                            fout_voq_detail, fout_uplink, fout_conn, lb_mode);  // every 10us
    }
    return;
}

/**
 * @brief Conga timeout number recording
 */
void conga_history_print() {
    std::cout << "\n------------CONGA History---------------" << std::endl;
    std::cout << "Number of flowlet's timeout:" << CongaRouting::nFlowletTimeout
              << "Conga's timeout: " << conga_flowletTimeout << std::endl;
}

/**
 * @brief Letflow timeout number recording
 */
void letflow_history_print() {
    std::cout << "\n------------Letflow History---------------" << std::endl;
    std::cout << "Number of flowlet's timeout:" << LetflowRouting::nFlowletTimeout
              << "\nLetflow's timeout: " << letflow_flowletTimeout << std::endl;
}

/**
 * @brief Conweave rerouting/VOQ number recording
 */
void conweave_history_print() {
    // Conweave params
    std::cout << "\n------ConWeave parameters-----" << std::endl;
    std::cout << "Param - extraReplyDeadline:" << conweave_extraReplyDeadline << std::endl;
    std::cout << "Param - extraVOQFlushTime:" << conweave_extraVOQFlushTime << std::endl;
    std::cout << "Param - txExpiryTime:" << conweave_txExpiryTime << std::endl;
    std::cout << "Param - defaultVOQWaitingTime:" << conweave_defaultVOQWaitingTime << std::endl;
    std::cout << "Param - pathPauseTime:" << conweave_pathPauseTime << std::endl;
    std::cout << "Param - pathAwareRerouting:" << conweave_pathAwareRerouting << std::endl;

    std::cout << "\n------------ConWeave History---------------" << std::endl;
    std::cout << "Number of INIT's Reply sent (RTT_REPLY):" << ConWeaveRouting::m_nReplyInitSent
              << "\nNumber of Timely RTT_REPLY (INIT's Reply):" << ConWeaveRouting::m_nTimelyInitReplied
              << "\nNumber of TAIL's Reply Sent (CLEAR):" << ConWeaveRouting::m_nReplyTailSent
              << "\nNumber of Timely CLEAR (TAIL's Reply):" << ConWeaveRouting::m_nTimelyTailReplied
              << "\nNumber of NOTIFY Sent:" << ConWeaveRouting::m_nNotifySent
              << "\nNumber of Rerouting:" << ConWeaveRouting::m_nReRoute
              << "\nNumber of OoO enqueued pkts:" << ConWeaveRouting::m_nOutOfOrderPkts
              << "\nNumber of VOQ Flush Total:" << ConWeaveRouting::m_nFlushVOQTotal
              << "\nNumber of VOQ Flush From History:" << ConWeaveRouting::m_historyVOQSize.size()
              << "\nNumber of VOQ Flush by TAIL:" << ConWeaveRouting::m_nFlushVOQByTail
              << std::endl;

    std::cout << "--------------------------" << std::endl;

    /** VOQ: Sanity check*/
    for (size_t ToRId = 0; ToRId < Settings::node_num; ToRId++) {
        Ptr<Node> node = n.Get(ToRId);
        if (node->GetNodeType() == 1) {  // switches
            auto swNode = DynamicCast<SwitchNode>(n.Get(ToRId));
            if (swNode->m_isToR) {  // TOR switch
                uint32_t num_remained_voq = swNode->m_mmu->m_conweaveRouting.GetNumVOQ();
                if (num_remained_voq > 0) {
                    printf("*******************************\n");
                    printf("*** WARNING - Tor Sw (%lu) - VOQ (num=%u) is not flushed yet!! ***\n",
                           ToRId, num_remained_voq);
                    printf(
                        " -- Probably the history print is too early so simulation might not be "
                        "finished?");
                    printf("********************************\n");
                }
            }
        }
    }

    /** Get ConWeave Flush Time Estimation Error */
    if (0) {
        // sanity check - extraVOQFlushTime must be large enough to get accuracy
        assert(conweave_extraVOQFlushTime >= MicroSeconds(128) && "PARAMETER ERROR!!");

        std::cout << "\n--------------------------" << std::endl;
        std::cout << "Extracting ConWeave Estimation Error Data..." << std::endl;
        est_error_output = fopen(est_error_output_file.c_str(), "w");
        for (auto x : ConWeaveVOQ::m_flushEstErrorhistory) {
            fprintf(est_error_output, "%d\n", x);
        }
        ConWeaveVOQ::m_flushEstErrorhistory.clear();
        std::cout << "---------D O N E---------" << std::endl;
    }
}

/**
 * @brief When one RDMA is finished, so does (1) QP, (2) RxQP, (3) write it on file fct.txt.
 */
void qp_finish(FILE *fout, Ptr<RdmaQueuePair> q) {
    uint32_t sid = Settings::ip_to_node_id(q->sip), did = Settings::ip_to_node_id(q->dip);
    uint64_t base_rtt = pairRtt[n.Get(sid)][n.Get(did)];
    uint64_t b = pairBw[n.Get(sid)][n.Get(did)];
    uint64_t actual_fct_ns = (Simulator::Now() - q->startTime).GetNanoSeconds();
    uint32_t total_bytes =
        q->m_size + ((q->m_size - 1) / packet_payload_size + 1) *
                        (CustomHeader::GetStaticWholeHeaderSize() -
                         IntHeader::GetStaticSize());  // translate to the minimum bytes required
                                                       // (with header but no INT)
    uint64_t standalone_fct = base_rtt + total_bytes * 8000000000lu / b;
    double throughput_bps = 0.0;
    double goodput_bps = 0.0;
    if (actual_fct_ns > 0) {
        throughput_bps = (total_bytes * 8.0) / actual_fct_ns;
        goodput_bps = (q->m_size * 8.0) / actual_fct_ns;
    }

    // XXX: remove rxQP from the receiver
    Ptr<Node> dstNode = n.Get(did);
    Ptr<RdmaDriver> rdma = dstNode->GetObject<RdmaDriver>();
    rdma->m_rdma->DeleteRxQp(q->sip.Get(), q->sport, q->dport, q->m_pg);

    fprintf(fout, "%u %u %u %u %lu %lu %lu %lu %lu %lu %.12g %.12g\n",
            Settings::ip_to_node_id(q->sip),
            Settings::ip_to_node_id(q->dip), q->sport, q->dport, q->m_size,
            q->startTime.GetTimeStep(), (Simulator::Now() - q->startTime).GetTimeStep(),
            standalone_fct, q->stat.txRetransPkts, q->stat.pathSwitchCount, throughput_bps,
            goodput_bps);

    // for debugging
    NS_LOG_DEBUG(Settings::ip_to_node_id(q->sip) << " " << Settings::ip_to_node_id(q->dip) << " "
                                                  << q->sport << " " << q->dport << " "
                                                  << q->m_size << " "
                                                  << q->startTime.GetTimeStep() << " "
                                                  << (Simulator::Now() - q->startTime).GetTimeStep()
                                                  << " " << standalone_fct << " "
                                                  << q->stat.txRetransPkts << " "
                                                  << q->stat.pathSwitchCount << " "
                                                  << throughput_bps << " " << goodput_bps);
    Settings::cnt_finished_flows++;
    fflush(fout);
}

/**
 * @brief PFC event logging
 */
void get_pfc(FILE *fout, Ptr<QbbNetDevice> dev, uint32_t type) {
    // time, nodeID, nodeType, Interface's Idx, 0:resume, 1:pause
    fprintf(fout, "%lu %u %u %u %u\n", Simulator::Now().GetTimeStep(), dev->GetNode()->GetId(),
            dev->GetNode()->GetNodeType(), dev->GetIfIndex(), type);
}

/*******************************************************************/
#if (false)

/**
 * @brief Qlen monitoring at switches (output: qlen.txt), I think "periodically"...
 *
 */
struct QlenDistribution {
    vector<uint32_t> cnt;  // cnt[i] is the number of times that the queue len is i KB
    void add(uint32_t qlen) {
        uint32_t kb = qlen / 1000;
        if (cnt.size() < kb + 1) cnt.resize(kb + 1);
        cnt[kb]++;
    }
};

map<uint32_t, map<uint32_t, QlenDistribution>> queue_result;
void monitor_buffer(FILE *qlen_output, NodeContainer *n) {
    /*******************************************************************/
    /************************** UNUSED NOW *****************************/
    /*******************************************************************/
    for (uint32_t i = 0; i < n->GetN(); i++) {
        if (n->Get(i)->GetNodeType() == 1) {  // is switch
            Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n->Get(i));
            if (queue_result.find(i) == queue_result.end()) queue_result[i];
            for (uint32_t j = 1; j < sw->GetNDevices(); j++) {
                uint32_t size = 0;
                for (uint32_t k = 0; k < SwitchMmu::qCnt; k++)
                    size += sw->m_mmu->egress_bytes[j][k];
                queue_result[i][j].add(size);
            }
        }
    }
    if (Simulator::Now().GetTimeStep() % qlen_dump_interval == 0) {
        fprintf(qlen_output, "time: %lu\n", Simulator::Now().GetTimeStep());
        for (auto &it0 : queue_result) {
            for (auto &it1 : it0.second) {
                fprintf(qlen_output, "%u %u", it0.first, it1.first);
                auto &dist = it1.second.cnt;
                for (uint32_t i = 0; i < dist.size(); i++) fprintf(qlen_output, " %u", dist[i]);
                fprintf(qlen_output, "\n");
            }
        }
        fflush(qlen_output);
    }
    if (Simulator::Now().GetTimeStep() < qlen_mon_end)
        Simulator::Schedule(NanoSeconds(qlen_mon_interval), &monitor_buffer, qlen_output, n);
}
#endif
/*******************************************************************/

/**
 * @brief Stop simulation in the middle (when almost all flows are done).
 * This function allows to finish simulation quickly when all messages are sent.
 */
void stop_simulation_middle() {
    uint32_t target_flow_num = flow_num - 0;  // can be lower than flownum
    if (Settings::cnt_finished_flows >= target_flow_num) {
        std::cout << "\n*** Simulator is enforced to be finished, finished so far: "
                  << Settings::cnt_finished_flows << "/ total: " << target_flow_num
                  << ", Time:" << Simulator::Now() << std::endl;

        // schedule conga timeout monitor
        if (lb_mode == 3) {  // CONGA
            conga_history_print();
        }
        if (lb_mode == 6) {  // LETFLOW
            letflow_history_print();
        }
        if (lb_mode == 9) {  // CONWEAVE
            conweave_history_print();
        }
        Simulator::Stop(NanoSeconds(1));  // finish soon, stop this schedule (NECESSARY!)
        return;
    }

    Simulator::Schedule(MicroSeconds(100), &stop_simulation_middle);  // check every 100us
}

/**
 * @brief Calculate edge-to-edge delays, TX delays, and bandwidths
 */
void CalculateRoute(Ptr<Node> host) {
    // queue for the BFS.
    vector<Ptr<Node>> q;
    // Distance from the host to each node.
    map<Ptr<Node>, int> dis;
    map<Ptr<Node>, uint64_t> delay;
    map<Ptr<Node>, uint64_t> txDelay;
    map<Ptr<Node>, uint64_t> bw;
    // init BFS.
    q.push_back(host);
    dis[host] = 0;
    delay[host] = 0;
    txDelay[host] = 0;
    bw[host] = 0xfffffffffffffffflu;

    // BFS.
    for (int i = 0; i < (int)q.size(); i++) {
        Ptr<Node> now = q[i];
        int d = dis[now];
        for (auto it = nbr2if[now].begin(); it != nbr2if[now].end(); it++) {
            // skip down link
            if (!it->second.up) continue;
            Ptr<Node> next = it->first;
            // If 'next' have not been visited.
            if (dis.find(next) == dis.end()) {
                dis[next] = d + 1;
                delay[next] = delay[now] + it->second.delay;  // maybe nanoseconds?
                txDelay[next] = txDelay[now] + packet_payload_size * 1000000000lu * 8 /
                                                   it->second.bw;  // maybe nanoseconds?
                bw[next] = std::min(bw[now], it->second.bw);
                // we only enqueue switch, because we do not want packets to go through host as
                // middle point
                if (next->GetNodeType() == 1) {
                    q.push_back(next);
                }
            }
            // if 'now' is on the shortest path from 'next' to 'host'.
            if (d + 1 == dis[next]) {
                nextHop[next][host].push_back(now);
            }
        }
    }
    for (auto it : delay) {
        pairDelay[it.first][host] = it.second;
    }
    for (auto it : txDelay) {
        pairTxDelay[it.first][host] = it.second;
    }
    for (auto it : bw) {
        pairBw[it.first][host] = it.second;
    }
}
void CalculateRoutes(NodeContainer &n) {
    for (int i = 0; i < (int)n.GetN(); i++) {
        Ptr<Node> node = n.Get(i);
        if (node->GetNodeType() == 0) {
            CalculateRoute(node);
        }
    }
}

/**
 * @brief Set the Routing Entries object
 */
void SetRoutingEntries() {
    // For each node.
    for (auto i = nextHop.begin(); i != nextHop.end(); i++) {
        Ptr<Node> node = i->first;
        auto &table = i->second;
        for (auto j = table.begin(); j != table.end(); j++) {
            // The destination node.
            Ptr<Node> dst = j->first;
            // The IP address of the dst.
            Ipv4Address dstAddr = dst->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
            // The next hops towards the dst.
            vector<Ptr<Node>> nexts = j->second;
            for (int k = 0; k < (int)nexts.size(); k++) {
                Ptr<Node> next = nexts[k];
                uint32_t interface = nbr2if[node][next].idx;
                if (node->GetNodeType() == 1)
                    DynamicCast<SwitchNode>(node)->AddTableEntry(dstAddr, interface);
                else {
                    node->GetObject<RdmaDriver>()->m_rdma->AddTableEntry(dstAddr, interface);
                }
            }
        }
    }
    DumpHostRoutingState(g_routeDumpPhase);
    DumpSwitchViewToHosts(g_routeDumpPhase);
}
/**
 * @brief take down the link between a and b, and redo the routing
 */
void TakeDownLink(NodeContainer n, Ptr<Node> a, Ptr<Node> b) {
    if (!nbr2if[a][b].up) return;
    ProgressLog("[LINK_DOWN] request a=%s b=%s ifA=%u ifB=%u\n", NodeName(a).c_str(),
                NodeName(b).c_str(), nbr2if[a][b].idx, nbr2if[b][a].idx);
    // take down link between a and b
    nbr2if[a][b].up = nbr2if[b][a].up = false;
    nextHop.clear();
    CalculateRoutes(n);
    // clear routing tables
    for (uint32_t i = 0; i < n.GetN(); i++) {
        if (n.Get(i)->GetNodeType() == 1)
            DynamicCast<SwitchNode>(n.Get(i))->ClearTable();
        else
            n.Get(i)->GetObject<RdmaDriver>()->m_rdma->ClearTable();
    }
    DynamicCast<QbbNetDevice>(a->GetDevice(nbr2if[a][b].idx))->TakeDown();
    DynamicCast<QbbNetDevice>(b->GetDevice(nbr2if[b][a].idx))->TakeDown();
    // reset routing table
    g_routeDumpPhase = "after_link_down";
    SetRoutingEntries();
    g_routeDumpPhase = "initial";

    // redistribute qp on each host
    for (uint32_t i = 0; i < n.GetN(); i++) {
        if (n.Get(i)->GetNodeType() == 0)
            n.Get(i)->GetObject<RdmaDriver>()->m_rdma->RedistributeQp();
    }
}

uint64_t get_nic_rate(NodeContainer &n) {
    uint64_t avg_nic_rate = 0;
    uint64_t n_servers = 0;
    for (uint32_t i = 0; i < n.GetN(); i++) {
        if (n.Get(i)->GetNodeType() == 0) {
            if (n.Get(i)->GetNDevices() <= 1) {
                ProgressLog("[PROGRESS] Host %u has no RDMA NIC device at index 1. Skip rate accounting.\n",
                            i);
                continue;
            }
            Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(n.Get(i)->GetDevice(1));
            if (dev == 0) {
                ProgressLog("[PROGRESS] Host %u device[1] is not a QbbNetDevice. Skip rate accounting.\n",
                            i);
                continue;
            }
            avg_nic_rate += dev->GetDataRate().GetBitRate();
            n_servers += 1;
        }
    }
    if (n_servers == 0) {
        return 0;
    }
    return avg_nic_rate / n_servers;
}

/************************************************************************/
//                                                                      //
//                                M A I N                               //
//                                                                      //
/************************************************************************/

namespace {

FILE *g_progressStream = nullptr;
const char *g_routeDumpPhase = "initial";

std::string NodeName(Ptr<Node> node) {
    if (node == nullptr) {
        return "null";
    }
    std::ostringstream oss;
    oss << node->GetId() << (node->GetNodeType() == 0 ? "(H)" : "(S)");
    return oss.str();
}

std::string HostIpString(Ptr<Node> node) {
    if (node == nullptr) {
        return "null";
    }
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (ipv4 == nullptr || ipv4->GetNInterfaces() <= 1) {
        return "no-ip";
    }
    std::ostringstream oss;
    oss << ipv4->GetAddress(1, 0).GetLocal();
    return oss.str();
}

std::string JoinNodeNames(const std::vector<Ptr<Node>> &nodes) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (i != 0) {
            oss << ",";
        }
        oss << NodeName(nodes[i]);
    }
    oss << "]";
    return oss.str();
}

void InitProgressLog() {
    if (g_progressStream != nullptr) {
        return;
    }
    int dupFd = dup(fileno(stderr));
    if (dupFd >= 0) {
        g_progressStream = fdopen(dupFd, "w");
    }
    if (g_progressStream == nullptr) {
        g_progressStream = stderr;
    }
}

void ProgressLog(const char *fmt, ...) {
    if (g_progressStream == nullptr) {
        InitProgressLog();
    }
    if (g_progressStream == nullptr) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    vfprintf(g_progressStream, fmt, args);
    va_end(args);
    fflush(g_progressStream);
}

std::string GetConfigPath(int argc, char *argv[]) {
#ifndef PGO_TRAINING
    if (argc > 1) {
        return argv[1];
    }
    return "";
#else
    return PATH_TO_PGO_CONFIG;
#endif
}

std::string DirName(const std::string &path) {
    size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) {
        return ".";
    }
    if (pos == 0) {
        return path.substr(0, 1);
    }
    return path.substr(0, pos);
}

uint32_t ScanConsoleLogLevel(const std::string &configPath) {
    if (configPath.empty()) {
        return 1;
    }

    std::ifstream conf(configPath.c_str());
    if (!conf.is_open()) {
        return 1;
    }

    uint32_t level = 1;
    while (!conf.eof()) {
        std::string key;
        conf >> key;
        if (!conf.good()) {
            break;
        }
        if (key.compare("ENABLE_CONSOLE_LOG") == 0) {
            uint32_t v = 1;
            conf >> v;
            level = v > 2 ? 2 : v;
            break;
        }
    }
    return level;
}

void ConfigureConsoleLogOutput(bool enabled) {
    if (enabled) {
        return;
    }

    fflush(stdout);
    fflush(stderr);
    FILE *stdoutResult = freopen("/dev/null", "w", stdout);
    FILE *stderrResult = freopen("/dev/null", "w", stderr);
    (void)stdoutResult;
    (void)stderrResult;
}

void DumpHostRoutingState(const char *phase) {
    ProgressLog("[ROUTE_DUMP] phase=%s host_count=%u\n", phase, Settings::host_num);
    for (uint32_t src = 0; src < n.GetN(); ++src) {
        Ptr<Node> srcNode = n.Get(src);
        if (srcNode->GetNodeType() != 0) {
            continue;
        }
        for (uint32_t dst = 0; dst < n.GetN(); ++dst) {
            Ptr<Node> dstNode = n.Get(dst);
            if (dstNode->GetNodeType() != 0 || src == dst) {
                continue;
            }
            auto srcIt = nextHop.find(srcNode);
            if (srcIt == nextHop.end()) {
                // ProgressLog("[HOST_ROUTE] phase=%s src=%s dst=%s dstIp=%s status=SRC_MISSING\n",
                //             phase, NodeName(srcNode).c_str(), NodeName(dstNode).c_str(),
                //             HostIpString(dstNode).c_str());
                continue;
            }
            auto dstIt = srcIt->second.find(dstNode);
            if (dstIt == srcIt->second.end()) {
                // ProgressLog("[HOST_ROUTE] phase=%s src=%s dst=%s dstIp=%s status=DST_MISSING\n",
                //             phase, NodeName(srcNode).c_str(), NodeName(dstNode).c_str(),
                //             HostIpString(dstNode).c_str());
                continue;
            }

            const std::vector<Ptr<Node>> &nexts = dstIt->second;
            std::ostringstream ifs;
            ifs << "[";
            for (size_t i = 0; i < nexts.size(); ++i) {
                if (i != 0) {
                    ifs << ",";
                }
                auto nbrIt = nbr2if[srcNode].find(nexts[i]);
                if (nbrIt == nbr2if[srcNode].end()) {
                    ifs << "missing";
                } else {
                    ifs << nbrIt->second.idx;
                }
            }
            ifs << "]";
            // ProgressLog("[HOST_ROUTE] phase=%s src=%s srcIp=%s dst=%s dstIp=%s nextHops=%s ifIdx=%s fanout=%zu\n",
            //             phase, NodeName(srcNode).c_str(), HostIpString(srcNode).c_str(),
            //             NodeName(dstNode).c_str(), HostIpString(dstNode).c_str(),
            //             JoinNodeNames(nexts).c_str(), ifs.str().c_str(), nexts.size());
        }
    }
}

void DumpSwitchViewToHosts(const char *phase) {
    ProgressLog("[NEXTHOP_DUMP] phase=%s\n", phase);
    for (auto i = nextHop.begin(); i != nextHop.end(); ++i) {
        Ptr<Node> node = i->first;
        for (auto j = i->second.begin(); j != i->second.end(); ++j) {
            Ptr<Node> dst = j->first;
            if (dst->GetNodeType() != 0) {
                continue;
            }
            const std::vector<Ptr<Node>> &nexts = j->second;
            std::ostringstream ifs;
            ifs << "[";
            for (size_t k = 0; k < nexts.size(); ++k) {
                if (k != 0) {
                    ifs << ",";
                }
                auto nbrIt = nbr2if[node].find(nexts[k]);
                if (nbrIt == nbr2if[node].end()) {
                    ifs << "missing";
                } else {
                    ifs << nbrIt->second.idx;
                }
            }
            ifs << "]";
            // ProgressLog("[NEXTHOP_VIEW] phase=%s node=%s dst=%s dstIp=%s nextHops=%s ifIdx=%s fanout=%zu\n",
            //             phase, NodeName(node).c_str(), NodeName(dst).c_str(),
            //             HostIpString(dst).c_str(), JoinNodeNames(nexts).c_str(),
            //             ifs.str().c_str(), nexts.size());
        }
    }
}

}  // namespace

int main(int argc, char *argv[]) {
    uint32_t *workload_cdf = nullptr;
    clock_t begint, endt;
    begint = clock();
    InitProgressLog();
#ifndef PGO_TRAINING
    if (argc > 1)
#else
    if (true)
#endif
    {
        std::string configPath = GetConfigPath(argc, argv);
        std::string logDir = DirName(configPath);
        RdmaHw::SetEndpointLogFiles(logDir + "/sender.log", logDir + "/receiver.log");
        enable_console_log = ScanConsoleLogLevel(configPath);
        ConfigureConsoleLogOutput(enable_console_log != 0);
        // Read the configuration file
        std::ifstream conf;
#ifndef PGO_TRAINING
        conf.open(argv[1]);
#else
        conf.open(PATH_TO_PGO_CONFIG);
#endif
        while (!conf.eof()) {
            std::string key;
            conf >> key;
            if (key.compare("FLOW_INPUT_FILE") == 0) {
                std::string v;
                conf >> v;
                flow_input_file = v;
                std::cerr << "FLOW_INPUT_FILE\t\t\t" << flow_input_file << "\n";
            } else if (key.compare("CNP_OUTPUT_FILE") == 0) {
                std::string v;
                conf >> v;
                cnp_output_file = v;
                std::cerr << "CNP_OUTPUT_FILE\t\t\t" << cnp_output_file << "\n";
            } else if (key.compare("EST_ERROR_MON_FILE") == 0) {
                std::string v;
                conf >> v;
                est_error_output_file = v;
                std::cerr << "EST_ERROR_MON_FILE\t\t\t" << est_error_output_file << "\n";
            } else if (key.compare("LB_MODE") == 0) {
                uint32_t v;
                conf >> v;
                lb_mode = v;
                std::cerr << "LB_MODE\t\t\t" << lb_mode << "\n";
            } else if (key.compare("SW_MONITORING_INTERVAL") == 0) {
                uint32_t v;
                conf >> v;
                switch_mon_interval = v;
                std::cerr << "SW_MONITORING_INTERVAL\t\t\t" << switch_mon_interval << "\n";
            } else if (key.compare("CONWEAVE_TX_EXPIRY_TIME") == 0) {
                uint32_t v;
                conf >> v;
                conweave_txExpiryTime = Time(MicroSeconds(v));
                std::cerr << "CONWEAVE_TX_EXPIRY_TIME\t\t\t" << conweave_txExpiryTime << "\n";
            } else if (key.compare("CONWEAVE_REPLY_TIMEOUT_EXTRA") == 0) {
                uint32_t v;
                conf >> v;
                conweave_extraReplyDeadline = Time(MicroSeconds(v));
                std::cerr << "CONWEAVE_REPLY_TIMEOUT_EXTRA\t\t\t" << conweave_extraReplyDeadline
                          << "\n";
            } else if (key.compare("CONWEAVE_EXTRA_VOQ_FLUSH_TIME") == 0) {
                uint32_t v;
                conf >> v;
                conweave_extraVOQFlushTime = Time(MicroSeconds(v));
                std::cerr << "CONWEAVE_EXTRA_VOQ_FLUSH_TIME\t\t\t" << conweave_extraVOQFlushTime
                          << "\n";
            } else if (key.compare("CONWEAVE_PATH_PAUSE_TIME") == 0) {
                uint32_t v;
                conf >> v;
                conweave_pathPauseTime = Time(MicroSeconds(v));
                std::cerr << "CONWEAVE_PATH_PAUSE_TIME\t\t\t" << conweave_pathPauseTime << "\n";
            } else if (key.compare("CONWEAVE_DEFAULT_VOQ_WAITING_TIME") == 0) {
                uint32_t v;
                conf >> v;
                conweave_defaultVOQWaitingTime = Time(MicroSeconds(v));
                std::cerr << "CONWEAVE_DEFAULT_VOQ_WAITING_TIME\t\t\t"
                          << conweave_defaultVOQWaitingTime << "\n";
            } else if (key.compare("ENABLE_PFC") == 0) {
                uint32_t v;
                conf >> v;
                enable_pfc = v;
                if (enable_pfc)
                    std::cerr << "ENABLE_PFC\t\t\t"
                              << "Yes"
                              << "\n";
                else
                    std::cerr << "ENABLE_PFC\t\t\t"
                              << "No"
                              << "\n";
            } else if (key.compare("ENABLE_QCN") == 0) {
                uint32_t v;
                conf >> v;
                enable_qcn = v;
                if (enable_qcn)
                    std::cerr << "ENABLE_QCN\t\t\t"
                              << "Yes"
                              << "\n";
                else
                    std::cerr << "ENABLE_QCN\t\t\t"
                              << "No"
                              << "\n";
            } else if (key.compare("USE_DYNAMIC_PFC_THRESHOLD") == 0) {
                uint32_t v;
                conf >> v;
                use_dynamic_pfc_threshold = v;
                if (use_dynamic_pfc_threshold)
                    std::cerr << "USE_DYNAMIC_PFC_THRESHOLD\t"
                              << "Yes"
                              << "\n";
                else
                    std::cerr << "USE_DYNAMIC_PFC_THRESHOLD\t"
                              << "No"
                              << "\n";
            } else if (key.compare("CLAMP_TARGET_RATE") == 0) {
                uint32_t v;
                conf >> v;
                clamp_target_rate = v;
                if (clamp_target_rate)
                    std::cerr << "CLAMP_TARGET_RATE\t\t"
                              << "Yes"
                              << "\n";
                else
                    std::cerr << "CLAMP_TARGET_RATE\t\t"
                              << "No"
                              << "\n";
            } else if (key.compare("PAUSE_TIME") == 0) {
                double v;
                conf >> v;
                pause_time = v;
                std::cerr << "PAUSE_TIME\t\t\t" << pause_time << "\n";
            } else if (key.compare("DATA_RATE") == 0) {
                std::string v;
                conf >> v;
                data_rate = v;
                std::cerr << "DATA_RATE\t\t\t" << data_rate << "\n";
            } else if (key.compare("LINK_DELAY") == 0) {
                std::string v;
                conf >> v;
                link_delay = v;
                std::cerr << "LINK_DELAY\t\t\t" << link_delay << "\n";
            } else if (key.compare("PACKET_PAYLOAD_SIZE") == 0) {
                uint32_t v;
                conf >> v;
                packet_payload_size = v;
                std::cerr << "PACKET_PAYLOAD_SIZE\t\t" << packet_payload_size << "\n";
            } else if (key.compare("L2_CHUNK_SIZE") == 0) {
                uint32_t v;
                conf >> v;
                l2_chunk_size = v;
                std::cerr << "L2_CHUNK_SIZE\t\t\t" << l2_chunk_size << "\n";
            } else if (key.compare("L2_ACK_INTERVAL") == 0) {
                uint32_t v;
                conf >> v;
                l2_ack_interval = v;
                std::cerr << "L2_ACK_INTERVAL\t\t\t" << l2_ack_interval << "\n";
            } else if (key.compare("L2_BACK_TO_ZERO") == 0) {
                uint32_t v;
                conf >> v;
                l2_back_to_zero = v;
                if (l2_back_to_zero)
                    std::cerr << "L2_BACK_TO_ZERO\t\t\t"
                              << "Yes"
                              << "\n";
                else
                    std::cerr << "L2_BACK_TO_ZERO\t\t\t"
                              << "No"
                              << "\n";
            } else if (key.compare("TOPOLOGY_FILE") == 0) {
                std::string v;
                conf >> v;
                topology_file = v;
                std::cerr << "TOPOLOGY_FILE\t\t\t" << topology_file << "\n";
            } else if (key.compare("FLOW_FILE") == 0) {
                std::string v;
                conf >> v;
                flow_file = v;
                std::cerr << "FLOW_FILE\t\t\t" << flow_file << "\n";
            } else if (key.compare("FLOWGEN_START_TIME") == 0) {
                double v;
                conf >> v;
                flowgen_start_time = v;
                qlen_mon_start = v;
                qlen_mon_end = v;
                cnp_mon_start = v;
                irn_mon_start = v;
                std::cerr << "FLOWGEN_START_TIME\t\t" << flowgen_start_time << "\n";
            } else if (key.compare("FLOWGEN_STOP_TIME") == 0) {
                double v;
                conf >> v;
                flowgen_stop_time = v;
                std::cerr << "FLOWGEN_STOP_TIME\t\t" << flowgen_stop_time << "\n";
            } else if (key.compare("ALPHA_RESUME_INTERVAL") == 0) {
                double v;
                conf >> v;
                alpha_resume_interval = v;
                std::cerr << "ALPHA_RESUME_INTERVAL\t\t" << alpha_resume_interval << "\n";
            } else if (key.compare("RP_TIMER") == 0) {
                double v;
                conf >> v;
                rp_timer = v;
                std::cerr << "RP_TIMER\t\t\t" << rp_timer << "\n";
            } else if (key.compare("EWMA_GAIN") == 0) {
                double v;
                conf >> v;
                ewma_gain = v;
                std::cerr << "EWMA_GAIN\t\t\t" << ewma_gain << "\n";
            } else if (key.compare("FAST_RECOVERY_TIMES") == 0) {
                uint32_t v;
                conf >> v;
                fast_recovery_times = v;
                std::cerr << "FAST_RECOVERY_TIMES\t\t" << fast_recovery_times << "\n";
            } else if (key.compare("RATE_AI") == 0) {
                std::string v;
                conf >> v;
                rate_ai = v;
                std::cerr << "RATE_AI\t\t\t\t" << rate_ai << "\n";
            } else if (key.compare("RATE_HAI") == 0) {
                std::string v;
                conf >> v;
                rate_hai = v;
                std::cerr << "RATE_HAI\t\t\t" << rate_hai << "\n";
            } else if (key.compare("ERROR_RATE_PER_LINK") == 0) {
                double v;
                conf >> v;
                error_rate_per_link = v;
                std::cerr << "ERROR_RATE_PER_LINK\t\t" << error_rate_per_link << "\n";
            } else if (key.compare("BURST_LOSS_SCHEDULE") == 0) {
                conf >> burst_loss_schedule;
                std::cerr << "BURST_LOSS_SCHEDULE\t\t" << burst_loss_schedule << "\n";
            } else if (key.compare("BURST_LOSS_LOG_FILE") == 0) {
                conf >> burst_loss_log_file;
                std::cerr << "BURST_LOSS_LOG_FILE\t\t" << burst_loss_log_file << "\n";
            } else if (key.compare("BURST_STAT_START_NS") == 0) {
                conf >> burst_stat_start_ns;
                std::cerr << "BURST_STAT_START_NS\t\t" << burst_stat_start_ns << "\n";
            } else if (key.compare("BURST_STAT_END_NS") == 0) {
                conf >> burst_stat_end_ns;
                std::cerr << "BURST_STAT_END_NS\t\t" << burst_stat_end_ns << "\n";
            } else if (key.compare("BURST_STAT_INTERVAL_NS") == 0) {
                conf >> burst_stat_interval_ns;
                std::cerr << "BURST_STAT_INTERVAL_NS\t\t" << burst_stat_interval_ns << "\n";
            } else if (key.compare("BURST_GOODPUT_OUTPUT_FILE") == 0) {
                conf >> burst_goodput_output_file;
                std::cerr << "BURST_GOODPUT_OUTPUT_FILE\t" << burst_goodput_output_file << "\n";
            } else if (key.compare("BURST_REDUNDANCY_OUTPUT_FILE") == 0) {
                conf >> burst_redundancy_output_file;
                std::cerr << "BURST_REDUNDANCY_OUTPUT_FILE\t" << burst_redundancy_output_file
                          << "\n";
            } else if (key.compare("CC_MODE") == 0) {
                conf >> cc_mode;
                std::cerr << "CC_MODE\t\t" << cc_mode << '\n';
            } else if (key.compare("RATE_DECREASE_INTERVAL") == 0) {
                double v;
                conf >> v;
                rate_decrease_interval = v;
                std::cerr << "RATE_DECREASE_INTERVAL\t\t" << rate_decrease_interval << "\n";
            } else if (key.compare("MIN_RATE") == 0) {
                conf >> min_rate;
                std::cerr << "MIN_RATE\t\t" << min_rate << "\n";
            } else if (key.compare("FCT_OUTPUT_FILE") == 0) {
                conf >> fct_output_file;
                std::cerr << "FCT_OUTPUT_FILE\t\t" << fct_output_file << '\n';
            } else if (key.compare("HAS_WIN") == 0) {
                conf >> has_win;
                std::cerr << "HAS_WIN\t\t" << has_win << "\n";
            } else if (key.compare("GLOBAL_T") == 0) {
                conf >> global_t;
                std::cerr << "GLOBAL_T\t\t" << global_t << '\n';
            } else if (key.compare("MI_THRESH") == 0) {
                conf >> mi_thresh;
                std::cerr << "MI_THRESH\t\t" << mi_thresh << '\n';
            } else if (key.compare("VAR_WIN") == 0) {
                uint32_t v;
                conf >> v;
                var_win = v;
                std::cerr << "VAR_WIN\t\t" << v << '\n';
            } else if (key.compare("FAST_REACT") == 0) {
                uint32_t v;
                conf >> v;
                fast_react = v;
                std::cerr << "FAST_REACT\t\t" << v << '\n';
            } else if (key.compare("U_TARGET") == 0) {
                conf >> u_target;
                std::cerr << "U_TARGET\t\t" << u_target << '\n';
            } else if (key.compare("INT_MULTI") == 0) {
                conf >> int_multi;
                std::cerr << "INT_MULTI\t\t\t\t" << int_multi << '\n';
            } else if (key.compare("RATE_BOUND") == 0) {
                uint32_t v;
                conf >> v;
                rate_bound = v;
                std::cerr << "RATE_BOUND\t\t" << rate_bound << '\n';
            } else if (key.compare("DCTCP_RATE_AI") == 0) {
                conf >> dctcp_rate_ai;
                std::cerr << "DCTCP_RATE_AI\t\t\t\t" << dctcp_rate_ai << "\n";
            } else if (key.compare("PFC_OUTPUT_FILE") == 0) {
                conf >> pfc_output_file;
                std::cerr << "PFC_OUTPUT_FILE\t\t\t\t" << pfc_output_file << '\n';
            } else if (key.compare("LINK_DOWN") == 0) {
                conf >> link_down_time >> link_down_A >> link_down_B;
                std::cerr << "LINK_DOWN\t\t\t\t" << link_down_time << ' ' << link_down_A << ' '
                          << link_down_B << '\n';
            } else if (key.compare("KMAX_MAP") == 0) {
                int n_k;
                conf >> n_k;
                std::cerr << "KMAX_MAP\t\t\t\t";
                for (int i = 0; i < n_k; i++) {
                    uint64_t rate;
                    uint32_t k;
                    conf >> rate >> k;
                    rate2kmax[rate] = k;
                    std::cerr << ' ' << rate << ' ' << k;
                }
                std::cerr << '\n';
            } else if (key.compare("KMIN_MAP") == 0) {
                int n_k;
                conf >> n_k;
                std::cerr << "KMIN_MAP\t\t\t\t";
                for (int i = 0; i < n_k; i++) {
                    uint64_t rate;
                    uint32_t k;
                    conf >> rate >> k;
                    rate2kmin[rate] = k;
                    std::cerr << ' ' << rate << ' ' << k;
                }
                std::cerr << '\n';
            } else if (key.compare("PMAX_MAP") == 0) {
                int n_k;
                conf >> n_k;
                std::cerr << "PMAX_MAP\t\t\t\t";
                for (int i = 0; i < n_k; i++) {
                    uint64_t rate;
                    double p;
                    conf >> rate >> p;
                    rate2pmax[rate] = p;
                    std::cerr << ' ' << rate << ' ' << p;
                }
                std::cerr << '\n';
            } else if (key.compare("BUFFER_SIZE") == 0) {
                conf >> buffer_size;
                std::cerr << "BUFFER_SIZE\t\t\t\t" << buffer_size << '\n';
            } else if (key.compare("QLEN_MON_FILE") == 0) {
                conf >> qlen_mon_file;
                std::cerr << "QLEN_MON_FILE\t\t\t\t" << qlen_mon_file << '\n';
            } else if (key.compare("VOQ_MON_FILE") == 0) {
                conf >> voq_mon_file;
                std::cerr << "VOQ_MON_FILE\t\t\t\t" << voq_mon_file << '\n';
            } else if (key.compare("VOQ_MON_DETAIL_FILE") == 0) {
                conf >> voq_mon_detail_file;
                std::cerr << "VOQ_MON_DETAIL_FILE\t\t\t\t" << voq_mon_detail_file << '\n';
            } else if (key.compare("UPLINK_MON_FILE") == 0) {
                conf >> uplink_mon_file;
                std::cerr << "UPLINK_MON_FILE\t\t\t\t" << uplink_mon_file << '\n';
            } else if (key.compare("CONN_MON_FILE") == 0) {
                conf >> conn_mon_file;
                std::cerr << "CONN_MON_FILE\t\t\t\t" << conn_mon_file << '\n';
            } else if (key.compare("QLEN_MON_START") == 0) {
                conf >> qlen_mon_start;
                std::cerr << "QLEN_MON_START\t\t\t\t" << qlen_mon_start << '\n';
            } else if (key.compare("QLEN_MON_END") == 0) {
                conf >> qlen_mon_end;
                std::cerr << "QLEN_MON_END\t\t\t\t" << qlen_mon_end << '\n';
            } else if (key.compare("MULTI_RATE") == 0) {
                int v;
                conf >> v;
                multi_rate = v;
                std::cerr << "MULTI_RATE\t\t\t\t" << multi_rate << '\n';
            } else if (key.compare("SAMPLE_FEEDBACK") == 0) {
                int v;
                conf >> v;
                sample_feedback = v;
                std::cerr << "SAMPLE_FEEDBACK\t\t\t\t" << sample_feedback << '\n';
            } else if (key.compare("HPCC_TRACE") == 0) {
                int v;
                conf >> v;
                hpcc_trace = v;
                std::cerr << "HPCC_TRACE\t\t\t\t" << hpcc_trace << '\n';
            } else if (key.compare("INT_TRACE") == 0) {
                int v;
                conf >> v;
                int_trace = v;
                std::cerr << "INT_TRACE\t\t\t\t" << int_trace << '\n';
            } else if (key.compare("HPCC_TRACE_INTERVAL") == 0) {
                conf >> hpcc_trace_interval;
                if (hpcc_trace_interval == 0) hpcc_trace_interval = 1;
                std::cerr << "HPCC_TRACE_INTERVAL\t\t\t" << hpcc_trace_interval << '\n';
            } else if (key.compare("LOAD") == 0) {
                double v;
                conf >> v;
                load = v;
                std::cerr << "LOAD\t\t\t" << load << "\n";
            } else if (key.compare("ENABLE_IRN") == 0) {
                bool v;
                conf >> v;
                enable_irn = v;
                std::cerr << "ENABLE_IRN\t\t" << enable_irn << "\n";
            } else if (key.compare("PSN_PATH_K") == 0) {
                conf >> psn_path_k;
                std::cerr << "PSN_PATH_K\t\t" << psn_path_k << "\n";
            } else if (key.compare("PSN_PATH_BETA") == 0) {
                conf >> psn_path_beta;
                std::cerr << "PSN_PATH_BETA\t\t" << psn_path_beta << "\n";
            } else if (key.compare("PSN_PATH_GAP_TIMEOUT_NS") == 0) {
                conf >> psn_path_gap_timeout_ns;
                std::cerr << "PSN_PATH_GAP_TIMEOUT_NS\t" << psn_path_gap_timeout_ns << "\n";
            } else if (key.compare("ENABLE_PSN_PATH") == 0) {
                bool v;
                conf >> v;
                enable_psn_path = v;
                std::cerr << "ENABLE_PSN_PATH\t\t" << (enable_psn_path ? "Yes" : "No") << "\n";
            } else if (key.compare("ENABLE_PATH_SELECT") == 0) {
                bool v;
                conf >> v;
                enable_path_select = v;
                std::cerr << "ENABLE_PATH_SELECT\t" << (enable_path_select ? "Yes" : "No")
                          << "\n";
            } else if (key.compare("ENABLE_PATH_SWITCH") == 0) {
                bool v;
                conf >> v;
                enable_path_switch = v;
                std::cerr << "ENABLE_PATH_SWITCH\t" << (enable_path_switch ? "Yes" : "No")
                          << "\n";
            } else if (key.compare("ENABLE_PATH_AWARE_RETRANS") == 0) {
                bool v;
                conf >> v;
                enable_path_aware_retrans = v;
                std::cerr << "ENABLE_PATH_AWARE_RETRANS\t"
                          << (enable_path_aware_retrans ? "Yes" : "No") << "\n";
            } else if (key.compare("ENABLE_RX_OOO_NACK") == 0) {
                bool v;
                conf >> v;
                enable_rx_ooo_nack = v;
                std::cerr << "ENABLE_RX_OOO_NACK\t" << (enable_rx_ooo_nack ? "Yes" : "No")
                          << "\n";
            } else if (key.compare("ENABLE_TX_NACK_GOBACK") == 0) {
                bool v;
                conf >> v;
                enable_tx_nack_goback = v;
                std::cerr << "ENABLE_TX_NACK_GOBACK\t"
                          << (enable_tx_nack_goback ? "Yes" : "No") << "\n";
            } else if (key.compare("TX_NACK_RETRANS_INTERVAL_NS") == 0) {
                conf >> tx_nack_retrans_interval_ns;
                std::cerr << "TX_NACK_RETRANS_INTERVAL_NS\t"
                          << tx_nack_retrans_interval_ns << "\n";
            } else if (key.compare("ENABLE_BITMAP_RETRANS") == 0) {
                bool v;
                conf >> v;
                enable_bitmap_retrans = v;
                std::cerr << "ENABLE_BITMAP_RETRANS\t"
                          << (enable_bitmap_retrans ? "Yes" : "No") << "\n";
            } else if (key.compare("BITMAP_RETRANS_SIZE") == 0) {
                uint32_t v;
                conf >> v;
                bitmap_retrans_size = std::max<uint32_t>(1, v);
                std::cerr << "BITMAP_RETRANS_SIZE\t" << bitmap_retrans_size << "\n";
            } else if (key.compare("BITMAP_RETRANS_TIMEOUT_NS") == 0) {
                conf >> bitmap_retrans_timeout_ns;
                std::cerr << "BITMAP_RETRANS_TIMEOUT_NS\t" << bitmap_retrans_timeout_ns << "\n";
            } else if (key.compare("ENABLE_FALCON") == 0) {
                bool v;
                conf >> v;
                enable_falcon = v;
                std::cerr << "ENABLE_FALCON\t\t" << (enable_falcon ? "Yes" : "No") << "\n";
            } else if (key.compare("FALCON_RX_SEND_DELAY_NS") == 0) {
                conf >> falcon_rx_send_delay_ns;
                std::cerr << "FALCON_RX_SEND_DELAY_NS\t" << falcon_rx_send_delay_ns << "\n";
            } else if (key.compare("FALCON_RETRANS_RTT_K") == 0) {
                conf >> falcon_retrans_rtt_k;
                falcon_retrans_rtt_k = std::max(0.0, falcon_retrans_rtt_k);
                std::cerr << "FALCON_RETRANS_RTT_K\t" << falcon_retrans_rtt_k << "\n";
            } else if (key.compare("ENABLE_ORNIC") == 0) {
                bool v;
                conf >> v;
                enable_ornic = v;
                std::cerr << "ENABLE_ORNIC\t\t" << (enable_ornic ? "Yes" : "No") << "\n";
            } else if (key.compare("ORNIC_BW_GBPS") == 0) {
                double v;
                conf >> v;
                ornic_bw_gbps = v;
                std::cerr << "ORNIC_BW_GBPS\t\t" << ornic_bw_gbps << "\n";
            } else if (key.compare("ORNIC_RX_SEND_DELAY_NS") == 0) {
                conf >> ornic_rx_send_delay_ns;
                std::cerr << "ORNIC_RX_SEND_DELAY_NS\t" << ornic_rx_send_delay_ns << "\n";
            } else if (key.compare("RANDOM_SEED") == 0) {
                int v;
                conf >> v;
                random_seed = v;
                std::cerr << "RANDOM_SEED\t\t\t" << random_seed << "\n";
            } else if (key.compare("ENABLE_CONSOLE_LOG") == 0) {
                uint32_t v;
                conf >> v;
                enable_console_log = v > 2 ? 2 : v;
                Settings::enable_console_log = enable_console_log;
                std::cerr << "ENABLE_CONSOLE_LOG\t\t" << enable_console_log << "\n";
            } else if (key.compare("CONSOLE_LOG_PSNS") == 0) {
                std::getline(conf, console_log_psns);
                size_t first = console_log_psns.find_first_not_of(" \t");
                console_log_psns = first == std::string::npos ? "" : console_log_psns.substr(first);
                Settings::console_log_psns = console_log_psns;
                std::cerr << "CONSOLE_LOG_PSNS\t\t" << console_log_psns << "\n";
            }

            fflush(stdout);
        }
        conf.close();

    } else {
        std::cerr << "Error: require a config file\n";
        fflush(stdout);
        return 1;
    }

    /******************* READING CONFIG FILE IS DONE ***********************/
    ParseBurstLossSchedule();
    if (burst_stat_end_ns != 0 && burst_stat_end_ns <= burst_stat_start_ns) {
        NS_FATAL_ERROR("BURST_STAT_END_NS must be greater than BURST_STAT_START_NS, or 0 to disable the end bound");
    }
    ProgressLog("[PROGRESS] Config loaded: topo=%s flow=%s cc_mode=%u lb_mode=%u\n",
                topology_file.c_str(), flow_file.c_str(), cc_mode, lb_mode);

    /**
     * Activate ns3 logging
     */
    LogComponentEnable("GENERIC_SIMULATION", LOG_LEVEL_DEBUG);

    /**
     * @brief Random seed setup
     */
    NS_LOG_INFO("Initialize random seed: " << random_seed);
    srand((unsigned)random_seed);
    SeedManager::SetSeed(random_seed);

    /**
     * @brief PFC/QCN setup
     */
    bool dynamicth = use_dynamic_pfc_threshold;
    Config::SetDefault("ns3::QbbNetDevice::PauseTime", UintegerValue(pause_time));
    Config::SetDefault("ns3::QbbNetDevice::QcnEnabled", BooleanValue(enable_qcn));
    Config::SetDefault("ns3::QbbNetDevice::DynamicThreshold", BooleanValue(dynamicth));
    Config::SetDefault("ns3::QbbNetDevice::QbbEnabled", BooleanValue(enable_pfc));

    if (cc_mode != 1 && cc_mode != 3 && lb_mode == 9) {
        std::cout
            << "Currently, ConWeave supports only DCQCN(1) and HPCC(3) congestion control for RDMA."
            << std::endl;
        exit(1);
    }

    /**
     * @brief INT header setup
     */
    IntHop::multi = int_multi;
    // IntHeader::mode
    if (cc_mode == 7)  // timely, use ts
        IntHeader::mode = 1;
    else if (cc_mode == 3)  // hpcc, use int
        IntHeader::mode = 0;
    else  // others, no extra header
        IntHeader::mode = 5;

    /**
     * @brief open topology config, input-flows config.
     */
    topof.open(topology_file.c_str());
    flowf.open(flow_file.c_str());
    uint32_t node_num, switch_num, link_num;
    topof >> node_num >> switch_num >> link_num;
    flowf >> flow_num;
    ProgressLog("[PROGRESS] Input opened: topology nodes=%u switches=%u links=%u, flows=%u\n",
                node_num, switch_num, link_num, flow_num);

    /*-------Parameter of Settings-------*/
    Settings::node_num = node_num;
    Settings::host_num = node_num - switch_num;
    Settings::switch_num = switch_num;
    Settings::lb_mode = lb_mode;
    Settings::packet_payload = packet_payload_size;
    // Settings::MTU = packet_payload_size + 48;  // for simplicity
    /*------------------------------------*/

    std::vector<uint32_t> node_type(node_num, 0);
    for (uint32_t i = 0; i < switch_num; i++) {
        uint32_t sid;
        topof >> sid;
        node_type[sid] = 1;
    }
    for (uint32_t i = 0; i < node_num; i++) {
        if (node_type[i] == 0)
            n.Add(CreateObject<Node>());
        else {
            Ptr<SwitchNode> sw = CreateObject<SwitchNode>();
            n.Add(sw);
            sw->SetAttribute("EcnEnabled", BooleanValue(enable_qcn));
        }
    }
    NS_LOG_INFO("Create nodes.");

    /*----------------------------------------*/

    InternetStackHelper internet;
    internet.Install(n);  // aggregate ipv4, ipv6, udp, tcp, etc

    //
    // Assign IP to each server
    //
    for (uint32_t i = 0; i < node_num; i++) {
        if (n.Get(i)->GetNodeType() == 0) {  // is server
            serverAddress.resize(i + 1);
            serverAddress[i] = Settings::node_id_to_ip(i);
        }
    }

    NS_LOG_INFO("Create channels.");

    //
    // Explicitly create the channels required by the topology.
    //
    Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    rem->SetRandomVariable(uv);
    uv->SetStream(50);
    rem->SetAttribute("ErrorRate", DoubleValue(error_rate_per_link));
    rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
    RegisterBurstLossErrorModel(rem);

    pfc_file = fopen(pfc_output_file.c_str(), "w");

    QbbHelper qbb;
    Ipv4AddressHelper ipv4;
    std::vector<std::pair<uint32_t, uint32_t>> link_pairs;  // src, dst link pairs
    for (uint32_t i = 0; i < link_num; i++) {
        uint32_t src, dst;
        std::string data_rate, link_delay;
        double error_rate;
        topof >> src >> dst >> data_rate >> link_delay >> error_rate;

        /** ASSUME: fixed one-hop delay across network */
        assert(std::to_string(one_hop_delay) + "ns" == link_delay);

        link_pairs.push_back(std::make_pair(src, dst));
        Ptr<Node> snode = n.Get(src), dnode = n.Get(dst);

        qbb.SetDeviceAttribute("DataRate", StringValue(data_rate));
        qbb.SetChannelAttribute("Delay", StringValue(link_delay));

        if (error_rate > 0) {
            Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
            Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
            rem->SetRandomVariable(uv);
            uv->SetStream(50);
            rem->SetAttribute("ErrorRate", DoubleValue(error_rate));
            rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
            RegisterBurstLossErrorModel(rem);
            qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
        } else {
            qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
        }

        fflush(stdout);

        // Assigne server IP
        // Note: this should be before the automatic assignment below (ipv4.Assign(d)),
        // because we want our IP to be the primary IP (first in the IP address list),
        // so that the global routing is based on our IP
        NetDeviceContainer d = qbb.Install(snode, dnode);
        if (snode->GetNodeType() == 0) {
            Ptr<Ipv4> ipv4 = snode->GetObject<Ipv4>();
            ipv4->AddInterface(d.Get(0));
            ipv4->AddAddress(1, Ipv4InterfaceAddress(serverAddress[src], Ipv4Mask(0xff000000)));
        }
        if (dnode->GetNodeType() == 0) {
            Ptr<Ipv4> ipv4 = dnode->GetObject<Ipv4>();
            ipv4->AddInterface(d.Get(1));
            ipv4->AddAddress(1, Ipv4InterfaceAddress(serverAddress[dst], Ipv4Mask(0xff000000)));
        }

        // used to create a graph of the topology
        nbr2if[snode][dnode].idx = DynamicCast<QbbNetDevice>(d.Get(0))->GetIfIndex();
        nbr2if[snode][dnode].up = true;
        nbr2if[snode][dnode].delay =
            DynamicCast<QbbChannel>(DynamicCast<QbbNetDevice>(d.Get(0))->GetChannel())
                ->GetDelay()
                .GetTimeStep();
        nbr2if[snode][dnode].bw = DynamicCast<QbbNetDevice>(d.Get(0))->GetDataRate().GetBitRate();
        nbr2if[dnode][snode].idx = DynamicCast<QbbNetDevice>(d.Get(1))->GetIfIndex();
        nbr2if[dnode][snode].up = true;
        nbr2if[dnode][snode].delay =
            DynamicCast<QbbChannel>(DynamicCast<QbbNetDevice>(d.Get(1))->GetChannel())
                ->GetDelay()
                .GetTimeStep();
        nbr2if[dnode][snode].bw = DynamicCast<QbbNetDevice>(d.Get(1))->GetDataRate().GetBitRate();

        // This is just to set up the connectivity between nodes. The IP addresses are useless
        char ipstring[16];
        Ipv4Address x;
        uint32_t subnetId = i;
        uint32_t octet2 = 12 + subnetId / (256 * 256);
        uint32_t octet3 = (subnetId / 256) % 256;
        uint32_t octet4 = subnetId % 256;
        NS_ASSERT_MSG(octet2 <= 223, "Temporary link subnet space exhausted");
        snprintf(ipstring, sizeof(ipstring), "%u.%u.%u.0", octet2, octet3, octet4);
        ipv4.SetBase(ipstring, "255.255.255.0");
        ipv4.Assign(d);

        // setup PFC trace
        DynamicCast<QbbNetDevice>(d.Get(0))->TraceConnectWithoutContext(
            "QbbPfc", MakeBoundCallback(&get_pfc, pfc_file, DynamicCast<QbbNetDevice>(d.Get(0))));
        DynamicCast<QbbNetDevice>(d.Get(1))->TraceConnectWithoutContext(
            "QbbPfc", MakeBoundCallback(&get_pfc, pfc_file, DynamicCast<QbbNetDevice>(d.Get(1))));
    }
    ProgressLog("[PROGRESS] Topology parsed and channels created.\n");

    std::cout << "(AVG) NIC RATE: " << get_nic_rate(n) << std::endl;
    ProgressLog("[PROGRESS] Host NIC rate summary computed.\n");

    /* Get IP address <-> NodeID pairs */
    Ipv4Address empty_ip;
    for (uint32_t i = 0; i < node_num; ++i) {
        if (n.Get(i)->GetNodeType() == 0) {  // is server
            if (serverAddress[i].IsEqual(empty_ip)) {
                printf("XXX ERROR %d\n", i);
                printf("size of serverAddress: %lu", serverAddress.size());
                NS_FATAL_ERROR("An end-host belongs to no link");
            }
        }
        Settings::hostId2IpMap[i] = serverAddress[i].Get();
        Settings::hostIp2IdMap[serverAddress[i].Get()] = i;
    }
    ProgressLog("[PROGRESS] Host IP mapping initialized.\n");

    // config switch
    for (uint32_t i = 0; i < node_num; i++) {
        if (n.Get(i)->GetNodeType() == 1) {  // is switch
            Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n.Get(i));
            uint32_t shift = 3;  // by default 1/8
            for (uint32_t j = 1; j < sw->GetNDevices(); j++) {
                Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(sw->GetDevice(j));
                if (dev == 0) {
                    ProgressLog("[PROGRESS] Switch %u device[%u] is not a QbbNetDevice. Skip switch config.\n",
                                i, j);
                    continue;
                }
                // set ecn
                uint64_t rate = dev->GetDataRate().GetBitRate();
                NS_ASSERT_MSG(rate2kmin.find(rate) != rate2kmin.end(),
                              "must set kmin for each link speed");
                NS_ASSERT_MSG(rate2kmax.find(rate) != rate2kmax.end(),
                              "must set kmax for each link speed");
                NS_ASSERT_MSG(rate2pmax.find(rate) != rate2pmax.end(),
                              "must set pmax for each link speed");
                assert(rate2kmin.find(rate) != rate2kmin.end() &&
                       rate2kmax.find(rate) != rate2kmax.end() &&
                       rate2pmax.find(rate) != rate2pmax.end());
                sw->m_mmu->ConfigEcn(j, rate2kmin[rate], rate2kmax[rate], rate2pmax[rate]);
                if (cc_mode == 1 && enable_qcn) {
                    std::cerr << "[DCQCN_ECN_CONFIG] switch=" << i << " port=" << j
                              << " rate_bps=" << rate
                              << " kmin_bytes=" << rate2kmin[rate] * 1000
                              << " kmax_bytes=" << rate2kmax[rate] * 1000
                              << " pmax=" << rate2pmax[rate] << "\n";
                }
                // set pfc
                uint64_t delay =
                    DynamicCast<QbbChannel>(dev->GetChannel())->GetDelay().GetTimeStep();
                uint32_t headroom = rate * delay / 8 / 1000000000 * 2 + 2 * sw->m_mmu->MTU;
                sw->m_mmu->ConfigHdrm(j, headroom);
            }
            sw->m_mmu->ConfigNPort(sw->GetNDevices() - 1);
            sw->m_mmu->ConfigBufferSize(buffer_size * 1024 *
                                        1024);  // default 0, specify in run.py!!
            sw->m_mmu->node_id = sw->GetId();
            NS_LOG_INFO("Node " << i << " : Broadcom switch (" << (sw->GetNDevices() - 1)
                                << " ports / " << (sw->m_mmu->GetMmuBufferBytes() / 1000000.)
                                << "MB MMU)");
        }
    }
    ProgressLog("[PROGRESS] Switch ECN/PFC configuration completed.\n");

    fct_output = fopen(fct_output_file.c_str(), "w");
    flow_input_stream = fopen(flow_input_file.c_str(), "w");
    if (cc_mode == 1) {
        cnp_output = fopen(cnp_output_file.c_str(), "w");
    }

    /**
     * @brief install RDMA driver (Mellanox parameters)
     *
     * [ClampTargetRate]clamp_tgt_rate (false) - when receiving a CNP, the target rate is always
     *updated to be the current rate
     *[-]clamp_tgt_rate_after_time_inc (true) - when receiving a CNP, the target rate is updated to
     *be the current rate also if the last rate increase event was due to the timer, and not only
     *due to the byte counter
     * [-]initial_alpha_value(1023) -
     * [RateDecreaseInterval]rate_reduce_monitor_period(4) - Minimal interval for rate reduction for
     *a flow. If a CNP is received during the interval, the flow rate is reduced at the beginning of
     *the next rate_reduce_monitor_period interval to (1-Alpha/Gd)*CurrentRate. rpg_gd is given as
     *log2(Gd), where Gd may only be powers of 2.
     * [-]rpg_gd(11) - If an CNP is received, the flow rate is reduced at the beginning of the next
     *rate_reduce_monitor_period interval to (1-Alpha/Gd)*CurrentRate.
     * -> in this simulator, (alpha / gd) ~ 0.5 setup, initially. We do not need rpg_gd parameter.
     * [RateOnFirstCnp]rate_to_set_on_first_cnp(0) - The rate that is set for the flow, upon first
     *CNP received, in Mbps. [RPTimer]rpg_time_reset(300us) - Time counter for rate increase event
     *[FastRecoveryTimes]rpg_threshold(1) - Number of rate increase events for switching between
     *Fast Recovery, Active Increase, Hyper Active Increase modes.
     * [AlphaResumInterval]dce_tcp_rtt(1) - Window for sampling of moving average calculation of
     *alpha
     * [-]dce_tcp_g(1019) - Weight of the new sampling in moving average calculation of alpha
     * [-]rpg_byte_reset(32767) - Byte counter for rate increase event
     * [-]rpg_min_dec_fac(50) -  Maximal factor by which the rate can be reduced (2 means that the
     *new rate can be divided by 2 at maximum)
     */

    // manually type BDP
    std::map<std::string, uint32_t> topo2bdpMap;
    topo2bdpMap[std::string("leaf_spine_128_100G_OS2")] = 104000;  // RTT=8320
    topo2bdpMap[std::string("fat_k8_100G_OS2")] = 156000;      // RTT=12480 --> all 100G links

    // topology_file
    bool found_topo2bdpMap = false;
    uint32_t irn_bdp_lookup = 0;
    for (auto pair : topo2bdpMap) {
        if (topology_file.find(pair.first) !=
            std::string::npos) {  // if topology file string includes the word
            irn_bdp_lookup = pair.second;
            found_topo2bdpMap = true;
            break;
        }
    }
    if (found_topo2bdpMap == false) {
        // Allow custom topologies without enforcing a hardcoded IRN BDP mapping.
        irn_bdp_lookup = 0;
        std::cout << __FILE__ << "(" << __LINE__ << ")"
                  << " WARN - topo2bdpMap has no matched item with " << topology_file
                  << ", disable IRN BDP strict check and continue with IrnBdp=0" << std::endl;
    }

    // rdmaHw config
    for (uint32_t i = 0; i < node_num; i++) {
        if (n.Get(i)->GetNodeType() == 0) {  // is server
            if (i % 256 == 0) {
                ProgressLog("[PROGRESS] Initializing RDMA host node=%u\n", i);
            }
            // create RdmaHw
            Ptr<RdmaHw> rdmaHw = CreateObject<RdmaHw>();
            rdmaHw->SetAttribute("ClampTargetRate", BooleanValue(clamp_target_rate));
            rdmaHw->SetAttribute("AlphaResumInterval", DoubleValue(alpha_resume_interval));
            rdmaHw->SetAttribute("RPTimer", DoubleValue(rp_timer));
            rdmaHw->SetAttribute("FastRecoveryTimes", UintegerValue(fast_recovery_times));
            rdmaHw->SetAttribute("EwmaGain", DoubleValue(ewma_gain));
            rdmaHw->SetAttribute("RateAI", DataRateValue(DataRate(rate_ai)));
            rdmaHw->SetAttribute("RateHAI", DataRateValue(DataRate(rate_hai)));
            rdmaHw->SetAttribute("L2BackToZero", BooleanValue(l2_back_to_zero));
            rdmaHw->SetAttribute("L2ChunkSize", UintegerValue(l2_chunk_size));
            rdmaHw->SetAttribute("L2AckInterval", UintegerValue(l2_ack_interval));
            rdmaHw->SetAttribute("CcMode", UintegerValue(cc_mode));
            rdmaHw->SetAttribute("RateDecreaseInterval", DoubleValue(rate_decrease_interval));
            rdmaHw->SetAttribute("MinRate", DataRateValue(DataRate(min_rate)));
            rdmaHw->SetAttribute("Mtu", UintegerValue(packet_payload_size));
            rdmaHw->SetAttribute("MiThresh", UintegerValue(mi_thresh));
            rdmaHw->SetAttribute("VarWin", BooleanValue(var_win));
            rdmaHw->SetAttribute("FastReact", BooleanValue(fast_react));
            rdmaHw->SetAttribute("MultiRate", BooleanValue(multi_rate));
            rdmaHw->SetAttribute("SampleFeedback", BooleanValue(sample_feedback));
            rdmaHw->SetAttribute("HpccTrace", BooleanValue(hpcc_trace));
            rdmaHw->SetAttribute("HpccTraceInterval", UintegerValue(hpcc_trace_interval));
            rdmaHw->SetAttribute("EnableConsoleLog", UintegerValue(enable_console_log));
            rdmaHw->SetAttribute("ConsoleLogPsns", StringValue(console_log_psns));
            rdmaHw->SetAttribute("TargetUtil", DoubleValue(u_target));
            rdmaHw->SetAttribute("RateBound", BooleanValue(rate_bound));
            rdmaHw->SetAttribute("DctcpRateAI", DataRateValue(DataRate(dctcp_rate_ai)));
            rdmaHw->SetAttribute("IrnEnable", BooleanValue(enable_irn));
            rdmaHw->SetAttribute("EnablePsnPath", BooleanValue(enable_psn_path));
            rdmaHw->SetAttribute("EnablePathSelection", BooleanValue(enable_path_select));
            rdmaHw->SetAttribute("EnablePathSwitch", BooleanValue(enable_path_switch));
            rdmaHw->SetAttribute("EnablePathAwareRetrans",
                                 BooleanValue(enable_path_aware_retrans));
            rdmaHw->SetAttribute("EnableRxOooNack", BooleanValue(enable_rx_ooo_nack));
            rdmaHw->SetAttribute("EnableTxNackGoBack", BooleanValue(enable_tx_nack_goback));
            rdmaHw->SetAttribute("TxNackRetransInterval",
                                 TimeValue(NanoSeconds(tx_nack_retrans_interval_ns)));
            rdmaHw->SetAttribute("EnableBitmapRetrans", BooleanValue(enable_bitmap_retrans));
            rdmaHw->SetAttribute("BitmapRetransSize", UintegerValue(bitmap_retrans_size));
            rdmaHw->SetAttribute("BitmapRetransTimeout",
                                 TimeValue(NanoSeconds(bitmap_retrans_timeout_ns)));
            rdmaHw->SetAttribute("EnableFalcon", BooleanValue(enable_falcon));
            rdmaHw->SetAttribute("FalconRxSendDelay",
                                 TimeValue(NanoSeconds(falcon_rx_send_delay_ns)));
            rdmaHw->SetAttribute("FalconRetransRttK", DoubleValue(falcon_retrans_rtt_k));
            rdmaHw->SetAttribute("EnableOrnic", BooleanValue(enable_ornic));
            rdmaHw->SetAttribute("OrnicBandwidthGbps", DoubleValue(ornic_bw_gbps));
            rdmaHw->SetAttribute("OrnicRxSendDelay",
                                 TimeValue(NanoSeconds(ornic_rx_send_delay_ns)));
            rdmaHw->SetAttribute("PsnPathK", UintegerValue(psn_path_k));
            rdmaHw->SetAttribute("PsnPathBeta", DoubleValue(psn_path_beta));
            rdmaHw->SetAttribute("PsnPathGapTimeout",
                                 TimeValue(NanoSeconds(psn_path_gap_timeout_ns)));
            // topo2bdpMap (e.g., longest BDP 25000: 8us * 25Gbps)
            rdmaHw->SetAttribute("IrnRtoHigh", TimeValue(MicroSeconds(320)));  // 1930
            rdmaHw->SetAttribute("IrnRtoLow", TimeValue(MicroSeconds(100)));   // 454
            rdmaHw->SetAttribute("IrnBdp", UintegerValue(irn_bdp_lookup));
            // Monitoring CNP Marking frequency of DCQCN
            if (cc_mode == 1) {
                Simulator::Schedule(NanoSeconds(cnp_mon_start), &cnp_freq_monitoring, cnp_output,
                                    rdmaHw);
            }

            // create and install RdmaDriver
            Ptr<RdmaDriver> rdma = CreateObject<RdmaDriver>();
            Ptr<Node> node = n.Get(i);
            if (node == 0) {
                NS_FATAL_ERROR("Host node is null during RDMA initialization");
            }
            rdma->SetNode(node);
            rdma->SetRdmaHw(rdmaHw);

            node->AggregateObject(rdma);
            if (i % 256 == 0) {
                ProgressLog("[PROGRESS] Host %u entering RdmaDriver::Init\n", i);
            }
            rdma->Init();
            if (i % 256 == 0) {
                ProgressLog("[PROGRESS] Host %u finished RdmaDriver::Init\n", i);
            }
            rdma->TraceConnectWithoutContext("QpComplete",
                                             MakeBoundCallback(qp_finish, fct_output));
            if (i % 256 == 0) {
                ProgressLog("[PROGRESS] Host %u connected QpComplete trace\n", i);
            }
        }
    }
    ProgressLog("[PROGRESS] RDMA drivers initialized on all hosts.\n");

    /**
     * @brief setup switch's CcMode and ACK with high priority
     */
    for (uint32_t i = 0; i < node_num; i++) {
        if (n.Get(i)->GetNodeType() == 1) {  // switch
            Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(n.Get(i));
            sw->SetAttribute("CcMode", UintegerValue(cc_mode));
            sw->SetAttribute("AckHighPrio", UintegerValue(1));
            sw->SetAttribute("IntTrace", BooleanValue(int_trace));
        }
    }

    /**
     * @brief setup routing
     */
    CalculateRoutes(n);
    SetRoutingEntries();

    /**
     * @brief get BDP and delay
     */
    maxRtt = maxBdp = 0;
    fprintf(stderr, "node_num=%d\n", node_num);
    for (uint32_t i = 0; i < node_num; i++) {
        if (n.Get(i)->GetNodeType() != 0) continue;
        for (uint32_t j = i + 1; j < node_num; j++) {
            if (n.Get(j)->GetNodeType() != 0) continue;
            uint64_t delay = pairDelay[n.Get(i)][n.Get(j)];
            uint64_t txDelay = pairTxDelay[n.Get(i)][n.Get(j)];
            uint64_t rtt = delay * 2 + txDelay;
            uint64_t bw = pairBw[n.Get(i)][n.Get(j)];
            uint64_t bdp = rtt * bw / 1000000000 / 8;
            pairBdp[n.Get(i)][n.Get(j)] = bdp;
            pairBdp[n.Get(j)][n.Get(i)] = bdp;
            pairRtt[n.Get(i)][n.Get(j)] = rtt;
            pairRtt[n.Get(j)][n.Get(i)] = rtt;

            if (bdp > maxBdp) maxBdp = bdp;
            if (rtt > maxRtt) maxRtt = rtt;
        }
    }
    fprintf(stderr, "maxRtt: %lu, maxBdp: %lu\n", maxRtt, maxBdp);
    // if (enable_irn) {
    //     assert(maxBdp == irn_bdp_lookup);
    // }

    std::cout << "Configuring switches" << std::endl;
    /* config ToR Switch */
    for (auto &pair : link_pairs) {
        Ptr<Node> probably_host = n.Get(pair.first);
        Ptr<Node> probably_switch = n.Get(pair.second);

        // host-switch link
        if (probably_host->GetNodeType() == 0 && probably_switch->GetNodeType() == 1) {
            Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(probably_switch);
            sw->m_isToR = true;
            uint32_t hostIP = serverAddress[pair.first].Get();
            sw->m_isToR_hostIP.insert(hostIP);
            if (idxNodeToR.find(sw->GetId()) == idxNodeToR.end()) {
                idxNodeToR[sw->GetId()] = sw;
            };
        }
    }

    /* config load balancer's switches using ToR-to-ToR routing */
    if (lb_mode == 3 || lb_mode == 6 || lb_mode == 9) {  // Conga, Letflow, Conweave
        NS_LOG_INFO("Configuring Load Balancer's Switches");
        for (auto &pair : link_pairs) {
            Ptr<Node> probably_host = n.Get(pair.first);
            Ptr<Node> probably_switch = n.Get(pair.second);

            // host-switch link
            if (probably_host->GetNodeType() == 0 && probably_switch->GetNodeType() == 1) {
                Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(probably_switch);
                uint32_t hostIP = serverAddress[pair.first].Get();
                Settings::hostIp2SwitchId[hostIP] = sw->GetId();  // hostIP -> connected switch's ID
            }
        }

        // Conga: m_congaFromLeafTable, m_congaToLeafTable, m_congaRoutingTable
        // Letflow: m_letflowRoutingTable
        // Conweave: m_ConWeaveRoutingTable, m_rxToRId2BaseRTT
        for (auto i = nextHop.begin(); i != nextHop.end(); i++) {  // every node
            if (i->first->GetNodeType() == 1) {                    // switch
                Ptr<Node> nodeSrc = i->first;
                Ptr<SwitchNode> swSrc = DynamicCast<SwitchNode>(nodeSrc);  // switch
                uint32_t swSrcId = swSrc->GetId();

                if (swSrc->m_isToR) {
                    // printf("--- ToR Switch %d\n", swSrcId);

                    auto table1 = i->second;
                    for (auto j = table1.begin(); j != table1.end(); j++) {
                        Ptr<Node> dst = j->first;  // dst
                        uint32_t dstIP = Settings::hostId2IpMap[dst->GetId()];
                        uint32_t swDstId = Settings::hostIp2SwitchId[dstIP];  // Rx(dst)ToR

                        if (swSrcId == swDstId) {
                            continue;  // if in the same pod, then skip
                        }

                        if (lb_mode == 3) {
                            // initialize `m_congaFromLeafTable` and `m_congaToLeafTable`
                            swSrc->m_mmu->m_congaRouting
                                .m_congaFromLeafTable[swDstId];  // dynamically will be added in
                                                                 // conga
                            swSrc->m_mmu->m_congaRouting.m_congaToLeafTable[swDstId];
                        }

                        // construct paths
                        uint32_t pathId;
                        uint8_t path_ports[4] = {0, 0, 0, 0};  // interface is always large than 0
                        vector<Ptr<Node>> nexts1 = j->second;
                        for (auto next1 : nexts1) {
                            uint32_t outPort1 = nbr2if[nodeSrc][next1].idx;
                            auto nexts2 = nextHop[next1][dst];
                            if (nexts2.size() == 1 && nexts2[0]->GetId() == swDstId) {
                                // this destination has 2-hop distance
                                uint32_t outPort2 = nbr2if[next1][nexts2[0]].idx;
                                // printf("[IntraPod-2hop] %d (%d)-> %d (%d) -> %d -> %d\n",
                                // nodeSrc->GetId(), outPort1, next1->GetId(), outPort2,
                                // nexts2[0]->GetId(), dst->GetId());
                                path_ports[0] = (uint8_t)outPort1;
                                path_ports[1] = (uint8_t)outPort2;
                                pathId = *((uint32_t *)path_ports);
                                if (lb_mode == 3) {
                                    swSrc->m_mmu->m_congaRouting.m_congaRoutingTable[swDstId]
                                        .insert(pathId);
                                }
                                if (lb_mode == 6) {
                                    swSrc->m_mmu->m_letflowRouting.m_letflowRoutingTable[swDstId]
                                        .insert(pathId);
                                }
                                if (lb_mode == 9) {
                                    swSrc->m_mmu->m_conweaveRouting.m_ConWeaveRoutingTable[swDstId]
                                        .insert(pathId);
                                    swSrc->m_mmu->m_conweaveRouting.m_rxToRId2BaseRTT[swDstId] =
                                        one_hop_delay * 4;
                                }
                                continue;
                            }

                            for (auto next2 : nexts2) {
                                uint32_t outPort2 = nbr2if[next1][next2].idx;
                                auto nexts3 = nextHop[next2][dst];
                                if (nexts3.size() == 1 && nexts3[0]->GetId() == swDstId) {
                                    // this destination has 3-hop distance
                                    uint32_t outPort3 = nbr2if[next2][nexts3[0]].idx;
                                    // printf("[IntraPod-3hop] %d (%d)-> %d (%d) -> %d (%d) -> %d ->
                                    // %d\n", nodeSrc->GetId(), outPort1, next1->GetId(), outPort2,
                                    // next2->GetId(), outPort3, nexts3[0]->GetId(), dst->GetId());
                                    path_ports[0] = (uint8_t)outPort1;
                                    path_ports[1] = (uint8_t)outPort2;
                                    path_ports[2] = (uint8_t)outPort3;
                                    pathId = *((uint32_t *)path_ports);
                                    if (lb_mode == 3) {
                                        swSrc->m_mmu->m_congaRouting.m_congaRoutingTable[swDstId]
                                            .insert(pathId);
                                    }
                                    if (lb_mode == 6) {
                                        swSrc->m_mmu->m_letflowRouting
                                            .m_letflowRoutingTable[swDstId]
                                            .insert(pathId);
                                    }
                                    if (lb_mode == 9) {
                                        swSrc->m_mmu->m_conweaveRouting
                                            .m_ConWeaveRoutingTable[swDstId]
                                            .insert(pathId);
                                        swSrc->m_mmu->m_conweaveRouting.m_rxToRId2BaseRTT[swDstId] =
                                            one_hop_delay * 6;
                                    }
                                    continue;
                                }

                                for (auto next3 : nexts3) {
                                    uint32_t outPort3 = nbr2if[next2][next3].idx;
                                    auto nexts4 = nextHop[next3][dst];
                                    if (nexts4.size() == 1 && nexts4[0]->GetId() == swDstId) {
                                        // this destination has 4-hop distance
                                        uint32_t outPort4 = nbr2if[next3][nexts4[0]].idx;
                                        // printf("[IntraPod-4hop] %d (%d)-> %d (%d) -> %d (%d) ->
                                        // %d (%d) -> %d -> %d\n", nodeSrc->GetId(), outPort1,
                                        // next1->GetId(), outPort2, next2->GetId(), outPort3,
                                        // next3->GetId(), outPort4, nexts4[0]->GetId(),
                                        // dst->GetId());
                                        path_ports[0] = (uint8_t)outPort1;
                                        path_ports[1] = (uint8_t)outPort2;
                                        path_ports[2] = (uint8_t)outPort3;
                                        path_ports[3] = (uint8_t)outPort4;
                                        pathId = *((uint32_t *)path_ports);
                                        if (lb_mode == 3) {
                                            swSrc->m_mmu->m_congaRouting
                                                .m_congaRoutingTable[swDstId]
                                                .insert(pathId);
                                        }
                                        if (lb_mode == 6) {
                                            swSrc->m_mmu->m_letflowRouting
                                                .m_letflowRoutingTable[swDstId]
                                                .insert(pathId);
                                        }
                                        if (lb_mode == 9) {
                                            swSrc->m_mmu->m_conweaveRouting
                                                .m_ConWeaveRoutingTable[swDstId]
                                                .insert(pathId);
                                            swSrc->m_mmu->m_conweaveRouting
                                                .m_rxToRId2BaseRTT[swDstId] = one_hop_delay * 8;
                                        }
                                        continue;
                                    } else {
                                        printf("Too large topology?\n");
                                        assert(false);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // m_outPort2BitRateMap - only for Conga
        for (auto i = nextHop.begin(); i != nextHop.end(); i++) {  // every node
            if (i->first->GetNodeType() == 1) {                    // switch
                Ptr<Node> node = i->first;
                Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(node);  // switch
                uint32_t swId = sw->GetId();

                auto table = i->second;
                for (auto j = table.begin(); j != table.end(); j++) {
                    Ptr<Node> dst = j->first;  // dst
                    uint32_t dstIP = Settings::hostId2IpMap[dst->GetId()];
                    uint32_t swDstId = Settings::hostIp2SwitchId[dstIP];

                    for (auto next : j->second) {
                        uint32_t outPort = nbr2if[node][next].idx;
                        uint64_t bw = nbr2if[node][next].bw;
                        sw->m_mmu->m_congaRouting.SetLinkCapacity(outPort, bw);
                        // printf("Node: %d, interface: %d, bw: %lu\n", swId, outPort, bw);
                    }
                }
            }
        }

        // Constant setup, and switchInfo
        for (auto i = nextHop.begin(); i != nextHop.end(); i++) {  // every node
            if (i->first->GetNodeType() == 1) {
                Ptr<Node> node = i->first;
                Ptr<SwitchNode> sw = DynamicCast<SwitchNode>(node);  // switch
                NS_LOG_INFO("Switch Info - ID:" << sw->GetId() << ", ToR:" << sw->m_isToR);
                if (lb_mode == 3) {
                    sw->m_mmu->m_congaRouting.SetConstants(conga_dreTime, conga_agingTime,
                                                           conga_flowletTimeout, conga_quantizeBit,
                                                           conga_alpha);
                    sw->m_mmu->m_congaRouting.SetSwitchInfo(sw->m_isToR, sw->GetId());
                }
                if (lb_mode == 6) {
                    sw->m_mmu->m_letflowRouting.SetConstants(letflow_agingTime,
                                                             letflow_flowletTimeout);
                    sw->m_mmu->m_letflowRouting.SetSwitchInfo(sw->m_isToR, sw->GetId());
                }
                if (lb_mode == 9) {
                    sw->m_mmu->m_conweaveRouting.SetConstants(
                        conweave_extraReplyDeadline, conweave_extraVOQFlushTime,
                        conweave_txExpiryTime, conweave_defaultVOQWaitingTime,
                        conweave_pathPauseTime, conweave_pathAwareRerouting);
                    sw->m_mmu->m_conweaveRouting.SetSwitchInfo(sw->m_isToR, sw->GetId());
                }
            }
        }

        // schedule conga timeout monitor
        if (lb_mode == 3) {  // CONGA
            Simulator::Schedule(Seconds(flowgen_stop_time + simulator_extra_time),
                                conga_history_print);
        }
        if (lb_mode == 6) {  // LETFLOW
            Simulator::Schedule(Seconds(flowgen_stop_time + simulator_extra_time),
                                letflow_history_print);
        }
        if (lb_mode == 9) {  // CONWEAVE
            Simulator::Schedule(Seconds(flowgen_stop_time + simulator_extra_time),
                                conweave_history_print);
        }
    }

    // populate routing tables (although we use our custom impl in switch_node.cc)
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // maintain port number for each host
    for (uint32_t i = 0; i < node_num; i++) {
        if (n.Get(i)->GetNodeType() == 0) {
            portNumber[i] = 10000;  // each host use port number from 10000
            dportNumber[i] = 100;
        }
    }

    flow_input.idx = 0;
    port_per_host = new uint16_t[node_num - switch_num];
    if (flow_num > 0) {
        // generate flows
        ReadFlowInput();
        Simulator::Schedule(Seconds(0), &ScheduleFlowInputs, flow_input_stream);
        ProgressLog("[PROGRESS] Flow scheduling initialized. total_flows=%u\n", flow_num);
    } else {
        ProgressLog("[PROGRESS] Flow file is empty. No flows scheduled.\n");
    }

    topof.close();

    // schedule link down
    if (link_down_time > 0) {
        Simulator::Schedule(Seconds(flowgen_start_time) + MicroSeconds(link_down_time),
                            &TakeDownLink, n, n.Get(link_down_A), n.Get(link_down_B));
    }

    if (lb_mode == 9) {
        voq_output = fopen(voq_mon_file.c_str(), "w");                // specific to ConWeave
        voq_detail_output = fopen(voq_mon_detail_file.c_str(), "w");  // specific to ConWeave
    }

    uplink_output = fopen(uplink_mon_file.c_str(), "w");  // common
    conn_output = fopen(conn_mon_file.c_str(), "w");      // common

    // update torId2UplinkIf, torId2DownlinkIf
    for (size_t ToRId = 0; ToRId < Settings::node_num; ToRId++) {
        Ptr<Node> node = n.Get(ToRId);
        if (node->GetNodeType() == 1) {  // switches
            auto swNode = DynamicCast<SwitchNode>(n.Get(ToRId));
            if (swNode->m_isToR) {  // TOR switch
                for (auto &nextNodeIf : nbr2if[node]) {
                    if (nextNodeIf.first->GetNodeType() ==
                        1) {  // nextNode is switch (i.e., uplink)
                        auto &vec = torId2UplinkIf[ToRId];
                        vec.push_back(
                            nextNodeIf.second.idx);  // record this uplink port (outDev index)
                        // printf("Sw %lu - uplink port %u\n", ToRId, nextNodeIf.second.idx);  //
                        // debugging
                    } else {
                        auto &vec = torId2DownlinkIf[ToRId];
                        vec.push_back(
                            nextNodeIf.second.idx);  // record this downlink port (outDev index)
                        // printf("Sw %lu - downlink port %u\n", ToRId, nextNodeIf.second.idx);  //
                        // debugging
                    }
                }
            }
        }
    }
    Simulator::Schedule(Seconds(flowgen_start_time), &periodic_monitoring, voq_output,
                        voq_detail_output, uplink_output, conn_output, &lb_mode);
    RdmaHw::SetTxDataPacketCallback(MakeCallback(&RecordBurstTxDataPacket));
    RdmaHw::SetRxDataPacketCallback(MakeCallback(&RecordBurstRxDataPacket));
    ScheduleBurstLossEvents();

    //
    // Now, do the actual simulation.
    //
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "Running Simulation.\n";
    ProgressLog("[PROGRESS] Simulator::Run starting. stop_time=%.3f s\n", flowgen_stop_time + 10.0);
    fflush(stdout);
    NS_LOG_INFO("Run Simulation.");
    Simulator::Schedule(Seconds(flowgen_start_time),
                        &stop_simulation_middle);  // check every 100us
    Simulator::Stop(Seconds(flowgen_stop_time + 10.0));
    Simulator::Run();
    WriteBurstStatOutputs();
    if (burst_loss_log_output != NULL) {
        fclose(burst_loss_log_output);
        burst_loss_log_output = NULL;
    }

    /*-----------------------------------------------------------------------------*/
    /*----- we don't need below. Just we can enforce to close this simulation. -----*/
    /*-----------------------------------------------------------------------------*/
    Simulator::Destroy();
    NS_LOG_INFO("Total number of packets: " << RdmaHw::nAllPkts);
    NS_LOG_INFO("Done.");
    endt = clock();
    std::cerr << (double)(endt - begint) / CLOCKS_PER_SEC << "\n";
    std::cout.flush();
    std::cerr.flush();
    fflush(NULL);
    _Exit(0);
}
