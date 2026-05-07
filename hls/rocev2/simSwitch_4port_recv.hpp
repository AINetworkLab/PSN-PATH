/*
 * Copyright (c) 2022, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include "../axi_utils.hpp"
using namespace hls;

#define RECV_N_NODE_4 4

// 简单 deterministic RNG：LCG（仿真稳定、可复现）
static inline double rng01(uint32_t &state) {
    state = state * 1664525u + 1013904223u;
    // 取高 24 bit 映射到 [0,1)
    return double((state >> 8) & 0x00FFFFFF) * (1.0 / 16777216.0);
}

template <int WIDTH>
void simSwitchRecv(
    // path0 RX->TX
    stream<ipUdpMeta>& m_axis_rx_meta_n0,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n0,
    stream<ipUdpMeta>& s_axis_tx_meta_n0,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n0,

    // feedback opcode: from receiver ibtp (n1) back to sender (n0)
    stream<testForSwitch>&   recvIbOpcodeFifo_n1,
    stream<testForSwitch>&   txIbOpcodeFifo_n0,

    // receiver side port1
    stream<ipUdpMeta>& m_axis_rx_meta_n1,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n1,
    stream<ipUdpMeta>& s_axis_tx_meta_n1,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n1,

    // other path ports
    stream<ipUdpMeta>& m_axis_rx_meta_n2,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n2,
    stream<ipUdpMeta>& s_axis_tx_meta_n2,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n2,

    stream<ipUdpMeta>& m_axis_rx_meta_n3,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n3,
    stream<ipUdpMeta>& s_axis_tx_meta_n3,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n3,

    stream<ipUdpMeta>& m_axis_rx_meta_n4,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n4,
    stream<ipUdpMeta>& s_axis_tx_meta_n4,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n4,

    ap_uint<128> ip_address_n0,
    ap_uint<128> ip_address_n1,
    ap_uint<128> ip_address_n2,
    ap_uint<128> ip_address_n3,
    ap_uint<128> ip_address_n4,

    double dropRate,       // 可选：汇聚处再丢一次（一般置 0）
    double reorderRate     // 乱序比率（0~1）
){
#pragma HLS inline off
#pragma HLS pipeline II=1

    // ------------------ per-port meta buffer (0/2/3/4 paths) ------------------
    static ipUdpMeta meta0, meta2, meta3, meta4;
    static bool      vld0=false, vld2=false, vld3=false, vld4=false;
    static bool      drop0=false, drop2=false, drop3=false, drop4=false;

    // ------------------ feedback (src=1 -> dst=0) FSM ------------------
    static bool fb_active=false;
    static bool fb_meta_sent=false;
    static ipUdpMeta fb_meta;
    static testForSwitch fb_op;

    // ------------------ aggregation to dst=1 FSM (paths 0/2/3/4) ------------------
    static ap_uint<3> active = 7;   // 7=none, else {0,2,3,4}
    static bool agg_meta_sent=false;
    static ap_uint<2> rr=0;         // round-robin pointer in {0,1,2,3} -> {0,2,3,4}

    static uint32_t rng_state = 1;

    auto is_dst = [&](const ipUdpMeta& m, ap_uint<128> ip)->bool {
        return (m.their_address == ip);
    };

    auto try_read_meta = [&](int p){
        // 只对 0/2/3/4 做多路径汇聚缓存
        if (p==0) {
            if (!s_axis_tx_meta_n0.empty() && !vld0) {
                s_axis_tx_meta_n0.read(meta0);
                vld0 = true;
                drop0 = (dropRate>0.0) ? (rng01(rng_state) < dropRate) : false;
            }
        } else if (p==2) {
            if (!s_axis_tx_meta_n2.empty() && !vld2) {
                s_axis_tx_meta_n2.read(meta2);
                vld2 = true;
                drop2 = (dropRate>0.0) ? (rng01(rng_state) < dropRate) : false;
            }
        } else if (p==3) {
            if (!s_axis_tx_meta_n3.empty() && !vld3) {
                s_axis_tx_meta_n3.read(meta3);
                vld3 = true;
                drop3 = (dropRate>0.0) ? (rng01(rng_state) < dropRate) : false;
            }
        } else { // p==4
            if (!s_axis_tx_meta_n4.empty() && !vld4) {
                s_axis_tx_meta_n4.read(meta4);
                vld4 = true;
                drop4 = (dropRate>0.0) ? (rng01(rng_state) < dropRate) : false;
            }
        }
    };

    auto data_empty = [&](int p)->bool {
        if (p==0) return s_axis_tx_data_n0.empty();
        if (p==2) return s_axis_tx_data_n2.empty();
        if (p==3) return s_axis_tx_data_n3.empty();
        return s_axis_tx_data_n4.empty();
    };

    auto read_data = [&](int p, net_axis<WIDTH>& w){
        if (p==0) s_axis_tx_data_n0.read(w);
        else if (p==2) s_axis_tx_data_n2.read(w);
        else if (p==3) s_axis_tx_data_n3.read(w);
        else s_axis_tx_data_n4.read(w);
    };

    auto meta_of = [&](int p)->ipUdpMeta& {
        if (p==0) return meta0;
        if (p==2) return meta2;
        if (p==3) return meta3;
        return meta4;
    };

    auto vld_of = [&](int p)->bool& {
        if (p==0) return vld0;
        if (p==2) return vld2;
        if (p==3) return vld3;
        return vld4;
    };

    auto drop_of = [&](int p)->bool& {
        if (p==0) return drop0;
        if (p==2) return drop2;
        if (p==3) return drop3;
        return drop4;
    };

    // ------------------ 1) feedback path: handle src=1 -> dst=0 with opcode aligned ------------------
    if (!fb_active) {
        // 等 meta+opcode 都就绪再启动（保证对齐）
        if (!s_axis_tx_meta_n1.empty() && !recvIbOpcodeFifo_n1.empty()) {
            s_axis_tx_meta_n1.read(fb_meta);
            recvIbOpcodeFifo_n1.read(fb_op);

            // 只对发往 n0 的反馈包做“携带信息”的 opcode 转发
            fb_active = true;
            fb_meta_sent = false;
        }
    }

    if (fb_active) {
        // 只处理发往 n0 的反馈；其他目的地可按需扩展
        if (is_dst(fb_meta, ip_address_n0)) {
            if (!fb_meta_sent) {
                m_axis_rx_meta_n0.write(fb_meta);
                txIbOpcodeFifo_n0.write(fb_op);  // meta 与 opcode 同步发出
                fb_meta_sent = true;
            }

            // 连续 drain data 到 last（不与别的包交织）
            if (!s_axis_tx_data_n1.empty()) {
                net_axis<WIDTH> w;
                s_axis_tx_data_n1.read(w);
                m_axis_rx_data_n0.write(w);

                if (w.last) {
                    fb_active = false;
                    fb_meta_sent = false;
                }
            }
        } else {
            // 若不是反馈包：可直接按原逻辑 forward（这里给一个保守直通到 n1）
            if (!fb_meta_sent) {
                m_axis_rx_meta_n1.write(fb_meta);
                fb_meta_sent = true;
            }
            if (!s_axis_tx_data_n1.empty()) {
                net_axis<WIDTH> w;
                s_axis_tx_data_n1.read(w);
                m_axis_rx_data_n1.write(w);
                if (w.last) {
                    fb_active = false;
                    fb_meta_sent = false;
                }
            }
        }
    }

    // ------------------ 2) multi-path aggregation: cache metas from 0/2/3/4 ------------------
    try_read_meta(0);
    try_read_meta(2);
    try_read_meta(3);
    try_read_meta(4);

    // ------------------ 3) choose next packet for dst=1 with controllable reorderRate ------------------
    if (active == 7) {
        // 收集 ready ports：meta valid + dst==n1 + 至少有 1 个 data beat
        int ports[4] = {0,2,3,4};
        bool ready[4] = {false,false,false,false};
        int ready_cnt = 0;

        for (int i=0;i<4;i++){
            int p = ports[i];
            bool v = vld_of(p);
            if (v) {
                ipUdpMeta &m = meta_of(p);
                if (is_dst(m, ip_address_n1) && !data_empty(p)) {
                    ready[i] = true;
                    ready_cnt++;
                }
            }
        }

        if (ready_cnt > 0) {
            // 默认 RR 选择
            int rr_idx = -1;
            for (int k=0;k<4;k++){
                int idx = (int)((rr + k) & 0x3);
                if (ready[idx]) { rr_idx = idx; break; }
            }

            // 以 reorderRate 概率选择“非 RR”的另一路（如果存在）
            int pick_idx = rr_idx;
            if (ready_cnt >= 2 && reorderRate > 0.0) {
                double r = rng01(rng_state);
                if (r < reorderRate) {
                    // 选择 rr_idx 之后的下一个 ready（制造轻度乱序）
                    for (int k=1;k<4;k++){
                        int idx = (rr_idx + k) & 0x3;
                        if (ready[idx]) { pick_idx = idx; break; }
                    }
                }
            }

            // 锁定 active
            active = ports[pick_idx];
            agg_meta_sent = false;

            // 更新 RR 指针到 pick 的后一个
            rr = (ap_uint<2>)((pick_idx + 1) & 0x3);
        }
    }

    // ------------------ 4) send selected packet: meta once + drain data until last ------------------
    if (active != 7) {
        int p = (int)active;
        ipUdpMeta &m = meta_of(p);
        bool &v = vld_of(p);
        bool &dp = drop_of(p);

        if (!agg_meta_sent) {
            if (!dp) {
                m_axis_rx_meta_n1.write(m);
            }
            agg_meta_sent = true;
        }

        if (!data_empty(p)) {
            net_axis<WIDTH> w;
            read_data(p, w);

            if (!dp) {
                m_axis_rx_data_n1.write(w);
            }

            if (w.last) {
                v = false;
                dp = false;
                active = 7;
                agg_meta_sent = false;
            }
        }
    }

    // ------------------ 5) forward non-n1 traffic from paths (可选：保持原直通) ------------------
    // 如果你只有“到 n1 的数据”和“到 n0 的反馈”，这一段可以不写。
}
