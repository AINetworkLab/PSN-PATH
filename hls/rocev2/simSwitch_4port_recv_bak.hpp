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
#include <random>
using namespace hls;

#define RECV_N_NODE_4 4
#define PORT_LOC 5000
#define PORT_RMT 5001
#define PKT_LEN  64
#define PKT_LEN2 64
#define CLK_SIM 5000

#ifdef RECV_N_NODE_4
#define RECVFWMETA(srcNode)                                                        \
    if (!s_axis_tx_meta_n##srcNode.empty() && !udpMetaVldN##srcNode){           \
        s_axis_tx_meta_n##srcNode.read(udpMetaN##srcNode);                      \
        udpMetaVldN##srcNode = true;                                            \
        cntPacketN##srcNode++;                                                  \
        std::cout<<"recv switch:来自 "<<srcNode<<std::endl;\
        if(udpMetaN##srcNode.their_address==ip_address_n0)                 \
            m_axis_rx_meta_n0.write(udpMetaN##srcNode);                         \
        else if(udpMetaN##srcNode.their_address==ip_address_n1)                 \
            m_axis_rx_meta_n1.write(udpMetaN##srcNode);                         \
        else if(udpMetaN##srcNode.their_address==ip_address_n2)                 \
            m_axis_rx_meta_n2.write(udpMetaN##srcNode);                         \
        else                                                                    \
            m_axis_rx_meta_n3.write(udpMetaN##srcNode);                         \
        if(srcNode !=1){                 \
            onSend = true;              \
        }                               \
        else{                               \
            if(!recvIbOpcodeFifo_n1.empty()){   \
                testForSwitch ib_op;            \
                recvIbOpcodeFifo_n1.read(ib_op);\
                txIbOpcodeFifo_n0.write(ib_op); \
            }                                       \
        }                                               \
        }
#else
#define RECVFWMETA(srcNode)                                                        \
    if (!s_axis_tx_meta_n##srcNode.empty() && !udpMetaVldN##srcNode && !recvIbOpcodeFifo_n##srcNode.empty()){           \
        s_axis_tx_meta_n##srcNode.read(udpMetaN##srcNode);                      \
        recvIbOpcodeFifo_n##srcNode.read(ib_opN##srcNode);                      \
        udpMetaVldN##srcNode = true;                                            \
        cntPacketN##srcNode++;                                                  \
        double loss_r=dis(gen);                                                 \
        /*std::cout<<"loss_r:\t"<<loss_r<<std::endl;*/                              \
        if(loss_r<dropEveryNPacket && ib_opN##srcNode.op == 0x07 && ib_opN##srcNode.retr==0){              \
            std::cout<<"发生丢失:\t"<<ib_opN##srcNode.psn<<std::endl;            \
            dropPacketN##srcNode = true;                                        \
        }else{                                                                  \
            dropPacketN##srcNode = false;                                       \
        }                                                                       \
        if(dropPacketN##srcNode);                                               \
        else if(udpMetaN##srcNode.their_address==ip_address_n0)                 \
            m_axis_rx_meta_n0.write(udpMetaN##srcNode);                         \
        else if(udpMetaN##srcNode.their_address==ip_address_n1)                 \
            m_axis_rx_meta_n1.write(udpMetaN##srcNode);                         \
        else                                                                    \
            std::cout << "[ERROR] Non-existing IP:" << std::hex                 \
                << udpMetaN##srcNode.their_address << std::endl;                \
    }                                                                   
#endif

#ifdef RECV_N_NODE_4
#define RECVFWDATA(srcNode)                                                \
    if (!s_axis_tx_data_n##srcNode.empty() && udpMetaVldN##srcNode){    \
        s_axis_tx_data_n##srcNode.read(currWordN##srcNode);             \
        if(dropPacketN##srcNode);                                        \
        else if(udpMetaN##srcNode.their_address==ip_address_n0)         \
            m_axis_rx_data_n0.write(currWordN##srcNode);                \
        else if(udpMetaN##srcNode.their_address==ip_address_n1)         \
            m_axis_rx_data_n1.write(currWordN##srcNode);                \
        else if(udpMetaN##srcNode.their_address==ip_address_n2)         \
            m_axis_rx_data_n2.write(currWordN##srcNode);                \
        else                                                            \
            m_axis_rx_data_n3.write(currWordN##srcNode);                \
        if(currWordN##srcNode.last){                                    \
            udpMetaVldN##srcNode = false;                               \
            dropPacketN##srcNode = false;                               \
            if(srcNode!=1){\
                onSend = false;\
            }\
        }                                                               \
    }
#else
#define RECVFWDATA(srcNode)                                                \
    if (!s_axis_tx_data_n##srcNode.empty() && udpMetaVldN##srcNode){    \
        s_axis_tx_data_n##srcNode.read(currWordN##srcNode);             \
        if(dropPacketN##srcNode);                                       \
        else if(udpMetaN##srcNode.their_address==ip_address_n0)         \
            m_axis_rx_data_n0.write(currWordN##srcNode);                \
        else                                                            \
            m_axis_rx_data_n1.write(currWordN##srcNode);                \
        if(currWordN##srcNode.last){                                    \
            udpMetaVldN##srcNode = false;                               \
            dropPacketN##srcNode = false;                               \
        }                                                               \
    }
#endif


// NOTE: print function will eat the data on bus
#define PRTRXMETA(node)                                                                         \
    if (!s_axis_rx_meta_n##node.empty()){                                                       \
        ipUdpMeta udpMeta;                                                                      \
        s_axis_rx_meta_n##node.read(udpMeta);                                                   \
        std::cout << "[s_axis_rx_meta_n" << node << "]:\t"                                      \
            << "dstIP:" << std::hex << udpMeta.their_address << std::dec                        \
            << ", dstPort:" << udpMeta.their_port                                               \
            << ", srcPort:" << udpMeta.my_port                                                  \
            << ", Len:" << udpMeta.length << std::endl;                                         \
    }

#define PRTRXDATA(node)                                                                         \
        if (!s_axis_rx_data_n##node.empty()){                                                   \
            net_axis<DATA_WIDTH> curWord;                                                       \
            s_axis_rx_data_n##node.read(curWord);                                               \
            std::cout << "[s_axis_rx_data_n" << node << "]:\t"                                  \
                << "Data:" << std::hex << curWord.data                                          \
                << ", Keep:" << curWord.keep << ", isLast:" << curWord.last << std::endl;       \
        }

#define PRTTXMETA(node)                                                                         \
    if (!m_axis_tx_meta_n##node.empty()){                                                       \
        ipUdpMeta udpMeta;                                                                      \
        m_axis_tx_meta_n##node.read(udpMeta);                                                   \
        std::cout << "[m_axis_tx_meta_n" << node << "]:\t"                                      \
            << "dstIP:" << std::hex << udpMeta.their_address << std::dec                        \
            << ", dstPort:" << udpMeta.their_port                                               \
            << ", srcPort:" << udpMeta.my_port                                                  \
            << ", Len:" << udpMeta.length << std::endl;                                         \
    }

#define PRTTXDATA(node)                                                                         \
        if (!m_axis_tx_data_n##node.empty()){                                                   \
            net_axis<DATA_WIDTH> curWord;                                                       \
            m_axis_tx_data_n##node.read(curWord);                                               \
            std::cout << "[m_axis_tx_data_n" << node << "]:\t"                                  \
                << "Data:" << std::hex << curWord.data                                          \
                << ", Keep:" << curWord.keep << ", isLast:" << curWord.last << std::endl;       \
        }


// ------------------------------------------------------------------------------------------------
// simulate switch behavior with udp packets
// ------------------------------------------------------------------------------------------------
template <int WIDTH>
void simSwitchRecv( 
    // RX - net module
    stream<ipUdpMeta>& m_axis_rx_meta_n0,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n0,
    // TX - net module
    stream<ipUdpMeta>& s_axis_tx_meta_n0,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n0,
    stream<testForSwitch>&   recvIbOpcodeFifo_n1,
    stream<testForSwitch>&   txIbOpcodeFifo_n0,

    stream<ipUdpMeta>& m_axis_rx_meta_n1,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n1,
    stream<ipUdpMeta>& s_axis_tx_meta_n1,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n1,

#ifdef RECV_N_NODE_4
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
#endif

    ap_uint<128> ip_address_n0,
    ap_uint<128> ip_address_n1,
#ifdef RECV_N_NODE_4
    ap_uint<128> ip_address_n2,
    ap_uint<128> ip_address_n3,
    ap_uint<128> ip_address_n4,
#endif

    double dropEveryNPacket // 0 means no drop
){

#pragma HLS inline off
#pragma HLS pipeline II=1

    static ipUdpMeta udpMetaN0, udpMetaN1;
    static testForSwitch ib_opN0,ib_opN1;
    static bool udpMetaVldN0 = false, udpMetaVldN1 = false;
    static bool dropPacketN0 = false, dropPacketN1 = false;
    static uint32_t cntPacketN0 = 0, cntPacketN1 = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);
    static bool lockN0 = true, lockN1 = true;
#ifdef RECV_N_NODE_4
    static ipUdpMeta udpMetaN2, udpMetaN3, udpMetaN4;
    static bool udpMetaVldN2 = false, udpMetaVldN3 = false, udpMetaVldN4 = false;
    static bool dropPacketN2 = false, dropPacketN3 = false, dropPacketN4 = false;
    static uint32_t cntPacketN2 = 0, cntPacketN3 = 0, cntPacketN4 = 0;  
    static bool lockN2 = true, lockN3 = true, lockN4 = true;  
#endif

    net_axis<WIDTH> currWordN0, currWordN1;
#ifdef RECV_N_NODE_4
    net_axis<WIDTH> currWordN2, currWordN3, currWordN4;
#endif
//     static bool onSend = false;
//     static uint8_t lock = 0;
//     static uint8_t port = 0;

//     if(!onSend){
//         lock = lock%4;
//         if(lock==0){
//             port = 0;
//         }else if(lock==1){
//             port = 2;
//         }else if(lock == 2){
//             port = 3;
//         }else{
//             port = 4;
//         }
//         lock += 1;
//     }
//     RECVFWMETA(1);
//     RECVFWDATA(1);
//     if(port==0){
//         RECVFWMETA(0);
//         RECVFWDATA(0);
//     }
//     else if(port==2){
//         RECVFWMETA(2);
//         RECVFWDATA(2);
//     }
//     else if(port==3){
//         RECVFWMETA(3);
//         RECVFWDATA(3);
//     }
//     else{
//         RECVFWMETA(4);
//         RECVFWDATA(4);
//     }
//     RECVFWMETA(0);
//     RECVFWMETA(1);
// #ifdef RECV_N_NODE_4
//     RECVFWMETA(2);
//     RECVFWMETA(3);
//     RECVFWMETA(4);
// #endif

//     RECVFWDATA(0);
//     RECVFWDATA(1);
// #ifdef RECV_N_NODE_4
//     RECVFWDATA(2);
//     RECVFWDATA(3);
//     RECVFWDATA(4);
// #endif
// 在 simSwitchRecv 内增加
static ap_uint<3> active = 7;        // 7 = none, 0/2/3/4 = chosen input for dst=1
static bool meta_sent = false;
static ap_uint<3> rr = 0;

// 先把各输入的 meta 读入本地，但不立刻 forward meta
auto try_read_meta = [&](int s){
    if (s==0 && !s_axis_tx_meta_n0.empty() && !udpMetaVldN0){ s_axis_tx_meta_n0.read(udpMetaN0); udpMetaVldN0=true; }
    if (s==2 && !s_axis_tx_meta_n2.empty() && !udpMetaVldN2){ s_axis_tx_meta_n2.read(udpMetaN2); udpMetaVldN2=true; }
    if (s==3 && !s_axis_tx_meta_n3.empty() && !udpMetaVldN3){ s_axis_tx_meta_n3.read(udpMetaN3); udpMetaVldN3=true; }
    if (s==4 && !s_axis_tx_meta_n4.empty() && !udpMetaVldN4){ s_axis_tx_meta_n4.read(udpMetaN4); udpMetaVldN4=true; }
};

// 每拍先尝试缓存 meta（不会改变输出顺序）
try_read_meta(0); try_read_meta(2); try_read_meta(3); try_read_meta(4);

// 选择一个 active（只选择“dst=1 且有包在进行”的输入）
if (active == 7) {
    for(int k=0;k<4;k++){
        int s = (rr + k) % 4;
        int p = (s==0)?0:(s==1)?2:(s==2)?3:4; // rr:0..3 -> port:0,2,3,4
        bool vld = (p==0)?udpMetaVldN0:(p==2)?udpMetaVldN2:(p==3)?udpMetaVldN3:udpMetaVldN4;
        ipUdpMeta m = (p==0)?udpMetaN0:(p==2)?udpMetaN2:(p==3)?udpMetaN3:udpMetaN4;
        if (vld && m.their_address == ip_address_n1) { active = p; meta_sent=false; rr=(s+1)%4; break; }
    }
}

// 若已选中 active，则先发 meta（只一次），再 drain data 直到 last
if (active != 7) {
    ipUdpMeta &m = (active==0)?udpMetaN0:(active==2)?udpMetaN2:(active==3)?udpMetaN3:udpMetaN4;

    if (!meta_sent) {
        m_axis_rx_meta_n1.write(m);
        meta_sent = true;
    }

    // drain data
    bool data_ok = (active==0)?!s_axis_tx_data_n0.empty():
                   (active==2)?!s_axis_tx_data_n2.empty():
                   (active==3)?!s_axis_tx_data_n3.empty():
                               !s_axis_tx_data_n4.empty();
    if (data_ok) {
        net_axis<WIDTH> w;
        if(active==0) s_axis_tx_data_n0.read(w);
        else if(active==2) s_axis_tx_data_n2.read(w);
        else if(active==3) s_axis_tx_data_n3.read(w);
        else s_axis_tx_data_n4.read(w);

        m_axis_rx_data_n1.write(w);

        if (w.last) {
            // 释放该输入的包状态
            if(active==0) udpMetaVldN0=false;
            else if(active==2) udpMetaVldN2=false;
            else if(active==3) udpMetaVldN3=false;
            else udpMetaVldN4=false;

            active = 7;
            meta_sent = false;
        }
    }
}

// 反馈方向 src=1 -> dst=0 建议单独通道处理（输出口不同，可以并行），同样做到“meta 与 opcode 同步”
static bool fb_active = false;
static bool fb_meta_sent = false;
static ipUdpMeta fb_meta;
static testForSwitch fb_op;

if (!fb_active) {
    // 等 meta + opcode 都就绪才启动一个反馈包
    if (!s_axis_tx_meta_n1.empty() && !recvIbOpcodeFifo_n1.empty()) {
        // 先窥探 meta，确认它确实是发往 n0 的反馈包（可选）
        // 这里简单起见直接读出来再判断
        s_axis_tx_meta_n1.read(fb_meta);
        recvIbOpcodeFifo_n1.read(fb_op);

        // 若不是发往 n0，你可以把它走普通路径（此处略）
        // 假设反馈包一定发往 n0：
        fb_active = true;
        fb_meta_sent = false;
    }
}

if (fb_active) {
    if (!fb_meta_sent) {
        m_axis_rx_meta_n0.write(fb_meta);
        txIbOpcodeFifo_n0.write(fb_op);   // 和 meta 同步发出
        fb_meta_sent = true;
    }

    // 连续 drain data 直到 last（包边界不破坏）
    if (!s_axis_tx_data_n1.empty()) {
        net_axis<WIDTH> w;
        s_axis_tx_data_n1.read(w);
        m_axis_rx_data_n0.write(w);

        if (w.last) {
            fb_active = false;
            fb_meta_sent = false;
        }
    }
}

}
