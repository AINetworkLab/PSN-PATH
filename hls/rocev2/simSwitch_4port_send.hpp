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

#define SEND_N_NODE_4 4
// #define N_NODE_5 5
#define PORT_LOC 5000
#define PORT_RMT 5001
#define PKT_LEN  64
#define PKT_LEN2 64
#define CLK_SIM 5000

#ifdef SEND_N_NODE_4
#define SENDFWDMETA(srcNode)                                                        \
    if (!s_axis_tx_meta_n##srcNode.empty() && !udpMetaVldN##srcNode){           \
        s_axis_tx_meta_n##srcNode.read(udpMetaN##srcNode);                      \
        udpMetaVldN##srcNode = true;                                            \
        cntPacketN##srcNode++;                                                  \
        if(srcNode == 0){\
            recvIbOpcodeFifo_n0.read(ib_opN0);\
            onSend = true;\
        }\
        else{\
            onSend = false;\
        }\
        if(!onSend || onSend&&(ib_opN0.op==0x11 || ib_opN0.op==0x12)){               \
        if(udpMetaN##srcNode.their_address==ip_address_n0)                      \
            m_axis_rx_meta_n0.write(udpMetaN##srcNode);                         \
        else if(udpMetaN##srcNode.their_address==ip_address_n1)                 \
            m_axis_rx_meta_n1.write(udpMetaN##srcNode);                         \
        else if(udpMetaN##srcNode.their_address==ip_address_n2)                 \
            m_axis_rx_meta_n2.write(udpMetaN##srcNode);                         \
        else                                                                    \
            m_axis_rx_meta_n3.write(udpMetaN##srcNode);                         \
        }                \
        else{\
            if(ib_opN0.psn%SEND_N_NODE_4==0){                                    \
            m_axis_rx_meta_n1.write(udpMetaN##srcNode);                         \
            txIbOpcodeFifo_n1.write(ib_opN0);                           \
            std::cout<<"send switch:"<<ib_opN0.psn<<"to port 2"<<std::endl;\
        }                                                                       \
        else if(ib_opN0.psn%SEND_N_NODE_4==1){                               \
            m_axis_rx_meta_n2.write(udpMetaN##srcNode);                         \
            txIbOpcodeFifo_n2.write(ib_opN0);                           \
            std::cout<<"send switch:"<<ib_opN0.psn<<"to port 4"<<std::endl;\
        }                                                                       \
        else if(ib_opN0.psn%SEND_N_NODE_4==2){                               \
            m_axis_rx_meta_n3.write(udpMetaN##srcNode);                         \
            txIbOpcodeFifo_n3.write(ib_opN0);                           \
            std::cout<<"send switch:"<<ib_opN0.psn<<"to port 6"<<std::endl;\
        }                                                                       \
        else if(ib_opN0.psn%SEND_N_NODE_4==3){                               \
            m_axis_rx_meta_n4.write(udpMetaN##srcNode);                         \
            txIbOpcodeFifo_n4.write(ib_opN0);                           \
            std::cout<<"send switch:"<<ib_opN0.psn<<"to port 8"<<std::endl;\
        }                                                                       \
        }\
        }
#else
#define SENDFWDMETA(srcNode)                                                        \
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

#ifdef SEND_N_NODE_4
#define SENDFWDATA(srcNode)                                                \
    if (!s_axis_tx_data_n##srcNode.empty() && udpMetaVldN##srcNode){    \
        s_axis_tx_data_n##srcNode.read(currWordN##srcNode);             \
        if(!onSend || onSend && (ib_opN0.op==0x11 || ib_opN0.op==0x12)){       \
        if(udpMetaN##srcNode.their_address==ip_address_n0)              \
            m_axis_rx_data_n0.write(currWordN##srcNode);                \
        else if(udpMetaN##srcNode.their_address==ip_address_n1)         \
            m_axis_rx_data_n1.write(currWordN##srcNode);                \
        else if(udpMetaN##srcNode.their_address==ip_address_n2)         \
            m_axis_rx_data_n2.write(currWordN##srcNode);                \
        else                                                            \
            m_axis_rx_data_n3.write(currWordN##srcNode);                \
        }                                                               \
        else{                                                           \
        if(ib_opN0.psn%SEND_N_NODE_4==0){                            \
            m_axis_rx_data_n1.write(currWordN##srcNode);                \
        }                                                               \
        else if(ib_opN0.psn%SEND_N_NODE_4==1){                       \
            m_axis_rx_data_n2.write(currWordN##srcNode);                \
        }                                                                       \
        else if(ib_opN0.psn%SEND_N_NODE_4==2){                               \
            m_axis_rx_data_n3.write(currWordN##srcNode);                         \
        }                                                                       \
        else if(ib_opN0.psn%SEND_N_NODE_4==3){                               \
            m_axis_rx_data_n4.write(currWordN##srcNode);                         \
        }                                                                 \
        }\
        if(currWordN##srcNode.last){                                    \
            udpMetaVldN##srcNode = false;                               \
            dropPacketN##srcNode = false;                               \
        }                                                               \
    }
#else
#define SENDFWDATA(srcNode)                                                \
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
void simSwitchSend( 
    // RX - net module
    stream<ipUdpMeta>& m_axis_rx_meta_n0,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n0,
    // TX - net module
    stream<ipUdpMeta>& s_axis_tx_meta_n0,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n0,
    stream<testForSwitch>&   recvIbOpcodeFifo_n0,
    stream<testForSwitch>&   txIbOpcodeFifo_n0,

    stream<ipUdpMeta>& m_axis_rx_meta_n1,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n1,
    stream<ipUdpMeta>& s_axis_tx_meta_n1,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n1,
    stream<testForSwitch>&   txIbOpcodeFifo_n1,

#ifdef SEND_N_NODE_4
    stream<ipUdpMeta>& m_axis_rx_meta_n2,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n2,
    stream<ipUdpMeta>& s_axis_tx_meta_n2,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n2,
    stream<testForSwitch>&   txIbOpcodeFifo_n2,

    stream<ipUdpMeta>& m_axis_rx_meta_n3,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n3,
    stream<ipUdpMeta>& s_axis_tx_meta_n3,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n3,
    stream<testForSwitch>&   txIbOpcodeFifo_n3,

    stream<ipUdpMeta>& m_axis_rx_meta_n4,
    stream<net_axis<WIDTH> >& m_axis_rx_data_n4,
    stream<ipUdpMeta>& s_axis_tx_meta_n4,
    stream<net_axis<WIDTH> >& s_axis_tx_data_n4,
    stream<testForSwitch>&   txIbOpcodeFifo_n4,
#endif

    ap_uint<128> ip_address_n0,
    ap_uint<128> ip_address_n1,
#ifdef SEND_N_NODE_4
    ap_uint<128> ip_address_n2,
    ap_uint<128> ip_address_n3,
    ap_uint<128> ip_address_n4,
#endif

    double dropEveryNPacket // 0 means no drop
){

#pragma HLS inline off
#pragma HLS pipeline II=1

    static ipUdpMeta udpMetaN0, udpMetaN1;
    static testForSwitch ib_opN0;
    static bool udpMetaVldN0 = false, udpMetaVldN1 = false;
    static bool dropPacketN0 = false, dropPacketN1 = false;
    static uint32_t cntPacketN0 = 0, cntPacketN1 = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);
#ifdef SEND_N_NODE_4
    static ipUdpMeta udpMetaN2, udpMetaN3, udpMetaN4;
    static bool udpMetaVldN2 = false, udpMetaVldN3 = false, udpMetaVldN4 = false;
    static bool dropPacketN2 = false, dropPacketN3 = false, dropPacketN4 = false;
    static uint32_t cntPacketN2 = 0, cntPacketN3 = 0, cntPacketN4 = 0;    
#endif

    net_axis<WIDTH> currWordN0, currWordN1;
#ifdef SEND_N_NODE_4
    net_axis<WIDTH> currWordN2, currWordN3, currWordN4;
#endif
    static bool onSend = true;

//     SENDFWDMETA(0);
    SENDFWDMETA(1);
#ifdef SEND_N_NODE_4
    SENDFWDMETA(2);
    SENDFWDMETA(3);
    SENDFWDMETA(4);
#endif

//     SENDFWDATA(0);
    SENDFWDATA(1);
#ifdef SEND_N_NODE_4
    SENDFWDATA(2);
    SENDFWDATA(3);
    SENDFWDATA(4);
#endif
// 在 simSwitchSend 内增加
static ap_uint<3> dstSelN0 = 0;
static bool metaSentN0 = false;

// 小工具：根据 dstSel 写 meta/data/opcode（dstSel: 1..4 对应 n1..n4）
auto write_meta_dst = [&](ap_uint<3> d, const ipUdpMeta& m){
    if(d==1) m_axis_rx_meta_n1.write(m);
    else if(d==2) m_axis_rx_meta_n2.write(m);
    else if(d==3) m_axis_rx_meta_n3.write(m);
    else if(d==4) m_axis_rx_meta_n4.write(m);
    else          m_axis_rx_meta_n0.write(m);
};
auto write_data_dst = [&](ap_uint<3> d, const net_axis<WIDTH>& w){
    if(d==1) m_axis_rx_data_n1.write(w);
    else if(d==2) m_axis_rx_data_n2.write(w);
    else if(d==3) m_axis_rx_data_n3.write(w);
    else if(d==4) m_axis_rx_data_n4.write(w);
    else          m_axis_rx_data_n0.write(w);
};
auto write_op_dst = [&](ap_uint<3> d, const testForSwitch& op){
    if(d==1) txIbOpcodeFifo_n1.write(op);
    else if(d==2) txIbOpcodeFifo_n2.write(op);
    else if(d==3) txIbOpcodeFifo_n3.write(op);
    else if(d==4) txIbOpcodeFifo_n4.write(op);
    else          txIbOpcodeFifo_n0.write(op);
};

// --- 替换 SENDFWDMETA(0) 的核心逻辑 ---
if (!s_axis_tx_meta_n0.empty() && !udpMetaVldN0) {

    // 关键：等待 opcode 准备好，避免空读/错配
    if (!recvIbOpcodeFifo_n0.empty()) {
        s_axis_tx_meta_n0.read(udpMetaN0);
        recvIbOpcodeFifo_n0.read(ib_opN0);

        udpMetaVldN0 = true;
        metaSentN0 = false;

        // 锁存 dstSel：该包全程不变
        if (ib_opN0.op==0x11 || ib_opN0.op==0x12) {
            // 直通：按 their_address
            if(udpMetaN0.their_address==ip_address_n0) dstSelN0 = 0;
            else if(udpMetaN0.their_address==ip_address_n1) dstSelN0 = 1;
            else if(udpMetaN0.their_address==ip_address_n2) dstSelN0 = 2;
            else if(udpMetaN0.their_address==ip_address_n3) dstSelN0 = 3;
            else dstSelN0 = 4;
        } else {
            // 分流：psn%4 -> 1..4
            dstSelN0 = 1 + (ib_opN0.psn % SEND_N_NODE_4);
        }

        // meta 与 opcode 在“确定 dstSel 后”立即送出
        write_meta_dst(dstSelN0, udpMetaN0);
        if (!(ib_opN0.op==0x11 || ib_opN0.op==0x12)) {
            write_op_dst(dstSelN0, ib_opN0);
        }
    }
}

// --- data 阶段：只用 dstSelN0，直到 last ---
if (!s_axis_tx_data_n0.empty() && udpMetaVldN0) {
    s_axis_tx_data_n0.read(currWordN0);
    write_data_dst(dstSelN0, currWordN0);
    if (currWordN0.last) {
        udpMetaVldN0 = false;
        metaSentN0 = false;
    }
}

}


// int testSimSwitch(double dropEveryNPacket){
// #pragma HLS inline region off

//     // RX - net module
//     stream<ipUdpMeta> s_axis_rx_meta_n0;
//     stream<net_axis<DATA_WIDTH> > s_axis_rx_data_n0;
//     stream<ipUdpMeta> s_axis_rx_meta_n1;
//     stream<net_axis<DATA_WIDTH> > s_axis_rx_data_n1;

//     // TX - net module
//     stream<ipUdpMeta> m_axis_tx_meta_n0;
//     stream<net_axis<DATA_WIDTH> > m_axis_tx_data_n0;
//     stream<ipUdpMeta> m_axis_tx_meta_n1;
//     stream<net_axis<DATA_WIDTH> > m_axis_tx_data_n1;

//     stream<testForSwitch> recvIbOpcodeFifo_n0;
//     stream<testForSwitch> recvIbOpcodeFifo_n1;
//     ap_uint<128> ip_address_n0, ip_address_n1;
//     ip_address_n0(127, 64) = 0xfe80000000000000;
//     ip_address_n0(63, 0)   = 0x92e2baff0b01d4d2;
//     ip_address_n1(127, 64) = 0xfe80000000000000;
//     ip_address_n1(63, 0)   = 0x92e2baff0b01d4d3;

//     ipUdpMeta metaN0 = ipUdpMeta(ip_address_n1, PORT_RMT, PORT_LOC, PKT_LEN);
//     ipUdpMeta metaN1 = ipUdpMeta(ip_address_n0, PORT_RMT, PORT_LOC, PKT_LEN2);

//     // write test packets to n0 tx
//     for (int i=0; i<8; i++){
//         m_axis_tx_meta_n0.write(metaN0);
//         for (int j=0; j<PKT_LEN; j+=DATA_WIDTH/8){
//             bool isLast = ((j+DATA_WIDTH/8)>=PKT_LEN) ? true : false;
//             m_axis_tx_data_n0.write(net_axis<DATA_WIDTH>((0x0000<<16)+(i<<8)+j/(DATA_WIDTH/8), lenToKeep(isLast ? PKT_LEN-j : DATA_WIDTH/8), isLast));
//         }
//     }

//     // write test packets to n1 tx
//     for (int i=0; i<8; i++){
//         m_axis_tx_meta_n1.write(metaN1);
//         for (int j=0; j<PKT_LEN2; j+=DATA_WIDTH/8){
//             bool isLast = ((j+DATA_WIDTH/8)>=PKT_LEN2) ? true : false;
//             m_axis_tx_data_n1.write(net_axis<DATA_WIDTH>((0x0001<<16)+(i<<8)+j/(DATA_WIDTH/8), lenToKeep(isLast ? PKT_LEN2-j : DATA_WIDTH/8), isLast));
//         }
//     }

//     for (int i=0; i<CLK_SIM; i++){
//         simSwitchD<DATA_WIDTH>(
//             s_axis_rx_meta_n0,
//             s_axis_rx_data_n0,
//             m_axis_tx_meta_n0,
//             m_axis_tx_data_n0,
//             recvIbOpcodeFifo_n0,
//             s_axis_rx_meta_n1,
//             s_axis_rx_data_n1,
//             m_axis_tx_meta_n1,
//             m_axis_tx_data_n1,
//             recvIbOpcodeFifo_n1,
//             ip_address_n0,
//             ip_address_n1,
//             dropEveryNPacket
//         );

//         // monitor the n1 rx
//         PRTRXMETA(1);
//         PRTRXDATA(1);

//         // monitor the n0 rx
//         PRTRXMETA(0);
//         PRTRXDATA(0);

//     }

//     return 0;
// }



