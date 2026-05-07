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
#include "rocev2.hpp"
#include <fstream>
#include <vector>
#include <algorithm>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h> /* Added for the nonblocking socket */
#include <cstdint>
#include <numeric>

#include "../axi_utils.hpp" //TODO why is this needed here
#include "../ib_transport_protocol/ib_transport_protocol.hpp"
#include "rocev2_config.hpp"

using namespace hls;
#include "newFakeDram.hpp"
#include "simSwitch.hpp"
#include "simSwitch_4port_send.hpp"
#include "simSwitch_4port_recv.hpp"

#define IBTPORT(ninst)                                                   \
    static stream<txMeta> s_axis_sq_meta_n##ninst;                       \
    static stream<ackMeta> m_axis_rx_ack_meta_n##ninst;                  \
    static stream<qpContext> s_axis_qp_interface_n##ninst;               \
    static stream<ifConnReq> s_axis_qp_conn_interface_n##ninst;          \
    static stream<memCmd> m_axis_mem_write_cmd_n##ninst;                 \
    static stream<memCmd> m_axis_mem_read_cmd_n##ninst;                  \
    static stream<net_axis<DATA_WIDTH> > m_axis_mem_write_data_n##ninst; \
    static stream<net_axis<DATA_WIDTH> > s_axis_mem_read_data_n##ninst;  \
    ap_uint<32> regInvalidPsnDropCount_n##ninst;                         \
    ap_uint<32> regRetransCount_n##ninst;                                \
    ap_uint<32> regValidIbvCountRx_n##ninst;                             \
    ap_uint<32> regValidIbvCountTx_n##ninst;                             \
    int sendPkgNum_n##ninst;                                             \
    bool isrecv_n##ninst;                                                \
    int disOrderCount_n##ninst;                                            \
    static stream<testForSwitch> ibOpCodeFifo_n##ninst;\

#define IBTRUN(ninst)                               \
    ib_transport_protocol<DATA_WIDTH, ninst>(       \
        s_axis_rx_meta_n##ninst,                    \
        s_axis_rx_data_n##ninst,                    \
        m_axis_tx_meta_n##ninst,                    \
        m_axis_tx_data_n##ninst,                    \
        s_axis_sq_meta_n##ninst,                    \
        m_axis_rx_ack_meta_n##ninst,                \
        m_axis_mem_write_cmd_n##ninst,              \
        m_axis_mem_read_cmd_n##ninst,               \
        m_axis_mem_write_data_n##ninst,             \
        s_axis_mem_read_data_n##ninst,              \
        s_axis_qp_interface_n##ninst,               \
        s_axis_qp_conn_interface_n##ninst,          \
        regInvalidPsnDropCount_n##ninst,            \
        regRetransCount_n##ninst,                   \
        regValidIbvCountRx_n##ninst,                \
        regValidIbvCountTx_n##ninst,                \
        ibOpCodeFifo_n##ninst,                      \
        sendPkgNum_n##ninst,                        \
        isrecv_n##ninst,                            \
        disOrderCount_n##ninst                        \
    );

#define SWITCHPORT(port)                                    \
    stream<ipUdpMeta> s_axis_rx_meta_n##port;               \
    stream<net_axis<DATA_WIDTH> > s_axis_rx_data_n##port;   \
    stream<ipUdpMeta> m_axis_tx_meta_n##port;               \
    stream<net_axis<DATA_WIDTH> > m_axis_tx_data_n##port;   \
    static stream<testForSwitch> txIbOpCodeFifo_n##port;      \

#define SWITCHRUN(p0,p1,dropEveryNPacket)                 \
    simSwitch<DATA_WIDTH>(                          \
        m_axis_tx_meta_n##p0,                          \
        m_axis_tx_data_n##p0,                          \
        s_axis_rx_meta_n##p0,                          \
        s_axis_rx_data_n##p0,                          \
        txIbOpCodeFifo_n##p0,                            \
        m_axis_tx_meta_n##p1,                          \
        m_axis_tx_data_n##p1,                          \
        s_axis_rx_meta_n##p1,                          \
        s_axis_rx_data_n##p1,                          \
        txIbOpCodeFifo_n##p1,                            \
        ipAddrN0,                                   \
        ipAddrN1,                                   \
        dropEveryNPacket                           \
    );

#define SWITCHRUNSEND(s0,s1,s2,s3,s4,dropEveryNPacket)                 \
    simSwitchSend<DATA_WIDTH>(                          \
        s_axis_rx_meta_n##s0,                          \
        s_axis_rx_data_n##s0,                          \
        m_axis_tx_meta_n##s0,                          \
        m_axis_tx_data_n##s0,                          \
        ibOpCodeFifo_n##s0,                        \
        txIbOpCodeFifo_n##s0,                            \
        s_axis_rx_meta_n##s1,                          \
        s_axis_rx_data_n##s1,                          \
        m_axis_tx_meta_n##s1,                          \
        m_axis_tx_data_n##s1,                          \
        txIbOpCodeFifo_n##s1,                            \
        s_axis_rx_meta_n##s2,                          \
        s_axis_rx_data_n##s2,                          \
        m_axis_tx_meta_n##s2,                          \
        m_axis_tx_data_n##s2,                          \
        txIbOpCodeFifo_n##s2,                            \
        s_axis_rx_meta_n##s3,                          \
        s_axis_rx_data_n##s3,                          \
        m_axis_tx_meta_n##s3,                          \
        m_axis_tx_data_n##s3,                          \
        txIbOpCodeFifo_n##s3,                            \
        s_axis_rx_meta_n##s4,                          \
        s_axis_rx_data_n##s4,                          \
        m_axis_tx_meta_n##s4,                          \
        m_axis_tx_data_n##s4,                          \
        txIbOpCodeFifo_n##s4,                            \
        ipAddrN0,                                   \
        ipAddrN1,                                   \
        ipAddrN2,                                   \
        ipAddrN3,                                   \
        ipAddrN4,                                   \
        dropEveryNPacket                            \
    );


#define SWITCHRUNRECV(r0,r1,r2,r3,r4,dropEveryNPacket,reorderRate)                 \
    simSwitchRecv<DATA_WIDTH>(                          \
        s_axis_rx_meta_n##r0,                          \
        s_axis_rx_data_n##r0,                          \
        m_axis_tx_meta_n##r0,                          \
        m_axis_tx_data_n##r0,                          \
        ibOpCodeFifo_n##r1,\
        txIbOpCodeFifo_n##r0,\
        s_axis_rx_meta_n##r1,                          \
        s_axis_rx_data_n##r1,                          \
        m_axis_tx_meta_n##r1,                          \
        m_axis_tx_data_n##r1,                          \
        s_axis_rx_meta_n##r2,                          \
        s_axis_rx_data_n##r2,                          \
        m_axis_tx_meta_n##r2,                          \
        m_axis_tx_data_n##r2,                          \
        s_axis_rx_meta_n##r3,                          \
        s_axis_rx_data_n##r3,                          \
        m_axis_tx_meta_n##r3,                          \
        m_axis_tx_data_n##r3,                          \
        s_axis_rx_meta_n##r4,                          \
        s_axis_rx_data_n##r4,                          \
        m_axis_tx_meta_n##r4,                          \
        m_axis_tx_data_n##r4,                          \
        ipAddrN0,                                   \
        ipAddrN1,                                   \
        ipAddrN2,                                   \
        ipAddrN3,                                   \
        ipAddrN4,                                   \
        dropEveryNPacket,                           \
        reorderRate                                 \
    );

#define DRAMRUN(ninst)                                                                    \
if (!m_axis_mem_write_cmd_n##ninst.empty() && !writeCmdReady[ninst]){                     \
    m_axis_mem_write_cmd_n##ninst.read(writeCmd[ninst]);                                  \
    writeCmdReady[ninst] = true;                                                          \
    writeRemainLen[ninst] = writeCmd[ninst].len;                                          \
    /*std::cout << "[Memory]: Write command, address: " << writeCmd[ninst].addr              \
        << ", length: " << std::dec <<writeCmd[ninst].len << std::endl; */                  \
}                                                                                         \
if (writeCmdReady[ninst] && !m_axis_mem_write_data_n##ninst.empty()){                     \
    net_axis<DATA_WIDTH> currWord;                                                        \
    m_axis_mem_write_data_n##ninst.read(currWord);                                        \
    writeRemainLen[ninst] -= (DATA_WIDTH/8);                                              \
    writeCmdReady[ninst] = (writeRemainLen[ninst] <= 0) ? false : true;                   \
}                                                                                         \
if (!m_axis_mem_read_cmd_n##ninst.empty()){                                               \
    m_axis_mem_read_cmd_n##ninst.read(readCmd[ninst]);                                    \
    memoryN##ninst.processRead(readCmd[ninst], s_axis_mem_read_data_n##ninst);            \
}                                                                                         \
if (!m_axis_rx_ack_meta_n##ninst.empty()){                                                \
    m_axis_rx_ack_meta_n##ninst.read(ackMeta[ninst]);                                     \
    std::cout << "[Ack " << ninst << "]: qpn: " << std::hex <<                            \
        ackMeta[ninst].qpn << std::dec <<                                                 \
        "\tisNak:" << 0                                                                   \
        << std::endl;                                                                     \
}

 //std::cout << "[Memory]: Write data: " << std::hex                                      \
 //       << currWord.data << std::dec << std::endl;                                        \


 void test(int sendPkgNum, double networkLoss, double reorderRate, std::vector<double> &Times, std::vector<double> &goodputs){
    // testSimSwitch(8); // drop one packet for every 8; 0 means no drop

    // switch ports
    SWITCHPORT(0);
    SWITCHPORT(1);
    SWITCHPORT(2);
    SWITCHPORT(3);
    SWITCHPORT(4);
    SWITCHPORT(5);
    SWITCHPORT(6);
    SWITCHPORT(7);
    SWITCHPORT(8);
    SWITCHPORT(9);

    // interfaces
    IBTPORT(0);
    IBTPORT(1);

    // newFakeDRAM
    newFakeDRAM<DATA_WIDTH> memoryN0;
    newFakeDRAM<DATA_WIDTH> memoryN1;
    std::vector<bool> writeCmdReady {false, false};
    std::vector<memCmd> writeCmd(2);
    std::vector<memCmd> readCmd(2);
    std::vector<int> writeRemainLen(2);
    std::vector<ackMeta> ackMeta(2);

    // ipAddr
    ap_uint<128> ipAddrN0, ipAddrN1, ipAddrN2, ipAddrN3, ipAddrN4;
    ipAddrN0(127, 64) = 0xfe80000000000000;
    ipAddrN0(63, 0)   = 0x92e2baff0b01d4d2;
    ipAddrN1(127, 64) = 0xfe80000000000000;
    ipAddrN1(63, 0)   = 0x92e2baff0b01d4d3;
    ipAddrN2 = 0;
    ipAddrN3 = 0;
    ipAddrN4 = 0;


    // Create qp ctx
    qpContext ctxN00 = qpContext(READY_RECV, 0x00, 0xac701e, 0x2a19d6, 0, 0x00);
    qpContext ctxN01 = qpContext(READY_RECV, 0x01, 0x000000, 0x3a19d6, 0, 0x00);

    qpContext ctxN10 = qpContext(READY_RECV, 0x00, 0x3a19d6, 0x000000, 0, 0x00);
    qpContext ctxN11 = qpContext(READY_RECV, 0x01, 0x2a19d6, 0xac701e, 0, 0x00);
    
    ifConnReq connInfoN0 = ifConnReq(1, 0, ipAddrN1, 5000);
    ifConnReq connInfoN1 = ifConnReq(0, 1, ipAddrN0, 5000);

    s_axis_qp_interface_n0.write(ctxN00);
    s_axis_qp_interface_n0.write(ctxN01);
    s_axis_qp_interface_n1.write(ctxN10);
    s_axis_qp_interface_n1.write(ctxN11);

    s_axis_qp_conn_interface_n0.write(connInfoN0);
    s_axis_qp_conn_interface_n1.write(connInfoN1);

    int count = 0;
    isrecv_n0 = false;
    
    isrecv_n1 = false;
    int cycle = 0;
    sendPkgNum_n0 = sendPkgNum;
    sendPkgNum_n1 = sendPkgNum_n0;
    //Make sure it is initialized
    while (count < 10)
    {
        IBTRUN(0);
        IBTRUN(1);
        count++;
    }

    // for (int i=0; i<1; i++) {
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_FIRST, 0x01, 0, 0, 0, params));
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_MIDDLE, 0x01, 0, 0, 0, params));
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_MIDDLE, 0x01, 0, 0, 0, params));
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_LAST, 0x01, 0, 1, 0, params));

        //s_axis_sq_meta_n0.write(txMeta(RC_SEND_ONLY, 0x01, 0, 0, 0, params));
        // s_axis_sq_meta_n0.write(txMeta(RC_SEND_FIRST, 0x01, 0, 0, 0, params));
        // s_axis_sq_meta_n0.write(txMeta(RC_SEND_MIDDLE, 0x01, 0, 0, 0, params));
        // s_axis_sq_meta_n0.write(txMeta(RC_SEND_MIDDLE, 0x01, 0, 0, 0, params));
        // s_axis_sq_meta_n0.write(txMeta(RC_SEND_LAST, 0x01, 0, 1, 0, params));

        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_READ_REQUEST, 0x01, 0, 0, 0, 0x400, 0x500, 1 * 1024, 0));
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_FIRST, 0x01, 0, 0, 0, 0x400, 0x500, PMTU, 0));
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_MIDDLE, 0x01, 0, 0, 0, 0x400+PMTU, 0x500+PMTU, PMTU, 0));
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_MIDDLE, 0x01, 0, 0, 0, 0x400+2*PMTU, 0x500+2*PMTU, PMTU, 0));
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_LAST, 0x01, 0, 0, 0, 0x400+3*PMTU, 0x500+3*PMTU, PMTU, 0));
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_READ_REQUEST, 0x01, 0, 0, 1, params));
        // s_axis_sq_meta_n0.write(txMeta(RC_RDMA_READ_REQUEST, 0x01, 0, 1, 2, params));
    // }
    s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_FIRST, 0x01, 0, 0, 0, 0x400, 0x500, PMTU, 0));
    int i=1;
    for(i;i<sendPkgNum_n0-1;i++){
        s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_MIDDLE, 0x01, 0, 0, 0, 0x400+i*PMTU, 0x500+i*PMTU, PMTU, 0));
    }
    s_axis_sq_meta_n0.write(txMeta(RC_RDMA_WRITE_LAST, 0x01, 0, 0, 0, 0x400+i*PMTU, 0x500+i*PMTU, PMTU, 0));
    bool first = true;
    int end_count = 0;
    int end_cycle;
    // int t = 5000*2;
    int t = 16000;
    int wordBytes = 4;
    // int disOrderCont = 0;

    // while (count < 200000)
    // {
    //     IBTRUN(0);
    //     IBTRUN(1);
    //     SWITCHRUN(0.3);
    //     DRAMRUN(0);
    //     DRAMRUN(1);
    //     count++;
    //     if(isrecv_n1 && first){
    //         cycle = count;
    //         first = false;
    //     }
    // }
    while (count < 4000000)
    {
        if(!first){
            break;
        }
        for(int i=0;i<=t;i++){
            IBTRUN(0);
            DRAMRUN(0);
            count++;
            end_count++; 
        }
        for(int i=0;i<=10;i++){
            SWITCHRUNSEND(0,2,4,6,8,0);
            SWITCHRUN(2,3,0);
            SWITCHRUN(4,5,networkLoss);
            SWITCHRUN(6,7,networkLoss);
            SWITCHRUN(8,9,0); 
            SWITCHRUNRECV(3,1,5,7,9,0,reorderRate);           
        }
        for(int i=0;i<t;++i){
            SWITCHRUNSEND(0,2,4,6,8,0);
            SWITCHRUN(2,3,0);
            SWITCHRUN(4,5,networkLoss);
            SWITCHRUN(6,7,networkLoss);
            SWITCHRUN(8,9,0); 
        }
        for(int i=0;i<t;++i){
            SWITCHRUNRECV(3,1,5,7,9,0,reorderRate); 
        }
        std::cout<<"---------------发送方数据到达------------"<<std::endl;
        for(int i=0;i<=t;i++){
            IBTRUN(1);
            DRAMRUN(1);
            count++;
            // end_count++; 
            if(isrecv_n1 && first){
                cycle = count;
                end_cycle = end_count;
                first = false;
            }
        }

        // for(int i=0;i<=t;i++){
        //     SWITCHRUN(networkLoss);
        // }
        std::cout<<"---------------接收方反馈信息到达------------"<<std::endl;
        
        
        // DRAMRUN(0);
        // DRAMRUN(1);
       
    }
    double PCIE_latency = 0.1;
    double end_Time = cycle*(3.2)/1000; //单位微秒
    std::cout<<"cycle:"<<std::dec<<cycle<<std::endl;
    std::cout<<"传输时延:"<<std::dec<<end_Time/8<<"微秒"<<std::endl;
    Times.push_back(end_Time/8);
    std::cout<<"goodput:"<<std::dec<<8*sendPkgNum_n0/((end_cycle*(3.2)/1000)/8)<<"Gbps"<<std::endl;
    goodputs.push_back(8*sendPkgNum_n0/((end_cycle*(3.2)/1000)/8));
    std::cout<<"接收包乱序数目:"<<disOrderCount_n1<<std::endl;
}
int main(int argc, char* argv[]){
    // test(10,0.1);
    std::vector<double> Times;
    std::vector<double> goodputs;
    int k = 1;  //循环次数,测量平均值
    double p_loss = 0.01;  //网络丢包率
    double reorder = 0.1;
    int pkgNum = ALLPKGNUM;  //数据包数目,每个数据包1KB.
    for(int i=1;i<=k;i++){
        test(pkgNum,p_loss,reorder,Times,goodputs);
    }
    
    double Times_Sum = std::accumulate(Times.begin(),Times.end(),0.0);
    std::cout<<k<<"次实验平均传输时延:"<<Times_Sum/k<<"微秒"<<std::endl;
    std::vector<double>::iterator Times_max_iter = std::max_element(Times.begin(),Times.end());
    double Times_max = *Times_max_iter;
    std::cout<<"最长时延:"<<Times_max<<std::endl;
    double goodputs_Sum = std::accumulate(goodputs.begin(),goodputs.end(),0.0);
    std::cout<<k<<"次试验平均有效吞吐量为"<<goodputs_Sum/k<<" Gbps"<<std::endl;
    std::cout<<Times_Sum/k<<"\t"<<goodputs_Sum/k<<std::endl;

    return 0;
}
