#pragma once

#include "../axi_utils.hpp"
#include "rocev2_config.hpp"
#include "ib_transport_protocol.hpp"

struct psnAddrMap
{
    ap_uint<24> first_psn;
    ap_uint<64> local_addr;
    ap_uint<64> remote_addr;
    psnAddrMap(){}
    psnAddrMap(ap_uint<24> first_psn, ap_uint<64> local_addr,ap_uint<64> remote_addr):first_psn(first_psn),local_addr(local_addr),remote_addr(remote_addr){}
};

struct queryAddr
{
    ap_uint<24> qpn;
    ap_uint<24> psn;
    queryAddr(){}
    queryAddr(ap_uint<24> qpn,ap_uint<24> psn):qpn(qpn),psn(psn){}
};

struct initAddrTable  //该数据结构未携带first_psn因为首包psn信息需要再次查询，因此分开传输
{
    ap_uint<24> qpn;
    ap_uint<64> local_addr;
    ap_uint<64> remote_addr;
    initAddrTable(){}
    initAddrTable(ap_uint<24> qpn, ap_uint<64> local_addr,ap_uint<64> remote_addr)
        :qpn(qpn),local_addr(local_addr),remote_addr(remote_addr){}
};
struct addrInfo
{
    ap_uint<24> qpn;
    ap_uint<24> psn;
    ap_uint<64> local_addr;
    ap_uint<64> remote_addr;
    addrInfo(){}
    addrInfo(ap_uint<24> qpn, ap_uint<24> psn, ap_uint<64> local_addr, ap_uint<64> remote_addr):qpn(qpn),psn(psn),local_addr(local_addr),remote_addr(remote_addr){}
};

template <int INSTID>
void addr_table(hls::stream<initAddrTable>& initFifo,
                hls::stream<ap_uint<24>>& first_psnFifo,
                hls::stream<queryAddr>& queryFifo,
                hls::stream<addrInfo>& addrFifo);
                
template <int INSTID>
void addr_table(hls::stream<initAddrTable>& initFifo,
                hls::stream<ap_uint<24>>& first_psnFifo,
                hls::stream<queryAddr>& queryFifo,
                hls::stream<addrInfo>& addrFifo)
{
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off
    static psnAddrMap addrTable[MAX_QPS];
    initAddrTable initInfo;
    queryAddr query;
    ap_uint<24> first_psn;
    if(!initFifo.empty()&&!first_psnFifo.empty()){
        initFifo.read(initInfo);
        first_psnFifo.read(first_psn);
        addrTable[initInfo.qpn].first_psn = first_psn;
        addrTable[initInfo.qpn].local_addr = initInfo.local_addr;
        addrTable[initInfo.qpn].remote_addr = initInfo.remote_addr;
    }else if(!queryFifo.empty()){
        queryFifo.read(query);
        ap_uint<64> offs = (query.psn-addrTable[query.qpn].first_psn)*PMTU;
        addrFifo.write(addrInfo(query.qpn,query.psn,addrTable[query.qpn].local_addr+offs,addrTable[query.qpn].remote_addr+offs));
    }
}
