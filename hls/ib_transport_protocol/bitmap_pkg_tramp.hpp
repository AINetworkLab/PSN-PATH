#pragma once

#include "../axi_utils.hpp"
#include "ib_transport_protocol.hpp"
#include "rocev2_config.hpp"
#define BITMAP_SIZE 128

struct bitMapRow
{
    ap_uint<24> epsn;  
    ap_uint<BITMAP_SIZE> bitState;   //访问某个比特使用bitState[],访问某个区间[x,y]的比特使用bitState(x,y)
    ap_uint<24> pre_psn;  //上一个包，若psn!=pre_psn+1则触发重传;
    bitMapRow(){}
    bitMapRow(ap_uint<24> epsn):epsn(epsn), bitState(0){}
};
// struct initBitmapRow
// {
//     ap_uint<16> qpn;
//     ap_uint<24> psn;
//     initBitmapRow(){}
//     initBitmapRow(ap_uint<16> qpn, ap_uint<24> psn):qpn(qpn), psn(psn){}
// };
struct updateBitmap
{
    ap_uint<16> qpn;
    ap_uint<24> psn;
    ibOpCode op_code;
    updateBitmap(){}
    updateBitmap( ap_uint<16> qpn, ap_uint<24> psn, ibOpCode op_code):qpn(qpn),psn(psn),op_code(op_code){}
};


template <int INSTID>
void bitmap_pkg_tramp(hls::stream<updateBitmap>& initBitmapRowFifo,
                hls::stream<updateBitmap>& updateBitmapFifo,
                hls::stream<retrans>& retransFifo,
                hls::stream<bool>& isRecvFifo,
                int& disOrderCount);

template <int INSTID>
void bitmap_pkg_tramp(hls::stream<updateBitmap>& initBitmapRowFifo,
                hls::stream<updateBitmap>& updateBitmapFifo,
                hls::stream<retrans>& retransFifo,  //重传信息
                hls::stream<bool>& isRecvFifo,  //是否接收当前数据，true表示接收
                int& disOrderCount
            )
{
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off
    static bitMapRow bitmap[MAX_QPS];
    updateBitmap init_bitmap;
    updateBitmap update_bitmap;
    // static int disOrderCount = 0;
    if(!initBitmapRowFifo.empty()){
        initBitmapRowFifo.read(init_bitmap);
        bitmap[init_bitmap.qpn].pre_psn=init_bitmap.psn;
        bitmap[init_bitmap.qpn].epsn=init_bitmap.psn+1;
        bitmap[init_bitmap.qpn].bitState=0;
        isRecvFifo.write(true);
        disOrderCount = 0;
    }
    else if(!updateBitmapFifo.empty()){
        updateBitmapFifo.read(update_bitmap);
        if(bitmap[update_bitmap.qpn].pre_psn + 1==update_bitmap.psn){  //顺序接收
            bitmap[update_bitmap.qpn].pre_psn = update_bitmap.psn;
            if(bitmap[update_bitmap.qpn].epsn==update_bitmap.psn){   //并且之前从未丢包
                bitmap[update_bitmap.qpn].epsn += 1;
                isRecvFifo.write(true);
            }else{  //发生过丢包，更新位图
                if(update_bitmap.psn-bitmap[update_bitmap.qpn].epsn<BITMAP_SIZE){  //位图尚未溢出
                    bitmap[update_bitmap.qpn].bitState[update_bitmap.psn-bitmap[update_bitmap.qpn].epsn]=1;
                    isRecvFifo.write(true);
                }
                else{
                    bitmap[update_bitmap.qpn].pre_psn-=1; //不记录位图溢出时的包
                    retransFifo.write(retrans(update_bitmap.qpn,update_bitmap.psn));  //重传当前包
                    isRecvFifo.write(false);
                }
                // disOrderCount += 1;
            }
        }else{  //乱序
            disOrderCount += 1;
            // disOrderCountFifo.write(disOrderCount);
            if(update_bitmap.psn<bitmap[update_bitmap.qpn].epsn || update_bitmap.op_code < 0x06 || update_bitmap.op_code > 0x12){
                std::cout<<"当前收到的数据包为:"<<std::hex<<update_bitmap.psn<<std::endl;
                std::cout<<"EPSN:"<<std::hex<<bitmap[update_bitmap.qpn].epsn<<std::endl;
                isRecvFifo.write(false);  //重复到达的包，不接收
            }
            else if(bitmap[update_bitmap.qpn].epsn==update_bitmap.psn){  //此时到达的是重传的丢失包
                std::cout<<"收到了重传的丢失包"<<std::endl;
                bitmap[update_bitmap.qpn].bitState[0]=1;
                isRecvFifo.write(true);
                //重新检索位图找到新的epsn
                ap_uint<16> select_i = BITMAP_SIZE;
                for(ap_uint<16> i=0;i<BITMAP_SIZE;i++){
                    #pragma HLS unroll
                    bool cond = bitmap[update_bitmap.qpn].bitState[i]==0;
                    if(cond && select_i > i){
                        select_i=i;
                    }
                }
                std::cout<<"旧epsn:"<<bitmap[update_bitmap.qpn].epsn<<std::endl;
                std::cout<<"更新前的位图:"<<bitmap[update_bitmap.qpn].bitState<<std::endl;
                bitmap[update_bitmap.qpn].bitState=bitmap[update_bitmap.qpn].bitState>>select_i;
                std::cout<<"右移: "<<select_i<<"更新后的位图:"<<bitmap[update_bitmap.qpn].bitState<<std::endl;
                bitmap[update_bitmap.qpn].epsn+=select_i;
                std::cout<<"新epsn:"<<bitmap[update_bitmap.qpn].epsn<<std::endl;
                //此时不必更新pre_psn
            }else if(update_bitmap.psn<bitmap[update_bitmap.qpn].pre_psn){  //此时是重传的丢失包，但不是epsn
                isRecvFifo.write(true);
                bitmap[update_bitmap.qpn].bitState[update_bitmap.psn-bitmap[update_bitmap.qpn].epsn]=1;
                std::cout<<"收到了重传的丢失包"<<std::endl;
                // if(BITMAP_SIZE<=128){
                //     retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].bitState,bitmap[update_bitmap.qpn].epsn,update_bitmap.psn));
                // }
                // else{
                //     retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].bitState(0,126),bitmap[update_bitmap.qpn].epsn,update_bitmap.psn));
                // }
                
            }else{  //新包，此时发生了丢包
                if(update_bitmap.psn - bitmap[update_bitmap.qpn].epsn >= BITMAP_SIZE){  //当前包溢出
                    std::cout<<"溢出"<<std::endl;
                    isRecvFifo.write(false);
                    if(bitmap[update_bitmap.qpn].pre_psn - bitmap[update_bitmap.qpn].epsn!=BITMAP_SIZE-1){  // 位图需要更新
                        ap_uint<24> old_psn = bitmap[update_bitmap.qpn].pre_psn;
                        for(ap_uint<24> i=bitmap[update_bitmap.qpn].pre_psn+1-bitmap[update_bitmap.qpn].epsn;i<BITMAP_SIZE;i++){
                            #pragma HLS unroll
                            bitmap[update_bitmap.qpn].bitState[i]=0;
                            // if(BITMAP_SIZE>128){
                                // retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].epsn+i));
                            // }
                        }
                        retransFifo.write(retrans(update_bitmap.qpn,update_bitmap.psn));
                        if(BITMAP_SIZE<=128){
                            retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].bitState,bitmap[update_bitmap.qpn].epsn,bitmap[update_bitmap.qpn].pre_psn));
                        }
                        else{
                            retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].bitState(BITMAP_SIZE-128,BITMAP_SIZE-1),old_psn,bitmap[update_bitmap.qpn].pre_psn));
                        }
                        
                        // retransFifo.write(retran(update_bitmap.qpn,bitmap[update_bitmap.qpn].bitState,bitmap[update_bitmap.qpn].epsn,update_bitmap.psn));
                        bitmap[update_bitmap.qpn].pre_psn=bitmap[update_bitmap.qpn].epsn+BITMAP_SIZE-1;
                    }else{  //位图无需更新
                        retransFifo.write(retrans(update_bitmap.qpn,update_bitmap.psn));
                    }
                }else{  //当前包未溢出
                    isRecvFifo.write(true);
                    bitmap[update_bitmap.qpn].bitState[update_bitmap.psn - bitmap[update_bitmap.qpn].epsn]=1;
                    for(ap_uint<24> i=bitmap[update_bitmap.qpn].pre_psn+1-bitmap[update_bitmap.qpn].epsn;i<update_bitmap.psn - bitmap[update_bitmap.qpn].epsn&&i<128;i++){
                        #pragma HLS unroll
                        bitmap[update_bitmap.qpn].bitState[i]=0;
                        // if(BITMAP_SIZE>128){
                        //     retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].epsn+i));
                        // }
                    }
                    // if(BITMAP_SIZE<=128){
                    //     retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].bitState,bitmap[update_bitmap.qpn].epsn,update_bitmap.psn));
                    // }
                    // else{
                    //     retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].bitState(bitmap[update_bitmap.qpn].pre_psn-bitmap[update_bitmap.qpn].epsn,update_bitmap.psn-bitmap[update_bitmap.qpn].epsn),bitmap[update_bitmap.qpn].pre_psn,update_bitmap.psn));
                    // }
                    
                    bitmap[update_bitmap.qpn].pre_psn=update_bitmap.psn;
                    std::cout<<"位图:\t"<<bitmap[update_bitmap.qpn].bitState<<std::endl;
                }
            }  
        }
    }
}
//位图未溢出时SAK携带整个位图;溢出时回退到GBN，来一个丢一个重传一个

//网络具备一定的存储能力