#pragma once

#include "../axi_utils.hpp"
#include "ib_transport_protocol.hpp"
#include "rocev2_config.hpp"


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
    updateBitmap(){}
    updateBitmap( ap_uint<16> qpn, ap_uint<24> psn):qpn(qpn),psn(psn){}
};


template <int INSTID>
void bitmap_pkg(hls::stream<updateBitmap>& initBitmapRowFifo,
                hls::stream<updateBitmap>& updateBitmapFifo,
                hls::stream<retrans>& retransFifo,
                hls::stream<bool>& isRecvFifo);

template <int INSTID>
void bitmap_pkg(hls::stream<updateBitmap>& initBitmapRowFifo,
                hls::stream<updateBitmap>& updateBitmapFifo,
                hls::stream<retrans>& retransFifo,  //重传信息
                hls::stream<bool>& isRecvFifo  //是否接收当前数据，true表示接收
            )
{
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off
    static bitMapRow bitMap[MAX_QPS];
    updateBitmap init_bitmap;
    updateBitmap update_bitmap;
    if(!initBitmapRowFifo.empty()){
        initBitmapRow.read(init_bitmap);
        bitMap[init_bitmap.qpn].pre_psn=init_bitmap.psn;
        bitMap[init_bitmap.qpn].epsn=init_bitmap.psn+1;
        bitMap[init_bitmap.qpn].bitState=0;
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
                    retrans.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].epsn,update_bitmap.psn));  //重传当前包
                    isRecvFifo.write(false);
                }
            }
        }else{  //乱序
            if(update_bitmap.psn<bitmap[update_bitmap.qpn].epsn){
                isRecvFifo.write(false);  //重复到达的包，不接收
            }
            else if(bitmap[update_bitmap.qpn].epsn==update_bitmap.psn){  //此时到达的是重传的丢失包
                bitmap[update_bitmap.qpn].bitState[0]=1;
                isRecvFifo.write(true);
                //重新检索位图找到新的epsn
                ap_uint<8> select_i = BITMAP_SIZE;
                for(ap_uint<8> i=0;i<BITMAP_SIZE;i++){
                    #pragma HLS unroll
                    bool cond = bitmap[update_bitmap.qpn].bitState[0]==0;
                    if(cond && select_i > i){
                        select_i=i;
                    }
                }
                bitmap[update_bitmap.qpn].bitState=bitmap[update_bitmap.qpn].bitState<<select_i;
                bitmap[update_bitmap.qpn].epsn+=select_i;
                //此时不必更新pre_psn
            }else if(update_bitmap.psn<bitmap[update_bitmap.qpn].pre_psn){  //此时是重传的丢失包，但不是epsn
                isRecvFifo.write(true);
                bitmap[update_bitmap.qpn].bitState[update_bitmap.psn-bitmap[update_bitmap.qpn].epsn]=1;
            }else{  //新包，此时发生了丢包
                if(update_bitmap.psn - bitmap[update_bitmap.qpn].epsn >= BITMAP_SIZE){  //当前包溢出
                    isRecvFifo.write(false);
                    if(bitmap[update_bitmap.qpn].pre_psn!=BITMAP_SIZE-1){  // 位图需要更新
                        for(ap_uint<24> i=bitmap[update_bitmap.qpn].pre_psn+1-bitmap[update_bitmap.qpn].epsn;i<BITMAP_SIZE;i++){
                            #pragma HLS unroll
                            bitmap[update_bitmap.qpn].bitState[i]=0;
                            retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].epsn,i+bitmap[update_bitmap.qpn].epsn));
                        }
                        for(ap_uint<24> i=bitmap[update_bitmap.qpn].epsn+BITMAP_SIZE;i<=update_bitmap.psn;i++){
                            #pragma HLS unroll
                            retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].epsn,i));
                        }
                        // retransFifo.write(retran(update_bitmap.qpn,bitmap[update_bitmap.qpn].bitState,bitmap[update_bitmap.qpn].epsn,update_bitmap.psn));
                        bitmap[update_bitmap.qpn].pre_psn=BITMAP_SIZE-1;
                    }else{  //位图无需更新
                        retransFifo.write(retran(update_bitmap.qpn,bitmap[update_bitmap.qpn].epsn,update_bitmap.psn));
                    }
                }else{  //当前包未溢出
                    isRecvFifo.write(true);
                    bitmap[update_bitmap.qpn].bitState[update_bitmap.psn - bitmap[update_bitmap.qpn].epsn]=1;
                    for(ap_uint<24> i=bitmap[update_bitmap.qpn].pre_psn+1-bitmap[update_bitmap.qpn].epsn;i<update_bitmap.psn - bitmap[update_bitmap.qpn].epsn;i++){
                        #pragma HLS unroll
                        bitmap[update_bitmap.qpn].bitState[i]=0;
                        retransFifo.write(retrans(update_bitmap.qpn,bitmap[update_bitmap.qpn].epsn,i+bitmap[update_bitmap.qpn].epsn));
                    }
                    // retransFifo.write(retran(update_bitmap.qpn,bitmap[update_bitmap.qpn].bitState,bitmap[update_bitmap.qpn].epsn,update_bitmap.psn));
                    bitmap[update_bitmap.qpn].pre_psn=update_bitmap.psn;
                }
            }  
        }
    }
}
//重传时重传丢失包和epsn;位图溢出时NAK携带整个位图;

//网络具备一定的存储能力