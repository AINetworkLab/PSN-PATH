#pragma once

#include "../axi_utils.hpp"
#include "ib_transport_protocol.hpp"
#include "rocev2_config.hpp"
#define BITMAP_SIZE 1024
#define PATH 4
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
void bitmap_pkg_psn(hls::stream<updateBitmap>& initBitmapRowFifo,
                hls::stream<updateBitmap>& updateBitmapFifo,
                hls::stream<retrans>& retransFifo,
                hls::stream<bool>& isRecvFifo,
                int& disOrderCount);

template <int INSTID>
void bitmap_pkg_psn(hls::stream<updateBitmap>& initBitmapRowFifo,
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
    else if(!updateBitmapFifo.empty()){  //新颖的位图检测,发送方根据psn将不同的数据包发送到不同路径上,接收方位图根据psn识别不同路径上的数据包是否顺序.
        updateBitmapFifo.read(update_bitmap);
        if(update_bitmap.psn < bitmap[update_bitmap.qpn].epsn || update_bitmap.op_code < 0x06 || update_bitmap.op_code > 0x12){//重复到达的数据包或是错误包,不接收该数据包
            isRecvFifo.write(false);
        }
        else if(update_bitmap.psn == bitmap[update_bitmap.qpn].epsn){//全局顺序包,位图前移
            isRecvFifo.write(true);
            ap_uint<16> select_i = BITMAP_SIZE;
            for (ap_uint<16> i = 1; i < BITMAP_SIZE; i++)
            {
#pragma HLS unroll
                bool cond = bitmap[update_bitmap.qpn].bitState[i] == 0;
                if (cond && select_i > i)
                {
                    select_i = i;
                }
            }
            // std::cout << "旧epsn:" << bitmap[update_bitmap.qpn].epsn << std::endl;
            // std::cout << "更新前的位图:" << bitmap[update_bitmap.qpn].bitState << std::endl;
            bitmap[update_bitmap.qpn].bitState = bitmap[update_bitmap.qpn].bitState >> select_i;
            // std::cout << "右移: " << select_i << "更新后的位图:" << bitmap[update_bitmap.qpn].bitState << std::endl;
            bitmap[update_bitmap.qpn].epsn += select_i;
            // std::cout << "新epsn:" << bitmap[update_bitmap.qpn].epsn << std::endl;
        }
        else if(update_bitmap.psn - bitmap[update_bitmap.qpn].epsn >= BITMAP_SIZE){//位图溢出,不接收该数据包,回发重传信号
            isRecvFifo.write(false);
            retransFifo.write(retrans(update_bitmap.qpn,update_bitmap.psn));
            disOrderCount += 1;
        }
        else if(update_bitmap.psn - bitmap[update_bitmap.qpn].epsn < BITMAP_SIZE){//位图未溢出,全局乱序,进一步检测路径内是否乱序
            disOrderCount += 1;
            isRecvFifo.write(true);
            bitmap[update_bitmap.qpn].bitState[update_bitmap.psn - bitmap[update_bitmap.qpn].epsn] = 1;
            if(update_bitmap.psn - PATH >= bitmap[update_bitmap.qpn].epsn && bitmap[update_bitmap.qpn].bitState[update_bitmap.psn - PATH - bitmap[update_bitmap.qpn].epsn]==0){
                //发生丢包,检索位图,回发重传信号
                ap_uint<16> qpn = update_bitmap.qpn;
                ap_int<16> hi = (ap_int<16>)(update_bitmap.psn - PATH - bitmap[qpn].epsn);
                // 1) 先从 hi 往下找：定位连续丢包段的起点 low（不写 FIFO）
                ap_int<16> low = hi;
                for (ap_int<16> step = 0; step < (BITMAP_SIZE / PATH); ++step)
                {
#pragma HLS PIPELINE II = 1
                    if (low < 0)
                        break;
                    if (bitmap[qpn].bitState[low] == 0)
                        low -= PATH; // 仍在连续丢包段内
                    else
                        break; // 碰到已到达包，停止
                }
                low += PATH; // 回到“第一个丢包”的索引（若 hi 本身不丢包，则 low=hi+PATH，下面循环不会执行）

                // 2) 再从 low 往上写：保证 retrans 按 PSN 从小到大输出
                for (ap_int<16> i = low; i <= hi; i += PATH)
                {
#pragma HLS PIPELINE II = 1
                    retransFifo.write(retrans(qpn, bitmap[qpn].epsn + (ap_uint<16>)i));
                }
            }
            
        }
    }
}