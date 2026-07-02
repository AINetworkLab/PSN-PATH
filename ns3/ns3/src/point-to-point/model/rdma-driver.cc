#include "rdma-driver.h"

namespace ns3 {

/***********************
 * RdmaDriver
 **********************/
TypeId RdmaDriver::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaDriver")
		.SetParent<Object> ()
		.AddTraceSource ("QpComplete", "A qp completes.",
				MakeTraceSourceAccessor (&RdmaDriver::m_traceQpComplete))
		;
	return tid;
}

RdmaDriver::RdmaDriver(){
}

void RdmaDriver::Init(void){
	if (m_node == 0 || m_rdma == 0) {
		NS_FATAL_ERROR("RdmaDriver::Init requires both node and rdma hw to be set");
	}
	Ptr<Ipv4> ipv4 = m_node->GetObject<Ipv4> ();
	if (ipv4 == 0) {
		NS_FATAL_ERROR("RdmaDriver::Init cannot find Ipv4 object on node");
	}
	#if 0
	m_rdma->m_nic.resize(ipv4->GetNInterfaces());
	for (uint32_t i = 0; i < m_rdma->m_nic.size(); i++){
		m_rdma->m_nic[i] = CreateObject<RdmaQueuePairGroup>();
		// share the queue pair group with NIC
		if (ipv4->GetNetDevice(i)->IsQbb()){
			DynamicCast<QbbNetDevice>(ipv4->GetNetDevice(i))->m_rdmaEQ->m_qpGrp = m_rdma->m_nic[i];
		}
	}
	#endif
	for (uint32_t i = 0; i < m_node->GetNDevices(); i++){
		Ptr<NetDevice> netDev = m_node->GetDevice(i);
		if (netDev == 0) {
			NS_FATAL_ERROR("RdmaDriver::Init encountered null NetDevice");
		}
		Ptr<QbbNetDevice> dev = NULL;
		if (netDev->IsQbb())
			dev = DynamicCast<QbbNetDevice>(netDev);
		if (netDev->IsQbb() && dev == 0) {
			NS_FATAL_ERROR("RdmaDriver::Init failed to cast Qbb device");
		}
		m_rdma->m_nic.push_back(RdmaInterfaceMgr(dev));
		m_rdma->m_nic.back().qpGrp = CreateObject<RdmaQueuePairGroup>();
	}
	#if 0
	for (uint32_t i = 0; i < ipv4->GetNInterfaces (); i++){
		if (ipv4->GetNetDevice(i)->IsQbb() && ipv4->IsUp(i)){
			Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(ipv4->GetNetDevice(i));
			// add a new RdmaInterfaceMgr for this device
			m_rdma->m_nic.push_back(RdmaInterfaceMgr(dev));
			m_rdma->m_nic.back().qpGrp = CreateObject<RdmaQueuePairGroup>();
		}
	}
	#endif
	// RdmaHw do setup
	m_rdma->SetNode(m_node);
	m_rdma->Setup(MakeCallback(&RdmaDriver::QpComplete, this));
}

void RdmaDriver::SetNode(Ptr<Node> node){
	m_node = node;
}

void RdmaDriver::SetRdmaHw(Ptr<RdmaHw> rdma){
	m_rdma = rdma;
}

void RdmaDriver::AddQueuePair(uint64_t size, uint16_t pg, Ipv4Address sip, Ipv4Address dip, uint16_t sport, uint16_t dport, uint32_t win, uint64_t baseRtt, int32_t flow_id){
	m_rdma->AddQueuePair(size, pg, sip, dip, sport, dport, win, baseRtt, flow_id);
}

void RdmaDriver::QpComplete(Ptr<RdmaQueuePair> q){
	m_traceQpComplete(q);
}

} // namespace ns3
