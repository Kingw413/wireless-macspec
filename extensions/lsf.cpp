#include "lsf.hpp"
#include "common/logger.hpp"
#include "ndn-wifi-net-device-transport.hpp"
#include "ns3/mobility-model.h"
#include "ns3/ndnSIM/NFD/daemon/fw/algorithm.hpp"
#include "ns3/ndnSIM/model/ndn-net-device-transport.hpp"
#include "ns3/node.h"
#include "ns3/ptr.h"
#include "ns3/wifi-net-device.h"
#include "ns3/vector.h"
#include "ns3/node-container.h"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/NFD/daemon/table/strategy-choice-entry.hpp"
#include "ns3/ndnSIM/NFD/daemon/table/strategy-choice.hpp"
#include "core/common.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
namespace nfd {
namespace fw {

NFD_LOG_INIT(LSF);
NFD_REGISTER_STRATEGY(LSF);

const time::milliseconds LSF::RETX_SUPPRESSION_INITIAL(10);
const time::milliseconds LSF::RETX_SUPPRESSION_MAX(250);

LSF::LSF(Forwarder& forwarder, const Name& name)
    : Strategy(forwarder),
      ProcessNackTraits(this),
    //   fw(forwarder),
      m_retxSuppression(RETX_SUPPRESSION_INITIAL,
                        RetxSuppressionExponential::DEFAULT_MULTIPLIER,
                        RETX_SUPPRESSION_MAX)
		{
    ParsedInstanceName parsed = parseInstanceName(name);
    if (!parsed.parameters.empty()) {
        NDN_THROW(std::invalid_argument("LSF does not accept parameters"));
    }
    if (parsed.version &&
        *parsed.version != getStrategyName()[-1].toVersion()) {
        NDN_THROW(std::invalid_argument("LSF does not support version " +
                                        to_string(*parsed.version)));
    }
    this->setInstanceName(makeInstanceName(name, getStrategyName()));
	m_nodes = ns3::NodeContainer::GetGlobal();
	m_probtime = 0.1;
	num = m_nodes.GetN();
	m_isr = std::vector<std::map<std::string, std::vector<int>>>(num);
	initialPosition();
	ns3::Simulator::Schedule(ns3::Seconds(0.0), &LSF::sendPosition, this);
}

const Name& LSF::getStrategyName() {
    static Name strategyName("/localhost/nfd/strategy/lsf/%FD%10");
    return strategyName;
}

void LSF::afterReceiveInterest(const FaceEndpoint& ingress,
                               const Interest& interest,
                               const shared_ptr<pit::Entry>& pitEntry)						   
{	
    if(interest.getName().getPrefix(1) == "PosInfo"){
		NS_LOG_INFO(interest);
		updatePos(ingress, interest);
		return;
	}

	else{
		RetxSuppressionResult suppression =
			m_retxSuppression.decidePerPitEntry(*pitEntry);
		if (suppression == RetxSuppressionResult::SUPPRESS) {
			NFD_LOG_DEBUG(interest << " from=" << ingress << " suppressed");
			return;
		}

		const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
		const fib::NextHopList& nexthops = fibEntry.getNextHops();
		auto bestNextHop = getBestNextHop(nexthops, ingress, interest, pitEntry);
		if (bestNextHop == nexthops.end()) {
			NFD_LOG_DEBUG(interest << " from=" << ingress << " noNextHop");
			lp::NackHeader nackHeader;
			nackHeader.setReason(lp::NackReason::NO_ROUTE);
			this->sendNack(pitEntry, ingress, nackHeader);
			this->rejectPendingInterest(pitEntry);
			return;
		}

		auto egress = FaceEndpoint(bestNextHop->getFace(), 0);
		NS_LOG_DEBUG(interest << " from=" << ingress<<" to="<<egress
												<< " cost=" << bestNextHop->getCost());
		this->sendInterest(pitEntry, egress, interest);
		this->updateISR(ingress, interest, pitEntry, 0);
		return;
	}
}


void LSF::sendPosition(){
	for(uint32_t nodeId = 0; nodeId<m_nodes.GetN(); ++nodeId){
		ns3::Ptr<ns3::Node> node = m_nodes.Get(nodeId);
 		ns3::Ptr<ns3::ndn::L3Protocol> ndn = ns3::ndn::L3Protocol::getL3Protocol(m_nodes.Get(nodeId));
		const fib::Entry* fibEntry = ndn->getForwarder()->getFib().findExactMatch("/");
		const fib::NextHopList& nexthops = fibEntry->getNextHops();
		ns3::Ptr<ns3::MobilityModel> mobModel = node->GetObject<ns3::MobilityModel>();
		if (mobModel == nullptr) {
			NS_LOG_DEBUG("There is no mobility model in " << node->GetId());
		}
		ns3::Vector3D pos = mobModel->GetPosition();
		ns3::Vector3D speed = mobModel->GetVelocity();
		std::string name = "/PosInfo/Node" +to_string(node->GetId()) + "/posx" + to_string(pos.x) + "/posy" +to_string(pos.y) +"/volx" + to_string(speed.x) +"/voly" +to_string(speed.y);
		// NS_LOG_DEBUG(name);
		Interest pkt;
		pkt.setName(name);
		for (auto hop = nexthops.begin(); hop != nexthops.end(); ++hop) {
				auto egress = FaceEndpoint(hop->getFace(), 0);
				egress.face.sendInterest(pkt, egress.endpoint);
		}
	}
	ns3::Simulator::Schedule(ns3::Seconds(m_probtime), &LSF::sendPosition,this);
}


void LSF::updatePos(const FaceEndpoint& ingress, const Interest& interest){
	std::string  name = interest.getName().toUri();
	double pos_x= stod(name.substr(name.find("posx")+4, 5)), pos_y = stod(name.substr(name.find("posy")+4, 5)), 
				 vol_x = stod(name.substr(name.find("volx")+4, 5)), vol_y = stod(name.substr(name.find("voly")+4, 5));
	uint32_t neighNode = std::stoi(name.substr(name.find("Node")+4,1));
    const auto transport = ingress.face.getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
    ns3::Ptr<ns3::Node> localNode = wifiTrans->GetNetDevice()->GetNode();
	ns3::Vector3D pos = ns3::Vector3D(pos_x, pos_y, 0.0);
	ns3::Vector3D vol = ns3::Vector3D(vol_x, vol_y, 0.0);
	m_posMap[localNode->GetId()][neighNode] = pos;
	m_volMap[localNode->GetId()][neighNode] = vol;
	NS_LOG_INFO("Update Pos and Vel between "<<localNode->GetId()<<"-"<<neighNode<<": ("<<pos<<"), ("<<vol<<")");
}

void LSF::updateISR(const FaceEndpoint& egress,
                               const Interest& interest,
                               const shared_ptr<pit::Entry>& pitEntry,
							   int type)
{	NS_LOG_DEBUG(egress.face.getId());
	if(egress.face.getId() ==260) {return;};
	std::string name = interest.getName().toUri();
	NS_LOG_DEBUG(name);
    const auto transport = egress.face.getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
    ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
	auto a = m_isr[node->GetId()];
	if(a.find(name) == a.end()){
		m_isr[node->GetId()][name]={0,0};
	}
	if(type == 0){
		m_isr[node->GetId()][name][0] += 1;
	}
	else{
		m_isr[node->GetId()][name][1] += 1;
	}
}

void LSF::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data)
{
  NFD_LOG_DEBUG("afterReceiveData pitEntry=" << pitEntry->getName()
                << " in=" << ingress << " data=" << data.getName());
  Interest interest = pitEntry->getInterest();
  this->beforeSatisfyInterest(pitEntry, ingress, data);

  this->sendDataToAll(pitEntry, ingress, data);

  this->updateISR(ingress, interest, pitEntry, 1);
}

void LSF::initialPosition(){
	std::vector<std::vector<ns3::Vector3D>> posVector(num, std::vector<ns3::Vector3D>(num, ns3::Vector3D(0, 0, 0)));
	std::vector<std::vector<ns3::Vector3D>> volVector(num, std::vector<ns3::Vector3D>(num, ns3::Vector3D(0, 0, 0)));
	m_posMap = posVector;
	m_volMap = volVector;
}


const fib::NextHopList&
LSF::caculateHopProb(const fib::NextHopList& nexthops){
	// const fib::NextHopList& nexthops = fibEntry.getNextHops();
	// auto nextHop = nexthops.begin();
    const auto transport = nexthops.begin()->getFace().getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans =
        dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
	ns3::Ptr<ns3::Channel> channel = wifiTrans->GetNetDevice()->GetChannel();
    ns3::Ptr<ns3::Node> localNode = wifiTrans->GetNetDevice()->GetNode();
    ns3::Ptr<ns3::Node> remoteNode;
	double distance = std::numeric_limits<double>::max();
	for(auto hop = nexthops.begin(); hop != nexthops.end(); ++hop){
		std::string remoteUri = hop->getFace().getRemoteUri().getHost();
		for(uint32_t deviceId = 0; deviceId < channel->GetNDevices(); ++deviceId){
			ns3::Address address = channel->GetDevice(deviceId)->GetAddress();
			std::string uri = boost::lexical_cast<std::string>(ns3::Mac48Address::ConvertFrom(address));
			if(remoteUri == uri){
				remoteNode = channel->GetDevice(deviceId)->GetNode();	
				ns3::Vector3D localPos = m_posMap[remoteNode->GetId()][localNode->GetId()];
				ns3::Vector3D remotePos = m_posMap[localNode->GetId()][remoteNode->GetId()];
				double newDistance = sqrt(std::pow((localPos.x-remotePos.x), 2) + std::pow((localPos.y-remotePos.y), 2));
				auto nexthop = *hop;
				nexthop.setCost(newDistance);
			}
		}
	}
	return nexthops;
}


nfd::fib::NextHopList::const_iterator 
LSF::getBestNextHop(const fib::NextHopList& nexthops,
									const FaceEndpoint& ingress,
                               		const Interest& interest,
                               		const shared_ptr<pit::Entry>& pitEntry){
										
	this->caculateHopProb(nexthops);
	uint64_t cost =  std::numeric_limits<uint64_t>::max();
	auto besthop = nexthops.begin();
	for(auto nexthop = nexthops.begin(); nexthop != nexthops.end(); ++nexthop){
		if(isNextHopEligible(ingress.face, interest,  *nexthop, pitEntry)){
			if(nexthop->getCost() < cost){
				besthop = nexthop;
				cost = nexthop->getCost();
			}
		}
	}
	return besthop;
}


void LSF::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                           const shared_ptr<pit::Entry>& pitEntry) {
    this->processNack(ingress.face, nack, pitEntry);
}

}  // namespace fw
}  // namespace nfd
