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
	// ns3::NodeContainer nodes = ns3::NodeContainer::GetGlobal();
	NS_LOG_DEBUG("Num of Nodes "<<m_nodes.GetN());
	m_probtime = 0.1;
	num = ns3::NodeContainer::GetGlobal().GetN();
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
		auto nextHop = nexthops.begin();
		ns3::Ptr<ns3::Node> nextNode;
		double distance = std::numeric_limits<double>::max();

		for (auto hop = nexthops.begin(); hop != nexthops.end(); ++hop) {
			// NS_LOG_INFO("prefix: " << fibEntry.getPrefix() << ", "
			//                        << "fib: " << hop->getFace().getId());

			if(isNextHopEligible(ingress.face, interest,  *hop, pitEntry)){
				const auto transport = hop->getFace().getTransport();
				ns3::ndn::WifiNetDeviceTransport* wifiTrans =
					dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport); 
				ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
				ns3::Ptr<ns3::MobilityModel> mobModel = node->GetObject<ns3::MobilityModel>();
				if (mobModel == nullptr) {
					NS_LOG_DEBUG("There is no mobility model in "<<node->GetId());
				}
				ns3::Vector3D nodePos = mobModel->GetPosition();
				std::string remoteUri =  transport->getRemoteUri().getHost();
				// NS_LOG_DEBUG("Transport: "<<transport->getRemoteUri().getHost());
				ns3::Ptr<ns3::Channel> channel = wifiTrans->GetNetDevice()->GetChannel();
				for(uint32_t deviceId = 0; deviceId < channel->GetNDevices(); ++deviceId){
					ns3::Address address = channel->GetDevice(deviceId)->GetAddress();
					// NS_LOG_DEBUG(remoteUri =ns3::Mac48Address::ConvertFrom(address));
					std::string uri = boost::lexical_cast<std::string>(ns3::Mac48Address::ConvertFrom(address));
					// NS_LOG_DEBUG(uri);
					if(remoteUri == uri){
						ns3::Ptr<ns3::Node> remoteNode = channel->GetDevice(deviceId)->GetNode();
						ns3::Ptr<ns3::MobilityModel> mobModel = remoteNode->GetObject<ns3::MobilityModel>();
						if (mobModel == nullptr) {
							NS_LOG_DEBUG("There is no mobility model in "<<remoteNode->GetId());
						}
						ns3::Vector3D remotePos = mobModel->GetPosition();
						double newDistance = sqrt(std::pow((nodePos.x-remotePos.x), 2) + std::pow((nodePos.y-remotePos.y), 2));
						if (newDistance < distance) {
							distance = newDistance;
							nextHop = hop;
							nextNode = remoteNode;
						}
						// NS_LOG_INFO("prefix: " << fibEntry.getPrefix()
						// 							<< ", Face: " << hop->getFace().getId()
						// 							<<", Node: "<<remoteNode->GetId()
						// 							<<", Distance: "<<newDistance);
					}
				}
			}
		}

		if (nextHop == nexthops.end()) {
			NFD_LOG_DEBUG(interest << " from=" << ingress << " noNextHop");
			lp::NackHeader nackHeader;
			nackHeader.setReason(lp::NackReason::NO_ROUTE);
			this->sendNack(pitEntry, ingress, nackHeader);
			this->rejectPendingInterest(pitEntry);
			return;
		}

		auto egress = FaceEndpoint(nextHop->getFace(), 0);
		NS_LOG_DEBUG(interest << " from=" << ingress.face.getId()<<" to="<<nextHop->getFace().getId()
									<<" Next="<<nextNode->GetId()<< " Distance=" << distance);
		// NFD_LOG_DEBUG(interest << " from=" << ingress << " newPitEntry-to=" << egress);
		this->sendInterest(pitEntry, egress, interest);
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


void LSF::initialPosition(){
	std::vector<std::vector<ns3::Vector3D>> posVector(num, std::vector<ns3::Vector3D>(num, ns3::Vector3D(0, 0, 0)));
	std::vector<std::vector<ns3::Vector3D>> volVector(num, std::vector<ns3::Vector3D>(num, ns3::Vector3D(0, 0, 0)));
	m_posMap = posVector;
	m_volMap = volVector;
}


void LSF::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                           const shared_ptr<pit::Entry>& pitEntry) {
    this->processNack(ingress.face, nack, pitEntry);
}

}  // namespace fw
}  // namespace nfd
