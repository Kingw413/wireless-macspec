#include "prfs.hpp"

#include "common/logger.hpp"
#include "ndn-wifi-net-device-transport.hpp"
#include "ns3/mobility-model.h"
#include "ns3/ndnSIM/NFD/daemon/fw/algorithm.hpp"
#include "ns3/ndnSIM/model/ndn-net-device-transport.hpp"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/ptr.h"
#include "ns3/wifi-net-device.h"
#include "ns3/ndnSIM/apps/ndn-producer.hpp"
#include <cmath>

namespace nfd {
namespace fw {

NFD_LOG_INIT(PRFS);
NFD_REGISTER_STRATEGY(PRFS);

const time::milliseconds PRFS::RETX_SUPPRESSION_INITIAL(10);
const time::milliseconds PRFS::RETX_SUPPRESSION_MAX(250);

PRFS::PRFS(Forwarder& forwarder, const Name& name)
    : Strategy(forwarder),
      ProcessNackTraits(this),
      fw(forwarder),
      m_Rth(100.0),
      m_nodes(ns3::NodeContainer::GetGlobal()),
	  m_num(m_nodes.GetN()),
      m_retxSuppression(RETX_SUPPRESSION_INITIAL,
                        RetxSuppressionExponential::DEFAULT_MULTIPLIER,
                        RETX_SUPPRESSION_MAX) {
    ParsedInstanceName parsed = parseInstanceName(name);
    if (!parsed.parameters.empty()) {
        NDN_THROW(std::invalid_argument("PRFS does not accept parameters"));
    }
    if (parsed.version &&
        *parsed.version != getStrategyName()[-1].toVersion()) {
        NDN_THROW(std::invalid_argument("PRFS does not support version " +
                                        to_string(*parsed.version)));
    }
    this->setInstanceName(makeInstanceName(name, getStrategyName()));
}

const Name& PRFS::getStrategyName() {
    static Name strategyName("/localhost/nfd/strategy/prfs/%FD%01");
    return strategyName;
}


void PRFS::afterReceiveInterest(const FaceEndpoint& ingress,
                               const Interest& interest,
                               const shared_ptr<pit::Entry>& pitEntry)
{
    RetxSuppressionResult suppression =
        m_retxSuppression.decidePerPitEntry(*pitEntry);
    if (suppression == RetxSuppressionResult::SUPPRESS) {
        NFD_LOG_DEBUG(interest << " from=" << ingress << " suppressed");
        return;
    }
    const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
    const fib::NextHopList& nexthops = fibEntry.getNextHops();

    const auto transport = ingress.face.getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
    if (wifiTrans == nullptr) {
        auto next_rd = getBestHop(nexthops, ingress, interest, pitEntry, true);
        if (next_rd == nexthops.end()) {NFD_LOG_DEBUG(interest << " from=" << ingress << " noRDHop");}
        else{
            auto egress1 = FaceEndpoint(next_rd->getFace(), 0);
            NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << "to=" << egress1);
            this->sendInterest(pitEntry, egress1, interest);

			const auto transport2 = next_rd->getFace().getTransport();
    		ns3::ndn::WifiNetDeviceTransport* wifiTrans2 = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport2);
			ns3::Ptr<ns3::Node> consumer = wifiTrans2->GetNetDevice()->GetNode();
			int next_nodeId = next_rd->getFace().getId()-257 + (consumer->GetId()+257<=next_rd->getFace().getId());
	    	this->updateHopList(consumer->GetId(),next_nodeId, interest);
        }
        auto next_rrd = getBestHop(nexthops, ingress, interest, pitEntry, false);
        if (next_rrd == nexthops.end()) {NFD_LOG_DEBUG(interest << " from=" << ingress << " noRRDHop");}
        else{
            auto egress2 = FaceEndpoint(next_rrd->getFace(), 0);
            NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << "to=" << egress2);
            this->sendInterest(pitEntry, egress2, interest);

			const auto transport2 = next_rd->getFace().getTransport();
    		ns3::ndn::WifiNetDeviceTransport* wifiTrans2 = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport2);
			ns3::Ptr<ns3::Node> consumer = wifiTrans2->GetNetDevice()->GetNode();
			int next_nodeId = next_rd->getFace().getId()-257 + (consumer->GetId()+257<=next_rd->getFace().getId());
	    	this->updateHopList(consumer->GetId(),next_nodeId, interest);
        }
	    return;
    }

    ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
    int pre_node = (ingress.face.getId() - 257) + (node->GetId()+257 <= ingress.face.getId());
    auto nextHop = getBestHop(nexthops, ingress, interest, pitEntry, clarifyDirection(pre_node, node->GetId()));

    if (nextHop == nexthops.end()) {
        NFD_LOG_DEBUG(interest << " from=" << ingress << " noNextHop");
        return;
    }

	auto egress = FaceEndpoint(nextHop->getFace(), 0);
	NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << "to=" << egress);
	this->sendInterest(pitEntry, egress, interest);

	int next_nodeId;
	if (nextHop->getFace().getId()<256+m_nodes.GetN()) {
		next_nodeId = nextHop->getFace().getId()-257 + (node->GetId()+257<=nextHop->getFace().getId());
	    this->updateHopList(node->GetId(),next_nodeId, interest);
	}

	return;
}

void PRFS::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data)
{
  
  NFD_LOG_DEBUG("do Receive Data pitEntry=" << pitEntry->getName()
                << " in=" << ingress << " data=" << data.getName());

  Interest interest = pitEntry->getInterest();
  this->beforeSatisfyInterest(pitEntry, ingress, data);

  this->sendDataToAll(pitEntry, ingress, data);
	const auto transport =ingress.face.getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans =  dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
    if (wifiTrans==nullptr) {
        for(int i=0; i<m_nodes.GetN();++i){
            for (uint32_t j = 0; j < m_nodes[i]->GetNApplications(); ++j) {
                ns3::Ptr<ns3::Application> app = m_nodes[i]->GetApplication(j);
                if (app->GetInstanceTypeId().GetName() == "ns3::ndn::Producer") {
					this->getHopCounts(interest, m_nodes[i]);
				}
            }
        }
    }
}

void
PRFS::afterContentStoreHit(const shared_ptr<pit::Entry>& pitEntry,
                               const FaceEndpoint& ingress, const Data& data)
{
  NFD_LOG_DEBUG("afterContentStoreHit pitEntry=" << pitEntry->getName()
                << " in=" << ingress << " data=" << data.getName());

  this->sendData(pitEntry, data, ingress);

  	const auto transport =ingress.face.getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans =  dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
	ns3::Ptr<ns3::Node>  node = wifiTrans->GetNetDevice()->GetNode();
	this->getHopCounts(pitEntry->getInterest(), node);
}

void PRFS::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                           const shared_ptr<pit::Entry>& pitEntry) {
    this->processNack(ingress.face, nack, pitEntry);

}

nfd::fib::NextHopList::const_iterator 
PRFS::getBestHop(const fib::NextHopList& nexthops,
									const FaceEndpoint& ingress,
                               		const Interest& interest,
                               		const shared_ptr<pit::Entry>& pitEntry,
                                    int road_direction){
    auto nextHop = nexthops.end();
    ns3::Ptr<ns3::Node> nextNode;
    double distance = 0;
    for (auto hop = nexthops.begin(); hop != nexthops.end(); ++hop) {
		if(!isNextHopEligible(ingress.face, interest,  *hop, pitEntry)){continue;}
        const auto transport = hop->getFace().getTransport();
        ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
        if (wifiTrans == nullptr) { return hop; }
        ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
        ns3::Ptr<ns3::MobilityModel> mobility = node->GetObject<ns3::MobilityModel>();
        ns3::Vector3D nodePos = mobility->GetPosition();
        std::string remoteUri = transport->getRemoteUri().getHost();
        ns3::Ptr<ns3::Channel> channel = wifiTrans->GetNetDevice()->GetChannel();
        for (uint32_t deviceId = 0; deviceId < channel->GetNDevices(); ++deviceId) {
            ns3::Address address = channel->GetDevice(deviceId)->GetAddress();
            std::string uri = boost::lexical_cast<std::string>(ns3::Mac48Address::ConvertFrom(address));
            if (remoteUri != uri) {
                continue;
            }
            ns3::Ptr<ns3::Node> remoteNode = channel->GetDevice(deviceId)->GetNode();
            if ( road_direction &&  clarifyDirection(node->GetId(), remoteNode->GetId()) || ( (!road_direction) && (!clarifyDirection(node->GetId(), remoteNode->GetId())) ) ) {
                ns3::Ptr<ns3::MobilityModel> mobModel = remoteNode->GetObject<ns3::MobilityModel>();
                ns3::Vector3D remotePos = mobModel->GetPosition();
                ns3::Vector3D direction = mobModel->GetVelocity();
                double newDistance = caculateDR(nodePos, remotePos, direction);
                for (uint32_t i = 0; i < remoteNode->GetNApplications(); ++i) {
                    ns3::Ptr<ns3::Application> app = remoteNode->GetApplication(i);
                    if (app->GetInstanceTypeId().GetName() == "ns3::ndn::Producer" && newDistance<m_Rth) {
                        // NS_LOG_LOGIC("Arrived in Producer="<<hop->getFace().getId());
                        return hop;
                    }
                }
                if (newDistance > distance && newDistance<m_Rth) {
                    distance = newDistance;
                    nextHop = hop;
                    nextNode = remoteNode;
                }
            }
            break;
        }   
    }
    return nextHop;
}


bool
PRFS::clarifyDirection(int node, int remote_node) {
    ns3::Ptr<ns3::MobilityModel> mobility = m_nodes[node]->GetObject<ns3::MobilityModel>();
    ns3::Vector3D nodePos = mobility->GetPosition();
    ns3::Ptr<ns3::MobilityModel> remoteMob = m_nodes[remote_node]->GetObject<ns3::MobilityModel>();
    ns3::Vector3D remotePos = remoteMob->GetPosition();
    ns3::Vector3D direction = remoteMob->GetVelocity();
    if (direction.x==0 && direction.y==0) { direction.x += 0.001; direction.y  +=0.001;}
    if ( (remotePos.x-nodePos.x) * (direction.x) + (remotePos.y-nodePos.y) * (direction.y) >= 0 ) {return true;}
    return false;
}

double
PRFS::caculateDR(ns3::Vector3D nodePos, ns3::Vector3D remotePos, ns3::Vector3D direction) {
    double eculid = ns3::CalculateDistance(nodePos, remotePos);
    double angle = std::atan2(direction.y, direction.x) - std::atan2(remotePos.y-nodePos.y, remotePos.x-nodePos.x);
    double dr = abs(eculid * cos(angle));
    return dr;
}

void 
PRFS::updateHopList(int preId, int curId, const Interest& interest) 
{   
    uint32_t nonce = interest.getNonce();
    ns3::Ptr<ns3::Node> pre_node = m_nodes[preId];
    ns3::Ptr<ns3::Node> node = m_nodes[curId];
	ndn::Name prefix("/");
	ns3::Ptr<ns3::ndn::L3Protocol> pre_ndn = pre_node->GetObject<ns3::ndn::L3Protocol>();
	nfd::fw::Strategy& strategy1 = pre_ndn->getForwarder()->getStrategyChoice().findEffectiveStrategy(prefix);
	nfd::fw::PRFS& prfs_strategy1 = dynamic_cast<nfd::fw::PRFS&>(strategy1);
    ns3::Ptr<ns3::ndn::L3Protocol> ndn = node->GetObject<ns3::ndn::L3Protocol>();
	nfd::fw::Strategy& strategy2 = ndn->getForwarder()->getStrategyChoice().findEffectiveStrategy(prefix);
	nfd::fw::PRFS& prfs_strategy2 = dynamic_cast<nfd::fw::PRFS&>(strategy2);
	std::map<uint32_t, std::vector<int>>& pre_hop = prfs_strategy1.getHOP();
	std::map<uint32_t, std::vector<int>>& hop = prfs_strategy2.getHOP();
    prfs_strategy2.setHopList(nonce, pre_hop, hop, curId);
}

void
PRFS::setHopList(uint32_t nonce, std::map<uint32_t, std::vector<int>>& pre_hop, std::map<uint32_t, std::vector<int>>& hop, int hopId) {
    if (pre_hop.find(nonce)==pre_hop.end()) {
        pre_hop[nonce] = std::vector<int>{};
    }
    hop[nonce] = pre_hop[nonce];
	hop[nonce].push_back(hopId);
      std::ostringstream oss;
  for (const auto& element : hop[nonce]) {
    oss << element << " ";
  }
    NFD_LOG_INFO("HopList: "<<oss.str());
}

void
PRFS::getHopCounts(const Interest& interest,
                           		 ns3::Ptr<ns3::Node> node)
{
    uint32_t nonce = interest.getNonce();
	ns3::Ptr<ns3::ndn::L3Protocol> ndn = node->GetObject<ns3::ndn::L3Protocol>();
	ndn::Name prefix("/");
	nfd::fw::Strategy& strategy = ndn->getForwarder()->getStrategyChoice().findEffectiveStrategy(prefix);
	nfd::fw::PRFS& prfs_strategy = dynamic_cast<nfd::fw::PRFS&>(strategy);
	std::map<uint32_t, std::vector<int>>& hop = prfs_strategy.getHOP();
	std::vector<int> hop_list = hop[nonce];
	NS_LOG_INFO("Interest="<<interest.getName()<<" Nonce="<<nonce<<" HopCounts="<<hop_list.size());
}

}  // namespace fw
}  // namespace nfd