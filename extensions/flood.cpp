#include "flood.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/algorithm.hpp"
#include "common/logger.hpp"
#include "ndn-wifi-net-device-transport.hpp"
#include "ns3/mobility-model.h"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/apps/ndn-producer.hpp"

namespace nfd {
namespace fw {

NFD_REGISTER_STRATEGY(FLOOD);
NFD_LOG_INIT(FLOOD);

const time::milliseconds FLOOD::RETX_SUPPRESSION_INITIAL(10);
const time::milliseconds FLOOD::RETX_SUPPRESSION_MAX(250);

FLOOD::FLOOD(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  , ProcessNackTraits(this)
  ,	m_nodes(ns3::NodeContainer::GetGlobal())
  , m_Rth(100)
  , m_retxSuppression(RETX_SUPPRESSION_INITIAL,
                      RetxSuppressionExponential::DEFAULT_MULTIPLIER,
                      RETX_SUPPRESSION_MAX)
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    NDN_THROW(std::invalid_argument("FLOOD does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    NDN_THROW(std::invalid_argument(
      "FLOOD does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));
}

const Name&
FLOOD::getStrategyName()
{
  static Name strategyName("/localhost/nfd/strategy/flood/%FD%01");
  return strategyName;
}

void
FLOOD::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                                        const shared_ptr<pit::Entry>& pitEntry)
{
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
  const fib::NextHopList& nexthops = fibEntry.getNextHops();

  int nEligibleNextHops = 0;

  bool isSuppressed = false;

  for (const auto& nexthop : nexthops) {
    Face& outFace = nexthop.getFace();

    RetxSuppressionResult suppressResult = m_retxSuppression.decidePerUpstream(*pitEntry, outFace);

    if (suppressResult == RetxSuppressionResult::SUPPRESS) {
      NFD_LOG_DEBUG(interest << " from=" << ingress << " to=" << outFace.getId() << " suppressed");
      isSuppressed = true;
      continue;
    }

    if ((outFace.getId() == ingress.face.getId() && outFace.getLinkType() != ndn::nfd::LINK_TYPE_AD_HOC) ||
        wouldViolateScope(ingress.face, interest, outFace)) {
      continue;
    }
	
	if( !( isNextHopEligible(ingress.face, interest, nexthop, pitEntry) && this->isInRegion(nexthop))){
		continue;
	}

	this->sendInterest(pitEntry, FaceEndpoint(outFace, 0), interest);
  this->updateHopList(ingress.face, outFace, interest);
	NFD_LOG_DEBUG("do Send Interest="<<interest<<" from=" << ingress << " pitEntry-to=" << outFace.getId());
	// NFD_LOG_DEBUG(interest << " from=" << ingress << " pitEntry-to=" << outFace.getId());

    if (suppressResult == RetxSuppressionResult::FORWARD) {
      m_retxSuppression.incrementIntervalForOutRecord(*pitEntry->getOutRecord(outFace));
    }
    ++nEligibleNextHops;
  }

  if (nEligibleNextHops == 0 && !isSuppressed) {
    NFD_LOG_DEBUG(interest << " from=" << ingress << " noNextHop");

    lp::NackHeader nackHeader;
    nackHeader.setReason(lp::NackReason::NO_ROUTE);
    this->sendNack(pitEntry, ingress, nackHeader);
    this->rejectPendingInterest(pitEntry);
  }
}

void FLOOD::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data)
{
  NFD_LOG_DEBUG("afterReceiveData pitEntry=" << pitEntry->getName()
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
					this->getHopCounts(interest, m_nodes[i]); }
            }
        }
    }
}

void
FLOOD::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                                    const shared_ptr<pit::Entry>& pitEntry)
{
  this->processNack(ingress.face, nack, pitEntry);
}

bool
FLOOD::isInRegion(const nfd::fib::NextHop hop){
        const auto transport = hop.getFace().getTransport();
        ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
        if(wifiTrans==nullptr){return true;}
        ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
        ns3::Ptr<ns3::MobilityModel> mobModel = node->GetObject<ns3::MobilityModel>();
        ns3::Vector3D nodePos = mobModel->GetPosition();
        std::string remoteUri = transport->getRemoteUri().getHost();
        ns3::Ptr<ns3::Channel> channel = wifiTrans->GetNetDevice()->GetChannel();
        for (uint32_t deviceId = 0; deviceId < channel->GetNDevices(); ++deviceId) {
            ns3::Address address = channel->GetDevice(deviceId)->GetAddress();
            std::string uri = boost::lexical_cast<std::string>(ns3::Mac48Address::ConvertFrom(address));
            if (remoteUri != uri) {
                continue;
            }
            ns3::Ptr<ns3::Node> remoteNode = channel->GetDevice(deviceId)->GetNode();
            ns3::Ptr<ns3::MobilityModel> mobModel = remoteNode->GetObject<ns3::MobilityModel>();
            ns3::Vector3D remotePos = mobModel->GetPosition();
            double distance = sqrt(std::pow((nodePos.x - remotePos.x), 2) + std::pow((nodePos.y - remotePos.y), 2));
            if(distance < m_Rth){
              	return(true);
            }
        }   
	return(false);
}

void 
FLOOD::updateHopList(nfd::face::Face& inface, nfd::face::Face& outface, const Interest& interest) 
{   	
	if (outface.getId()<256+m_nodes.GetN()) {
		ns3::Ptr<ns3::Node> node;
		const auto transport = inface.getTransport();
		ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
		if(wifiTrans == nullptr) {
			const auto transport2 = outface.getTransport();
			ns3::ndn::WifiNetDeviceTransport* wifiTrans2 = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport2);
			node = wifiTrans2->GetNetDevice()->GetNode();
		}
		else{
			node = wifiTrans->GetNetDevice()->GetNode();
		}
		int next_nodeId = outface.getId()-257 + (node->GetId()+257<=outface.getId());
		uint32_t nonce = interest.getNonce();
		ns3::Ptr<ns3::Node> next_node = m_nodes[next_nodeId];
		ndn::Name prefix("/");
		ns3::Ptr<ns3::ndn::L3Protocol> pre_ndn = node->GetObject<ns3::ndn::L3Protocol>();
		nfd::fw::Strategy& strategy1 = pre_ndn->getForwarder()->getStrategyChoice().findEffectiveStrategy(prefix);
		nfd::fw::FLOOD& FLOOD_strategy1 = dynamic_cast<nfd::fw::FLOOD&>(strategy1);
		ns3::Ptr<ns3::ndn::L3Protocol> ndn = next_node->GetObject<ns3::ndn::L3Protocol>();
		nfd::fw::Strategy& strategy2 = ndn->getForwarder()->getStrategyChoice().findEffectiveStrategy(prefix);
		nfd::fw::FLOOD& FLOOD_strategy2 = dynamic_cast<nfd::fw::FLOOD&>(strategy2);
		std::map<uint32_t, std::vector<int>>& pre_hop = FLOOD_strategy1.getHOP();
		std::map<uint32_t, std::vector<int>>& hop = FLOOD_strategy2.getHOP();
		FLOOD_strategy2.setHopList(nonce, pre_hop, hop, node->GetId(), next_nodeId);
	}
}

void
FLOOD::setHopList(uint32_t nonce, std::map<uint32_t, std::vector<int>>& pre_hop, std::map<uint32_t, std::vector<int>>& hop, int hopId, int next_hopId) {
    if (pre_hop.find(nonce)==pre_hop.end()) {
        pre_hop[nonce] = {hopId};
    }
    hop[nonce] = pre_hop[nonce];
	hop[nonce].push_back(next_hopId);
      std::ostringstream oss;
//   for (const auto& element : pre_hop[nonce]) {
//     oss << element << " ";
//   }
//     NFD_LOG_INFO("HopList: "<<oss.str());
}

void
FLOOD::getHopCounts(const Interest& interest,
                           		 ns3::Ptr<ns3::Node> node)
{
    uint32_t nonce = interest.getNonce();
	ns3::Ptr<ns3::ndn::L3Protocol> ndn = node->GetObject<ns3::ndn::L3Protocol>();
	ndn::Name prefix("/");
	nfd::fw::Strategy& strategy = ndn->getForwarder()->getStrategyChoice().findEffectiveStrategy(prefix);
	nfd::fw::FLOOD& FLOOD_strategy = dynamic_cast<nfd::fw::FLOOD&>(strategy);
	std::map<uint32_t, std::vector<int>>& hop = FLOOD_strategy.getHOP();
	std::vector<int> hop_list = hop[nonce];
	std::ostringstream oss;
	for (const auto& element : hop_list) {
    	oss << element << " ";
  	}
    NFD_LOG_INFO("HopList: "<<oss.str());
	NS_LOG_INFO("Interest="<<interest.getName()<<" Nonce="<<nonce<<" HopCounts="<<hop_list.size()-1);
}

} // namespace fw
} // namespace nfd
