#include "flood.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/algorithm.hpp"
#include "common/logger.hpp"
#include "ndn-wifi-net-device-transport.hpp"
#include "ns3/mobility-model.h"

namespace nfd {
namespace fw {

NFD_REGISTER_STRATEGY(FLOOD);
NFD_LOG_INIT(FLOOD);

const time::milliseconds FLOOD::RETX_SUPPRESSION_INITIAL(10);
const time::milliseconds FLOOD::RETX_SUPPRESSION_MAX(250);

FLOOD::FLOOD(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  , ProcessNackTraits(this)
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
	
	if(! (isNextHopEligible(ingress.face, interest, nexthop, pitEntry) && this->isInRegion(nexthop))){
		continue;
	}

	this->sendInterest(pitEntry, FaceEndpoint(outFace, 0), interest);
	NFD_LOG_DEBUG("do Send Interest="<<interest);
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
//   NFD_LOG_DEBUG("afterReceiveData pitEntry=" << pitEntry->getName()
//                 << " in=" << ingress << " data=" << data.getName());

  NFD_LOG_DEBUG("do Receive Data="<<data.getName());
  Interest interest = pitEntry->getInterest();
  this->beforeSatisfyInterest(pitEntry, ingress, data);

  this->sendDataToAll(pitEntry, ingress, data);
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
              	NS_LOG_LOGIC("Face: " << hop.getFace().getId()
                                    << ", Node: " << remoteNode->GetId()
                                    << ", Distance: " << distance);
              	return(true);
            }
        }   
	return(false);
}

} // namespace fw
} // namespace nfd
