
#include "bestroute3.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/algorithm.hpp"
#include "common/logger.hpp"
#include "ns3/node.h"
#include "ns3/ptr.h"
#include "ns3/mobility-model.h"
#include "ns3/wifi-net-device.h"
#include "ndn-wifi-net-device-transport.hpp"

namespace nfd {
namespace fw {

NFD_LOG_INIT(BestTest);
NFD_REGISTER_STRATEGY(BestTest);

const time::milliseconds BestTest::RETX_SUPPRESSION_INITIAL(10);
const time::milliseconds BestTest::RETX_SUPPRESSION_MAX(250);

BestTest::BestTest(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  , ProcessNackTraits(this)
  , m_retxSuppression(RETX_SUPPRESSION_INITIAL,
                      RetxSuppressionExponential::DEFAULT_MULTIPLIER,
                      RETX_SUPPRESSION_MAX)
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    NDN_THROW(std::invalid_argument("BestTest does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    NDN_THROW(std::invalid_argument(
      "BestTest does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));
}

const Name&
BestTest::getStrategyName()
{
  static Name strategyName("/localhost/nfd/strategy/bestTest/%FD%01");
  return strategyName;
}

void
BestTest::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                                         const shared_ptr<pit::Entry>& pitEntry)
{	
	if (!isInRegion(ingress.face)) { return ;}

  RetxSuppressionResult suppression = m_retxSuppression.decidePerPitEntry(*pitEntry);
  if (suppression == RetxSuppressionResult::SUPPRESS) {
    NFD_LOG_DEBUG(interest << " from=" << ingress << " suppressed");
    return;
  }

  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
  const fib::NextHopList& nexthops = fibEntry.getNextHops();
  auto it = nexthops.end();

  if (suppression == RetxSuppressionResult::NEW) {
    // forward to nexthop with lowest cost except downstream
    it = std::find_if(nexthops.begin(), nexthops.end(), [&] (const auto& nexthop) {
      return isNextHopEligible(ingress.face, interest, nexthop, pitEntry);
    });

    if (it == nexthops.end()) {
      NFD_LOG_DEBUG(interest << " from=" << ingress << " noNextHop");

      lp::NackHeader nackHeader;
      nackHeader.setReason(lp::NackReason::NO_ROUTE);
      this->sendNack(pitEntry, ingress, nackHeader);

      this->rejectPendingInterest(pitEntry);
      return;
    }

    auto egress = FaceEndpoint(it->getFace(), 0);
    NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << " to=" << egress);
    this->sendInterest(pitEntry, egress, interest);
    return;
  }

  // find an unused upstream with lowest cost except downstream
  it = std::find_if(nexthops.begin(), nexthops.end(), [&] (const auto& nexthop) {
    return isNextHopEligible(ingress.face, interest, nexthop, pitEntry, true, time::steady_clock::now());
  });

  if (it != nexthops.end()) {
    auto egress = FaceEndpoint(it->getFace(), 0);
    this->sendInterest(pitEntry, egress, interest);
    NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << " retransmit-unused-to=" << egress);
    return;
  }

  // find an eligible upstream that is used earliest
  it = findEligibleNextHopWithEarliestOutRecord(ingress.face, interest, nexthops, pitEntry);
  if (it == nexthops.end()) {
    NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << " retransmitNoNextHop");
  }
  else {
    auto egress = FaceEndpoint(it->getFace(), 0);
    this->sendInterest(pitEntry, egress, interest);
    NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << " retransmit-retry-to=" << egress);
  }
}

void
BestTest::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                                     const shared_ptr<pit::Entry>& pitEntry)
{
  this->processNack(ingress.face, nack, pitEntry);
}

void
BestTest::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data)
{
	if ( !isInRegion(ingress.face) ) { return ;}
  NFD_LOG_DEBUG("do Receive Data pitEntry=" << pitEntry->getName()
                << " in=" << ingress << " data=" << data.getName());

  Interest interest = pitEntry->getInterest();
  this->beforeSatisfyInterest(pitEntry, ingress, data);

  this->sendDataToAll(pitEntry, ingress, data);
}

bool
BestTest::isInRegion(const nfd::face::Face& face) {
 	const auto transport = face.getTransport();
  	ns3::ndn::WifiNetDeviceTransport* wifiTrans =
      dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
	if (wifiTrans == nullptr) { return true; }
  	ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
  	ns3::Ptr<ns3::MobilityModel> mobModel = node->GetObject<ns3::MobilityModel>();
  	ns3::Vector3D nodePos = mobModel->GetPosition();
  	std::string remoteUri = transport->getRemoteUri().getHost();
  	ns3::Ptr<ns3::Channel> channel = wifiTrans->GetNetDevice()->GetChannel();
  	for (uint32_t deviceId = 0; deviceId < channel->GetNDevices(); ++deviceId) {
    	ns3::Address address = channel->GetDevice(deviceId)->GetAddress();
    	std::string uri = boost::lexical_cast<std::string>(
        ns3::Mac48Address::ConvertFrom(address));
    	if (remoteUri != uri) {
    		continue;
    	}
    	ns3::Ptr<ns3::Node> remoteNode = channel->GetDevice(deviceId)->GetNode();
    	ns3::Ptr<ns3::MobilityModel> mobModel =
        remoteNode->GetObject<ns3::MobilityModel>();
    	ns3::Vector3D remotePos = mobModel->GetPosition();
    	double distance = sqrt(std::pow((nodePos.x - remotePos.x), 2) +
                           std::pow((nodePos.y - remotePos.y), 2));
    	if (distance < m_Rth) {
      // NS_LOG_LOGIC("Face: " << hop.getFace().getId()
      //                     << ", Node: " << remoteNode->GetId()
      //                     << ", Distance: " << distance);
      	return (true);
    	}
  	}
  	return (false);
}

} // namespace fw
} // namespace nfd
