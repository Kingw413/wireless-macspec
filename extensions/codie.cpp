
#include "codie.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/algorithm.hpp"
#include "common/logger.hpp"
#include "ns3/node.h"
#include "ns3/ptr.h"
#include "ns3/mobility-model.h"
#include "ns3/wifi-net-device.h"
#include "ndn-wifi-net-device-transport.hpp"
#include "ns3/ndnSIM/apps/ndn-producer.hpp"

namespace nfd {
namespace fw {

NFD_LOG_INIT(CODIE);
NFD_REGISTER_STRATEGY(CODIE);

const time::milliseconds CODIE::RETX_SUPPRESSION_INITIAL(10);
const time::milliseconds CODIE::RETX_SUPPRESSION_MAX(250);

CODIE::CODIE(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  , ProcessNackTraits(this)
  , m_retxSuppression(RETX_SUPPRESSION_INITIAL,
                      RetxSuppressionExponential::DEFAULT_MULTIPLIER,
                      RETX_SUPPRESSION_MAX)
  ,	 m_nodes(ns3::NodeContainer::GetGlobal())
  ,	 m_num(m_nodes.GetN())
  ,  m_h(std::vector<std::map<std::string, int>>(m_num))
  ,  m_ddl(std::vector<std::map<std::string, int>>(m_num))
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    NDN_THROW(std::invalid_argument("CODIE does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    NDN_THROW(std::invalid_argument(
      "CODIE does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));
}

const Name&
CODIE::getStrategyName()
{
  static Name strategyName("/localhost/nfd/strategy/codie/%FD%01");
  return strategyName;
}

void
CODIE::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                                         const shared_ptr<pit::Entry>& pitEntry)
{	
	if (!isInRegion(ingress.face)) { 
    NFD_LOG_DEBUG("It is not in region");
    pitEntry->deleteInRecord(ingress.face) ; return;}

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

      auto egress = FaceEndpoint(it->getFace(), 0);
      NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << " to=" << egress);
      this->sendInterest(pitEntry, egress, interest);

    // CODIE的核心操作
    std::string name = interest.getName().toUri();
      const auto transport =nexthops.begin()->getFace().getTransport();
      ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
    if (wifiTrans == nullptr) {return; }
      int nodeId = wifiTrans->GetNetDevice()->GetNode()->GetId();
    int faceId = ingress.face.getId();
    if (faceId == 256+m_num) {
      m_h[nodeId][name] = 0;
      return;
    }

    int preId = faceId - 257 + ( faceId-257 >= nodeId );
    m_h[nodeId][name] = m_h[preId][name] + 1;

      ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
    for (uint32_t j = 0; j < node->GetNApplications(); ++j) {
      ns3::Ptr<ns3::Application> app = node->GetApplication(j);
      if (app->GetInstanceTypeId().GetName() == "ns3::ndn::Producer") {
        m_ddl[nodeId][name] = m_h[nodeId][name] + 1;
        break;
      }
    }

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

  
	// CODIE的核心操作
	std::string name = interest.getName().toUri();
  const auto transport =nexthops.begin()->getFace().getTransport();
   ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
	if (wifiTrans == nullptr) {return; }
    int nodeId = wifiTrans->GetNetDevice()->GetNode()->GetId();
	int faceId = ingress.face.getId();
	if (faceId == 256+m_num) {
		m_h[nodeId][name] = 0;
		return;
	}

	int preId = faceId - 257 + ( faceId-257 >= nodeId );
	m_h[nodeId][name] = m_h[preId][name] + 1;

    ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
	for (uint32_t j = 0; j < node->GetNApplications(); ++j) {
		ns3::Ptr<ns3::Application> app = node->GetApplication(j);
		if (app->GetInstanceTypeId().GetName() == "ns3::ndn::Producer") {
			m_ddl[nodeId][name] = m_h[nodeId][name] + 1;
			break;
		}
	}
}

void
CODIE::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                                     const shared_ptr<pit::Entry>& pitEntry)
{
  this->processNack(ingress.face, nack, pitEntry);
  NFD_LOG_DEBUG("Receive Nack from"<<ingress<<"pitEntry="<<pitEntry);
}

void
CODIE::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data)
{
	if ( !isInRegion(ingress.face) ) { pitEntry->deleteInRecord(ingress.face); return;}

	std::string name = data.getName().toUri();
    const auto transport = ingress.face.getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
	if ( wifiTrans != nullptr ) {
		int nodeId = wifiTrans->GetNetDevice()->GetNode()->GetId();
		int faceId = ingress.face.getId();
		int preId = faceId - 257 + ( faceId-257 >= nodeId );
		m_ddl[nodeId][name] = m_ddl[nodeId][name] - 1;
		if ( m_ddl[nodeId][name] <= m_h[nodeId][name] ) {return;}
	}

  NFD_LOG_DEBUG("do Receive Data pitEntry=" << pitEntry->getName()
                << " in=" << ingress << " data=" << data.getName());

  Interest interest = pitEntry->getInterest();
  this->beforeSatisfyInterest(pitEntry, ingress, data);

  this->sendDataToAll(pitEntry, ingress, data);
}

void
CODIE::afterContentStoreHit(const shared_ptr<pit::Entry>& pitEntry,
                               const FaceEndpoint& ingress, const Data& data)
{
  NFD_LOG_DEBUG("afterContentStoreHit pitEntry=" << pitEntry->getName()
                << " in=" << ingress << " data=" << data.getName());

  this->sendData(pitEntry, data, ingress);
}

bool
CODIE::isInRegion(const nfd::face::Face& face) {
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
      NS_LOG_LOGIC("Face: " << face.getId()
                          << ", Node: " << remoteNode->GetId()
                          << ", Distance: " << distance);
      	return (true);
    	}
  	}
  	return (false);
}

} // namespace fw
} // namespace nfd
