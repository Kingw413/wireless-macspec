#include "prfs.hpp"

#include "common/logger.hpp"
#include "ndn-wifi-net-device-transport.hpp"
#include "ns3/mobility-model.h"
#include "ns3/ndnSIM/NFD/daemon/fw/algorithm.hpp"
#include "ns3/ndnSIM/model/ndn-net-device-transport.hpp"
#include "ns3/node.h"
#include "ns3/ptr.h"
#include "ns3/wifi-net-device.h"
#include "ns3/ndnSIM/apps/ndn-producer.hpp"

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
    auto nextHop = getBestHop(nexthops, ingress, interest, pitEntry);
    // NS_LOG_DEBUG("Nexthop: #" << nextHop->getFace().getId());


    if (nextHop == nexthops.end()) {
        NFD_LOG_DEBUG(interest << " from=" << ingress << " noNextHop");
        lp::NackHeader nackHeader;
        nackHeader.setReason(lp::NackReason::NO_ROUTE);
        this->sendNack(pitEntry, ingress, nackHeader);
        this->rejectPendingInterest(pitEntry);
        return;
    }

	auto egress = FaceEndpoint(nextHop->getFace(), 0);
	NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << "to=" << egress);
	this->sendInterest(pitEntry, egress, interest);
	return;
}

void PRFS::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                           const shared_ptr<pit::Entry>& pitEntry) {
    this->processNack(ingress.face, nack, pitEntry);

}

void PRFS::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data)
{
  NFD_LOG_DEBUG("do Receive Data pitEntry=" << pitEntry->getName()
                << " in=" << ingress << " data=" << data.getName());

  Interest interest = pitEntry->getInterest();
  this->beforeSatisfyInterest(pitEntry, ingress, data);

  this->sendDataToAll(pitEntry, ingress, data);
}

nfd::fib::NextHopList::const_iterator 
PRFS::getBestHop(const fib::NextHopList& nexthops,
									const FaceEndpoint& ingress,
                               		const Interest& interest,
                               		const shared_ptr<pit::Entry>& pitEntry){
    auto nextHop = nexthops.begin();
    ns3::Ptr<ns3::Node> nextNode;
    double distance = 0;

    for (auto hop = nexthops.begin(); hop != nexthops.end(); ++hop) {
		if(!isNextHopEligible(ingress.face, interest,  *hop, pitEntry)){continue;}

        const auto transport = hop->getFace().getTransport();
        ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
        if (wifiTrans == nullptr) { return hop; }
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
            double newDistance = sqrt(std::pow((nodePos.x - remotePos.x), 2) + std::pow((nodePos.y - remotePos.y), 2));
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
            // NS_LOG_LOGIC("Face: " << hop->getFace().getId()
            //                       << ", Node: " << remoteNode->GetId()
            //                       << ", Distance: " << newDistance);
            break;
        }   
    }
    return nextHop;
}
                    

}  // namespace fw
}  // namespace nfd