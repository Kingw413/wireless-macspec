#include "lsf.hpp"

#include "common/logger.hpp"
#include "ndn-wifi-net-device-transport.hpp"
#include "ns3/mobility-model.h"
#include "ns3/ndnSIM/NFD/daemon/fw/algorithm.hpp"
#include "ns3/ndnSIM/model/ndn-net-device-transport.hpp"
#include "ns3/node.h"
#include "ns3/ptr.h"
#include "ns3/wifi-net-device.h"

namespace nfd {
namespace fw {

NFD_LOG_INIT(LSF);
NFD_REGISTER_STRATEGY(LSF);

const time::milliseconds LSF::RETX_SUPPRESSION_INITIAL(10);
const time::milliseconds LSF::RETX_SUPPRESSION_MAX(250);

LSF::LSF(Forwarder& forwarder, const Name& name)
    : Strategy(forwarder),
      ProcessNackTraits(this),
      fw(forwarder),
      m_retxSuppression(RETX_SUPPRESSION_INITIAL,
                        RetxSuppressionExponential::DEFAULT_MULTIPLIER,
                        RETX_SUPPRESSION_MAX) {
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
}

const Name& LSF::getStrategyName() {
    static Name strategyName("/localhost/nfd/strategy/lsf/%FD%10");
    return strategyName;
}

void LSF::afterReceiveInterest(const FaceEndpoint& ingress,
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
			NS_LOG_DEBUG("Transport: "<<transport->getRemoteUri().getHost());
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
                    NS_LOG_LOGIC("prefix: " << fibEntry.getPrefix()
                                                << ", Face: " << hop->getFace().getId()
                                                <<", Node: "<<remoteNode->GetId()
                                                <<", Distance: "<<newDistance);
                }
		    }
	    }
    }
    NS_LOG_DEBUG("Nexthop: [face=" << nextHop->getFace().getId()<<", node="<<nextNode->GetId()<< ", Distance=" << distance<<"]");


    if (nextHop == nexthops.end()) {
        NFD_LOG_DEBUG(interest << " from=" << ingress << " noNextHop");
        lp::NackHeader nackHeader;
        nackHeader.setReason(lp::NackReason::NO_ROUTE);
        this->sendNack(pitEntry, ingress, nackHeader);
        this->rejectPendingInterest(pitEntry);
        return;
    }

	auto egress = FaceEndpoint(nextHop->getFace(), 0);
	NFD_LOG_DEBUG(interest << " from=" << ingress << " newPitEntry-to=" << egress);
	this->sendInterest(pitEntry, egress, interest);
	return;
}

void LSF::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                           const shared_ptr<pit::Entry>& pitEntry) {
    this->processNack(ingress.face, nack, pitEntry);

    //   Interest pkti;
    //   pkti.setName("pos");
}

}  // namespace fw
}  // namespace nfd
