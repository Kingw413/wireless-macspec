#include "prfs.hpp"

#include "common/logger.hpp"
#include "ndn-wifi-net-device-transport.hpp"
#include "ns3/mobility-model.h"
#include "ns3/ndnSIM/NFD/daemon/fw/algorithm.hpp"
#include "ns3/ndnSIM/model/ndn-net-device-transport.hpp"
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
        }
        auto next_rrd = getBestHop(nexthops, ingress, interest, pitEntry, false);
        if (next_rrd == nexthops.end()) {NFD_LOG_DEBUG(interest << " from=" << ingress << " noRRDHop");}
        else{
            auto egress2 = FaceEndpoint(next_rrd->getFace(), 0);
            NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << "to=" << egress2);
            this->sendInterest(pitEntry, egress2, interest);
        }
	    return;
    }

    ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
    int pre_node = (ingress.face.getId() - 257) + (node->GetId()+257 <= ingress.face.getId());
    auto nextHop = getBestHop(nexthops, ingress, interest, pitEntry, clarifyDirection(pre_node, node->GetId()));

    if (nextHop == nexthops.end()) {
        NFD_LOG_DEBUG(interest << " from=" << ingress << " noNextHop");
        // lp::NackHeader nackHeader;
        // nackHeader.setReason(lp::NackReason::NO_ROUTE);
        // this->sendNack(pitEntry, ingress, nackHeader);
        // this->rejectPendingInterest(pitEntry);
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


}  // namespace fw
}  // namespace nfd