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
#include "ns3/ndnSIM/apps/ndn-producer.hpp"

#include "core/common.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
#include <random>
namespace nfd {
namespace fw {

NFD_LOG_INIT(LSF);
NFD_REGISTER_STRATEGY(LSF);

const time::milliseconds LSF::RETX_SUPPRESSION_INITIAL(10);
const time::milliseconds LSF::RETX_SUPPRESSION_MAX(250);

LSF::LSF(Forwarder& forwarder, const Name& name)
    : Strategy(forwarder),
      ProcessNackTraits(this),
	  m_Rth(100),
	  m_probtime(0.1),
	  m_nodes(ns3::NodeContainer::GetGlobal()),
	  m_num(m_nodes.GetN()),
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
	initial(m_num);
	// ns3::Simulator::Schedule(ns3::Seconds(1.0), &LSF::sendPosition, this);
}

const Name& LSF::getStrategyName() {
    static Name strategyName("/localhost/nfd/strategy/lsf/%FD%01");
    return strategyName;
}

void LSF::afterReceiveInterest(const FaceEndpoint& ingress,
                               const Interest& interest,
                               const shared_ptr<pit::Entry>& pitEntry)						   
{	
    if(interest.getName().getPrefix(1) == "PosInfo"){
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
		auto nextHop = getBestHop(nexthops, ingress, interest, pitEntry);
		auto egress = FaceEndpoint(nextHop->getFace(), 0);
		NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << "to=" << egress);
		this->sendInterest(pitEntry, egress, interest);

		this->updateISR(ingress, interest, 0);
		// this->probSend(nexthops, ingress, interest, pitEntry);
	}
}


void LSF::sendPosition(){
	for(uint32_t nodeId = 0; nodeId<m_nodes.GetN(); ++nodeId){
		ns3::Ptr<ns3::Node> node = m_nodes.Get(nodeId);
 		ns3::Ptr<ns3::ndn::L3Protocol> ndn = ns3::ndn::L3Protocol::getL3Protocol(m_nodes.Get(nodeId));
		const fib::Entry* fibEntry = ndn->getForwarder()->getFib().findExactMatch("/");
		const fib::NextHopList& nexthops = fibEntry->getNextHops();
		ns3::Ptr<ns3::MobilityModel> mobModel = node->GetObject<ns3::MobilityModel>();
		ns3::Vector3D pos = mobModel->GetPosition();
		ns3::Vector3D speed = mobModel->GetVelocity();
		std::string name = "/PosInfo/Node" +to_string(node->GetId()) + "/posx" + to_string(pos.x) + "/posy" +to_string(pos.y) +"/volx" + to_string(speed.x) +"/voly" +to_string(speed.y);
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
	// NS_LOG_INFO("Update Pos and Vel between "<<localNode->GetId()<<"-"<<neighNode<<": ("<<pos<<"), ("<<vol<<")");
}

void LSF::updateISR(const FaceEndpoint& egress,
                               const Interest& interest,
							   int type)
{	
	std::string name = interest.getName().toUri();
    const auto transport = egress.face.getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans = dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
	if (wifiTrans == nullptr) { return; }
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
  NFD_LOG_DEBUG("do Receive Data pitEntry=" << pitEntry->getName()
                << " in=" << ingress << " data=" << data.getName());
  Interest interest = pitEntry->getInterest();
  this->beforeSatisfyInterest(pitEntry, ingress, data);

  this->sendDataToAll(pitEntry, ingress, data);

  this->updateISR(ingress, interest, 1);
}

std::vector<double>
LSF::caculateHopProb(const fib::NextHopList& nexthoplist,
									const FaceEndpoint& ingress,
                               		const Interest& interest,
                               		const shared_ptr<pit::Entry>& pitEntry) {

    const auto transport = nexthoplist.begin()->getFace().getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans =  dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
	if ( wifiTrans == nullptr) { return std::vector<double>{0.0};}
	ns3::Ptr<ns3::Channel> channel = wifiTrans->GetNetDevice()->GetChannel();
    ns3::Ptr<ns3::Node> localNode = wifiTrans->GetNetDevice()->GetNode();
	ns3::Ptr<ns3::MobilityModel> mobModel = localNode->GetObject<ns3::MobilityModel>();
	ns3::Vector3D localPos = mobModel->GetPosition();
    ns3::Ptr<ns3::Node> remoteNode;
	std::vector<double> distance_list(m_num-1);
	std::vector<double> isr_list(m_num-1);
	std::vector<double> prob_list(m_num-1);
	double sum_distance = 0;
	double sum_isr = 0.001;

	for(int i=0; i<nexthoplist.size(); ++i) {
		std::string remoteUri = nexthoplist[i].getFace().getRemoteUri().getHost();
		for(uint32_t deviceId = 0; deviceId < channel->GetNDevices(); ++deviceId){
			ns3::Address address = channel->GetDevice(deviceId)->GetAddress();
			std::string uri = boost::lexical_cast<std::string>(ns3::Mac48Address::ConvertFrom(address));
			// 找到当前hop对应的下一跳节点
			if(remoteUri == uri){
				remoteNode = channel->GetDevice(deviceId)->GetNode();	
				// ns3::Vector3D remotePos = m_posMap[localNode->GetId()][remoteNode->GetId()];
			ns3::Ptr<ns3::MobilityModel> mobModel2 = remoteNode->GetObject<ns3::MobilityModel>();
            ns3::Vector3D remotePos = mobModel2->GetPosition();
				double newDistance = sqrt(std::pow((localPos.x-remotePos.x), 2) + std::pow((localPos.y-remotePos.y), 2));
				distance_list[i] = newDistance;
				// 计算isr
				auto &a = m_isr[remoteNode->GetId()];
				std::string name = interest.getName().toUri();
				if(a.find(name) == a.end()){
					isr_list[i] = 0;
				}
				else { 
					isr_list[i] = m_isr[remoteNode->GetId()][interest.getName().toUri()][1] / m_isr[remoteNode->GetId()][interest.getName().toUri()][0];
					// NS_LOG_DEBUG("Caculate ISR="<<isr_list[i]);
				}
				// 判断其是否为Producer，若是则概率置为1
				for (uint32_t j = 0; j < remoteNode->GetNApplications(); ++j) {
					ns3::Ptr<ns3::Application> app = remoteNode->GetApplication(j);
					if (app->GetInstanceTypeId().GetName() == "ns3::ndn::Producer" && newDistance<m_Rth) {
						prob_list[i] = 1.0;
						sum_distance -= newDistance;
						sum_isr -= isr_list[i];
					}
				}
				if (newDistance < m_Rth) {
					sum_distance += newDistance;
					sum_isr += isr_list[i];
				}
				break;
			}
		}
	}
	// 计算概率
	for (int i = 0; i < prob_list.size(); ++i) {
		double distance = distance_list[i];
		if(!isNextHopEligible(ingress.face, interest,  nexthoplist[i], pitEntry)) { sum_distance -= distance; continue; }
		if ( distance >= m_Rth || (prob_list[i]==1.0)) { continue; }
		// prob_list[i] = distance_list[i] / sum_distance;
		prob_list[i] = 0.5 * (distance_list[i] / sum_distance) + 0.5*(isr_list[i] / sum_isr);
	}
	for(int i = 0; i < prob_list.size(); ++i) {NS_LOG_DEBUG("hop="<<nexthoplist[i].getFace().getId()<<" distance="<<distance_list[i]<<" prob="<<prob_list[i]);}
		
	return prob_list;
}


void
LSF::probSend(const fib::NextHopList& nexthoplist,
									const FaceEndpoint& ingress,
                               		const Interest& interest,
                               		const shared_ptr<pit::Entry>& pitEntry){
	std::vector<double> prob_list = caculateHopProb(nexthoplist, ingress, interest, pitEntry);
	int best_hop_index;
	auto it = std::find(prob_list.begin(), prob_list.end(), 1.0);
	if( it != prob_list.end()){
		best_hop_index = std::distance(prob_list.begin(), it);
	}
	else{
		// best_hop_index = rouletteWheelSelection(prob_list);
		best_hop_index = getBestProb(prob_list);
	}
	auto egress = FaceEndpoint(nexthoplist[best_hop_index].getFace(), 0);
	NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << "to=" << egress<<" prob=" << prob_list[best_hop_index]);
	this->sendInterest(pitEntry, egress, interest);
}


int LSF::rouletteWheelSelection(const std::vector<double>& probabilities) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // 复制概率分布并进行排序
    std::vector<double> sortedProbabilities = probabilities;
    std::sort(sortedProbabilities.begin(), sortedProbabilities.end());

    // 计算概率分布的总和
    double totalProbability = std::accumulate(sortedProbabilities.begin(), sortedProbabilities.end(), 0.0);

    // 生成一个随机数
    double randomNum = dist(gen);

    // 使用轮盘法选择概率分布
    double sum = 0.0;
    for (int i = 0; i < sortedProbabilities.size(); ++i) {
        sum += sortedProbabilities[i] / totalProbability;
        if (randomNum <= sum) {
            // 在原始概率分布中查找对应索引
            auto it = std::find(probabilities.begin(), probabilities.end(), sortedProbabilities[i]);
            return std::distance(probabilities.begin(), it);
        }
    }

    // 如果未能选择到任何概率分布，则返回最后一个索引
    return probabilities.size() - 1;
}

int LSF::getBestProb(const std::vector<double>& problist) {
	int inx = 0;
	double max_prob = 0;
	for(int i = 0; i<problist.size(); ++i) {
		if (problist[i] > max_prob) {
			max_prob = problist[i];
			inx = i;
		}
	}
	return inx;
}

nfd::fib::NextHopList::const_iterator 
LSF::getBestHop(const fib::NextHopList& nexthops,
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

void LSF::initial(uint32_t num){
	m_isr = std::vector<std::map<std::string, std::vector<double>>>(num);
	double max = std::numeric_limits<double>::max();
	m_posMap = std::vector<std::vector<ns3::Vector3D>>(num, std::vector<ns3::Vector3D>(num, ns3::Vector3D(max, max, max)));
	m_volMap = std::vector<std::vector<ns3::Vector3D>>(num, std::vector<ns3::Vector3D>(num, ns3::Vector3D(0, 0, 0)));
	m_prob = std::vector<std::vector<double>>(num,std::vector<double>(num-1, {0.0}));
	for(uint32_t nodeId=0; nodeId<num; ++nodeId){
		for(uint32_t othId=0; othId<num; ++othId){
			ns3::Ptr<ns3::MobilityModel> mobModel = m_nodes.Get(othId)->GetObject<ns3::MobilityModel>();
			ns3::Vector3D pos = mobModel->GetPosition();
			ns3::Vector3D speed = mobModel->GetVelocity();
			m_posMap[nodeId][othId] = pos;
			m_volMap[nodeId][othId] = speed;
		}
	}
}


void LSF::afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                           const shared_ptr<pit::Entry>& pitEntry) {
    this->processNack(ingress.face, nack, pitEntry);
}

}  // namespace fw
}  // namespace nfd
