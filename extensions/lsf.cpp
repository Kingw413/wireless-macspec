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
	initial(m_num, m_isr);
	// std::vector<std::map<std::string, std::vector<double>>>m_isr(m_num);

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
		int best_hop_index = getBestHop(nexthops, ingress, interest, pitEntry);
		auto egress = FaceEndpoint(nexthops[best_hop_index].getFace(), 0);
		// NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << " to=" << egress);
		this->sendInterest(pitEntry, egress, interest);
		this->updateISR(egress, interest, 0, this->m_isr);

		if (ingress.face.getId() == 256+m_num) {}
	}
}

void LSF::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data)
{
//   NFD_LOG_DEBUG("do Receive Data pitEntry=" << pitEntry->getName()
//                 << " in=" << ingress << " data=" << data.getName());


  Interest interest = pitEntry->getInterest();
  uint32_t nonce = interest.getNonce();
  this->beforeSatisfyInterest(pitEntry, ingress, data);

  this->sendDataToAll(pitEntry, ingress, data);

  this->updateISR(ingress, interest, 1, this->m_isr);

  	const auto transport =ingress.face.getTransport();
    ns3::ndn::WifiNetDeviceTransport* wifiTrans =  dynamic_cast<ns3::ndn::WifiNetDeviceTransport*>(transport);
	if (wifiTrans != nullptr){
		ns3::Ptr<ns3::Node> node = wifiTrans->GetNetDevice()->GetNode();
		for (uint32_t j = 0; j < node->GetNApplications(); ++j) {
            ns3::Ptr<ns3::Application> app = node->GetApplication(j);
            if (app->GetInstanceTypeId().GetName() =="ns3::ndn::ConsumerZipfMandelbrot") {
				ns3::Ptr<ns3::ndn::L3Protocol> ndn = node->GetObject<ns3::ndn::L3Protocol>();
				ndn::Name prefix("/");
				nfd::fw::Strategy& strategy = ndn->getForwarder()->getStrategyChoice().findEffectiveStrategy(prefix);
				nfd::fw::LSF& lsf_strategy = dynamic_cast<nfd::fw::LSF&>(strategy);
				std::map<uint32_t, std::vector<int>>& hop = lsf_strategy.getHOP();
				std::vector<int> hop_list = hop[nonce];
				NS_LOG_INFO("Interest="<<data.getName()<<" Nonce="<<nonce<<" HopCounts="<<hop_list.size());
			}
		}
	}
}

void
LSF::afterContentStoreHit(const shared_ptr<pit::Entry>& pitEntry,
                               const FaceEndpoint& ingress, const Data& data)
{
//   NFD_LOG_DEBUG("afterContentStoreHit pitEntry=" << pitEntry->getName()
//                 << " in=" << ingress << " data=" << data.getName());

  this->sendData(pitEntry, data, ingress);
  this->updateISR(ingress, pitEntry->getInterest(), 1, this->m_isr);
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

	this->updateHopList(localNode->GetId(), interest);

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
				ns3::Ptr<ns3::ndn::L3Protocol> ndn = remoteNode->GetObject<ns3::ndn::L3Protocol>();
				ndn::Name prefix("/");
				nfd::fw::Strategy& strategy =  ndn->getForwarder()->getStrategyChoice().findEffectiveStrategy(prefix);
    			nfd::fw::LSF& lsf_strategy =  dynamic_cast<nfd::fw::LSF&>(strategy);
				std::map<std::string, std::vector<int>> isr = lsf_strategy.getISR();

				// for(auto it=isr.begin();it != isr.end();++it) {
				// 	std::string name=it->first;
				// 	std::vector<int> isr_num=it->second;
				// 	NS_LOG_INFO("node="<<remoteNode->GetId()<<" ,name="<<name<<" ,Inte_num="<<isr_num[0]<<" ,Data_num="<<isr_num[1]<<" ,ISR="<< (isr_num[1]/isr_num[0]));
				// }

				ns3::Ptr<ns3::MobilityModel> mobModel2 = remoteNode->GetObject<ns3::MobilityModel>();
				ns3::Vector3D remotePos = mobModel2->GetPosition();
				double newDistance = sqrt(std::pow((localPos.x-remotePos.x), 2) + std::pow((localPos.y-remotePos.y), 2));
				distance_list[i] = newDistance;
				// 计算isr
				std::string name = interest.getName().toUri();
				if(isr.find(name) == isr.end()){
					// NFD_LOG_DEBUG("Interest="<<name<<", ISR="<<0);
					isr_list[i] = 0;
				}
				else { 
					isr_list[i] = isr[name][1] / isr[name][0];
					// NFD_LOG_INFO("Interest="<<interest.getName().toUri()<<", ISR="<<isr_list[i]);
				}					

				// 判断其是否为Producer，若是则概率置为1
				for (uint32_t j = 0; j < remoteNode->GetNApplications(); ++j) {
					ns3::Ptr<ns3::Application> app = remoteNode->GetApplication(j);
					if (app->GetInstanceTypeId().GetName() == "ns3::ndn::Producer" && newDistance<m_Rth) {
						prob_list[i] = 1.0;
						// sum_distance -= newDistance;
						// sum_isr -= isr_list[i];
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
		if ( distance >= m_Rth || (prob_list[i]==1.0)) { continue; }
		if(!isNextHopEligible(ingress.face, interest,  nexthoplist[i], pitEntry)) { 
			sum_isr -= isr_list[i];
			sum_distance -= distance;
			continue; 
		}
		// prob_list[i] = distance_list[i] / sum_distance;
		prob_list[i] = 0.5 * (distance_list[i] / sum_distance) + 0.5*(isr_list[i] / sum_isr);
	}
	// for(int i = 0; i < prob_list.size(); ++i) {NS_LOG_DEBUG("hop="<<nexthoplist[i].getFace().getId()<<" distance="<<distance_list[i]<<" isr="<<isr_list[i]<<" prob="<<prob_list[i]);}
		
	return prob_list;
}

int
LSF::getBestHop(const fib::NextHopList& nexthoplist,
									const FaceEndpoint& ingress,
                               		const Interest& interest,
                               		const shared_ptr<pit::Entry>& pitEntry){
	std::vector<double> prob_list = caculateHopProb(nexthoplist, ingress, interest, pitEntry);
	int best_hop_index=0;
	auto it = std::find(prob_list.begin(), prob_list.end(), 1.0);
	if( it != prob_list.end()){
		best_hop_index = std::distance(prob_list.begin(), it);
	}
	else{
		double max_prob = 0;
		for(int i = 0; i<prob_list.size(); ++i) {
			if (prob_list[i] > max_prob) {
				max_prob = prob_list[i];
				best_hop_index = i;
			}
		}
	}
	return best_hop_index;
}

void
LSF::probSend(const fib::NextHopList& nexthoplist,
						const FaceEndpoint& ingress,
                        const Interest& interest,
                        const shared_ptr<pit::Entry>& pitEntry,
						int best_hop_index) {
	std::vector<double> probabilities = caculateHopProb(nexthoplist, ingress, interest, pitEntry);
	std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // 复制概率分布并进行排序
    std::vector<double> sortedProbabilities = probabilities;
    std::sort(sortedProbabilities.begin(), sortedProbabilities.end());

    double totalProbability = std::accumulate(sortedProbabilities.begin(), sortedProbabilities.end(), 0.0);
    double randomNum = dist(gen);

    // 使用轮盘法选择概率分布
    double sum = 0.0;
    for (int i = 0; i < sortedProbabilities.size(); ++i) {
        sum += sortedProbabilities[i] / totalProbability;
        if (randomNum <= sum) {
            // 在原始概率分布中查找对应索引
            auto it = std::find(probabilities.begin(), probabilities.end(), sortedProbabilities[i]);
            int idx = std::distance(probabilities.begin(), it);
			if (idx == best_hop_index) {continue;}
			auto egress = FaceEndpoint(nexthoplist[idx].getFace(), 0);
			NFD_LOG_DEBUG("do Send Interest="<<interest << " from=" << ingress << "to=" << egress<<" prob=" << probabilities[idx]);
			this->sendInterest(pitEntry, egress, interest);
			this->updateISR(egress, interest, 0, m_isr);
        }
    }
}

void LSF::updateISR(const FaceEndpoint& egress,
                               const Interest& interest,
							   int type,
							   std::map<std::string, std::vector<int>>& isr)
{	
	std::string name = interest.getName().toUri();
	if(isr.find(name) == isr.end()){
		isr[name]={0,0};
	}
	if(type == 0){
		isr[name][0] += 1;
	}
	else{
		isr[name][1] += 1;
	}
		// for(auto it=isr.begin();it != isr.end();++it) {
		// 	std::string name=it->first;
		// 	NS_LOG_INFO("name="<<name<<" ,Inte_num="<<isr[name][0]<<" ,Data_num="<<isr[name][1]<<" ,ISR="<<isr[name][1]/isr[name][0]);
		// }
}

void LSF::updateHopList(int nodeId, const Interest& interest) 
{NFD_LOG_INFO("updateHopList");
	uint32_t nonce = interest.getNonce();
    for (uint32_t i=0;i<m_nodes.GetN();++i) {
		ns3::Ptr<ns3::Node> node = m_nodes[i];
		for (uint32_t j = 0; j < node->GetNApplications(); ++j) {
            ns3::Ptr<ns3::Application> app = node->GetApplication(j);
            if (app->GetInstanceTypeId().GetName() =="ns3::ndn::ConsumerZipfMandelbrot") {
				ns3::Ptr<ns3::ndn::L3Protocol> ndn = node->GetObject<ns3::ndn::L3Protocol>();
				ndn::Name prefix("/");
				nfd::fw::Strategy& strategy = ndn->getForwarder()->getStrategyChoice().findEffectiveStrategy(prefix);
				nfd::fw::LSF& lsf_strategy = dynamic_cast<nfd::fw::LSF&>(strategy);
				std::map<uint32_t, std::vector<int>>& hop = lsf_strategy.getHOP();
				if (hop.find(nonce) == hop.end()){
					if (i==nodeId) {
						lsf_strategy.setHopList(nonce, nodeId, hop, true);
					}
				}
				else{
					lsf_strategy.setHopList(nonce, nodeId, hop, false);
				}
				// for( auto a = lsf_strategy.getHOP().begin();a!=lsf_strategy.getHOP().end();++a){
				// 	uint32_t  x=a->first;
				// 	std::vector<int> hops=a->second;
				// 	for(int i=0;i<hops.size();++i){NFD_LOG_INFO("HopList="<<hops[i]);}
				// }
            }
		}
	}
}

void
LSF::setHopList(uint32_t nonce, int nodeId, std::map<uint32_t, std::vector<int>>& hop, bool isinitial) {
	if(isinitial){
		hop[nonce] = {nodeId};
	}
	else{
		hop[nonce].push_back(nodeId);
	}
}

void LSF::initial(uint32_t num,
                      std::map<std::string, std::vector<int>>& isr) {
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

}  // namespace fw
}  // namespace nfd
