#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include <iostream>

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

#include "annotated-topology-reader-m.hpp"
#include "generic-link-service-m.hpp"

#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/ndnSIM/NFD/daemon/face/face-common.hpp"
#include "ns3/ndnSIM/apps/ndn-consumer-batches.hpp"
#include "ns3/ndnSIM/apps/ndn-producer.hpp"
#include "ns3/ndnSIM/helper/ndn-link-control-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-global-routing-helper.hpp"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/model/ndn-common.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-cs-tracer.hpp"
#include "ns3/network-module.h"
#include "ns3/node.h"
#include "ns3/point-to-point-module.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/ptr.h"
#include "ns3/qos-txop.h"
#include "ns3/rectangle.h"
#include "ns3/ssid.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-phy-state-helper.h"

NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");

std::ofstream  outRx1("Rx0.log");
std::ofstream  outRx2("Rx1.log");
namespace ns3{

// ndn::Face& getFace(Ptr<Node> node)
// {
//   // 获取 NDN 进程
//   Ptr<ndn::L3Protocol> ndn = node->GetObject<ndn::L3Protocol>();

//   // 获取默认 Face
//   return ndn->;
// }
//位置回调函数
void showPosition(NodeContainer nodes, double deltaTime) {
    cout.precision(3);
    std::cout.setf(std::ios::fixed);
    for(NodeContainer::Iterator i = nodes.Begin(); i != nodes.End(); ++i){
        Ptr<Node> node = *i;
        uint32_t nodeId = node->GetId();
        Ptr<MobilityModel> mobModel = node->GetObject<MobilityModel>();
        Vector3D pos = mobModel->GetPosition();
        Vector3D speed = mobModel->GetVelocity();
        std::cout << "Time " << Simulator::Now().GetSeconds() << " Node "
                << nodeId << ": Position(" << pos.x << ", " << pos.y << ", "
                << pos.z << ");   Speed(" << speed.x << ", " << speed.y << ", "
                << speed.z << ")" << std::endl;
    }
    Simulator::Schedule(Seconds(deltaTime), &showPosition, nodes, deltaTime);
}


//接收功率回调函数
void MyRxCallback1(Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{
    double rssi = signalNoise.signal;
    double noise = signalNoise.noise;
    double snr = rssi - noise;
    // string logfile = "Rx" + to_string(staId)+".log";
    // ofstream outRx(logfile, ios::app);
    outRx1.precision(3); 
    outRx1.setf(std::ios::fixed);
    outRx1 << "Time " << Simulator::Now().GetSeconds() <<"\t"
    // <<"Tx Power = "<< txPower << "dBm, "
    <<"sta"<<staId
    <<" Received packet with RSSI = " << rssi << " dBm, Noise = " << noise << " dBm, SNR = " << snr << " dB" << std::endl;
    //  Simulator::Schedule(Seconds(1.0), &MyRxCallback,  packet,  channelFreqMhz,  txVector,  aMpdu,  signalNoise,  staId);
}

void MyRxCallback2(Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{
    double rssi = signalNoise.signal;
    double noise = signalNoise.noise;
    double snr = rssi - noise;
    // string logfile = "Rx" + to_string(staId)+".log";
    // ofstream outRx(logfile, ios::app);
    outRx2.precision(3); 
    outRx2.setf(std::ios::fixed);
    outRx2 << "Time " << Simulator::Now().GetSeconds() <<"\t"
    // <<"Tx Power = "<< txPower << "dBm, "
    <<"sta"<<staId
    <<" Received packet with RSSI = " << rssi << " dBm, Noise = " << noise << " dBm, SNR = " << snr << " dB" << std::endl;
    //  Simulator::Schedule(Seconds(1.0), &MyRxCallback,  packet,  channelFreqMhz,  txVector,  aMpdu,  signalNoise,  staId);
}


int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate54Mbps");

  CommandLine cmd (__FILE__);

  NodeContainer nodes;
  nodes.Create (4);

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
//   if (verbose)
//     {
//       wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
//     }

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);

  // Tracing
//   wifiPhy.EnablePcap ("wave-simple-80211p", devices);

  MobilityHelper mobilityHelper;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (-5.0, 0.0, 0.0));
  positionAlloc ->Add(Vector(0.0, -10.0, 0.0));
  positionAlloc ->Add(Vector(10.0, 0.0, 0.0));
  mobilityHelper.SetPositionAllocator (positionAlloc);
  mobilityHelper.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  NodeContainer constantPosNodes;
  constantPosNodes.Add(nodes[0]);
  constantPosNodes.Add(nodes[2]);
  constantPosNodes.Add(nodes[3]);
  mobilityHelper.Install(constantPosNodes);

  //ConstantVelocity模型 
  Ptr<ConstantVelocityMobilityModel> mobility = CreateObject<ConstantVelocityMobilityModel>();
  mobility->SetPosition(Vector(0, 0, 0));
  mobility->SetVelocity(Vector(0, 5, 0));
  nodes[1]->AggregateObject(mobility);


  // Install NDN stack on all nodes
  extern shared_ptr<::nfd::Face> WifiApStaDeviceCallback(
      Ptr<Node> node, Ptr<ndn::L3Protocol> ndn, Ptr<NetDevice> device);
  ndn::StackHelper ndnHelper;
  ndnHelper.AddFaceCreateCallback(WifiNetDevice::GetTypeId(),
                                  MakeCallback(&WifiApStaDeviceCallback));
  // ndnHelper.SetLinkDelayAsFaceMetric();
  ndnHelper.SetDefaultRoutes(true);
  
  ndnHelper.setCsSize(50);
  ndnHelper.InstallAll();
  std::cout << "Install stack\n";

  //Routing strategy
  ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
  ndnGlobalRoutingHelper.InstallAll();
  ndnGlobalRoutingHelper.AddOrigin("/ustc", nodes[3]);
  ndnGlobalRoutingHelper.CalculateRoutes();
  std::cout << "Install routing\n";
 
  // nodes[1]->get
  // ndn::FibHelper::AddRoute(nodes[0], "/ustc", ,0);
  // ndn::FibHelper::AddRoute(nodes[1], "/ustc", nodes[3],0);
  // ndn::FibHelper::AddRoute(nodes[0], "/ustc", nodes[2],0);
  // ndn::FibHelper::AddRoute(nodes[2], "/ustc", nodes[3],0);
  for(uint32_t nodeId = 0; nodeId< nodes.GetN()-1; ++nodeId){
    ndn::StrategyChoiceHelper::Install(nodes.Get(nodeId), "/", "/localhost/nfd/strategy/lsf/%FD%10");
  }
  ndn::StrategyChoiceHelper::Install(nodes[3],"/","/localhost/nfd/strategy/best-route/%FD%05");

  // Installing Consumer
  ndn::AppHelper consumer("ns3::ndn::ConsumerCbr");
  consumer.SetAttribute("Frequency", DoubleValue(100.0));
  consumer.SetAttribute("Randomize", StringValue("none"));
  consumer.SetPrefix("/ustc");
  ApplicationContainer consumercontainer = consumer.Install(nodes[0]);
  // consumer.SetPrefix("/ustc/2");
  // consumercontainer.Add(consumer.Install(nodes[2]));
  // consumer.SetPrefix("/ustc/3");
  // consumercontainer.Add(consumer.Install(staNodes[2]));
  // consumer.SetPrefix("/ustc/4");
  // consumercontainer.Add(consumer.Install(staNodes[3]));
  std::cout << "Install consumer\n";

  // Installing Producer
  ndn::AppHelper producer("ns3::ndn::Producer");
  producer.SetPrefix("/ustc");
  producer.SetAttribute("PayloadSize", UintegerValue(1024));
  auto producercontainer = producer.Install(nodes[3]);
  // producer.SetPrefix("/ustc/1");
  // producercontainer.Add(producer.Install(nodes[2]));
  std::cout << "Install producer\n";
  std::cout << "Install consumers in " << consumercontainer.GetN()
            << " nodes and producers in " << producercontainer.GetN()
            << " nodes" << std::endl;

  // for(Ptr<Node> node : nodes){
  //   Ptr<ns3::Application> app = node->GetApplication();
  //   if(app->GetObject<ns3::ndn::Producer>() != nullptr){
  //       ndn::StrategyChoiceHelper::Install(node,"/","/localhost/nfd/strategy/lsf/%FD%10");
  //   }
  // };

  ndn::AppDelayTracer::Install(nodes[0], "delay0.log");
  // ndn::AppDelayTracer::Install(nodes[2], "delay1.log");
  // ndn::AppDelayTracer::Install(staNodes[2], "delay2.log");
  // ndn::AppDelayTracer::Install(staNodes[3], "delay3.log");

  ndn::CsTracer::InstallAll("cs.log", MilliSeconds(1000));

  // Simulator::Schedule(Seconds(0.0), &showPosition, nodes, double(1.0));

  // Ptr<WifiPhy> adhoc1Phy = devices.Get(0)->GetObject<WifiNetDevice>()->GetPhy();
  // Ptr<WifiPhy> adhoc2Phy = devices.Get(1)->GetObject<WifiNetDevice>()->GetPhy();

  // adhoc1Phy->TraceConnectWithoutContext("MonitorSnifferRx",
  //                                       MakeCallback(&MyRxCallback1));
  // adhoc2Phy->TraceConnectWithoutContext("MonitorSnifferRx",
  //                                       MakeCallback(&MyRxCallback2));

  Simulator::Stop(Seconds(4));
  Simulator::Run();
  Simulator::Destroy();
  std::cout << "end" << std::endl;
  return 0;
    }
}

int main(int argc, char* argv[]) { return ns3::main(argc, argv); }