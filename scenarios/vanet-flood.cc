#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"

#include "annotated-topology-reader-m.hpp"
#include "generic-link-service-m.hpp"

#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/node.h"
#include "ns3/point-to-point-module.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/ptr.h"
#include "ns3/qos-txop.h"
#include "ns3/rectangle.h"
#include "ns3/ssid.h"
#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/ns2-mobility-helper.h"
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

#include <iostream>
#include <random>


NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");


namespace ns3{

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6Mbps");

  NodeContainer nodes;
  uint32_t N = 70;
  nodes.Create (N);

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default();
  Ptr<YansWifiChannel> channel = channelHelper.Create();
  Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
  lossModel->SetReference(1, 40.00);
  lossModel -> SetPathLossExponent(1);
  channel -> SetPropagationLossModel(lossModel);
  wifiPhy.Set("TxPowerStart", DoubleValue(0));
  wifiPhy.Set("TxPowerEnd", DoubleValue(0));
  wifiPhy.SetChannel(channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));

  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);
  Ns2MobilityHelper ns2Mobiity = Ns2MobilityHelper("/home/whd/ndnSIM2.8/wireless-macspec/scenarios/manhattan.tcl");
  ns2Mobiity.Install();


  // Install NDN stack on all nodes
  extern shared_ptr<::nfd::Face> WifiApStaDeviceCallbackOld(
      Ptr<Node> node, Ptr<ndn::L3Protocol> ndn, Ptr<NetDevice> device);
  ndn::StackHelper ndnHelper;
  ndnHelper.AddFaceCreateCallback(WifiNetDevice::GetTypeId(),
                                  MakeCallback(&WifiApStaDeviceCallbackOld));
  ndnHelper.SetLinkDelayAsFaceMetric();
  ndnHelper.SetDefaultRoutes(true);
  
  ndnHelper.setCsSize(50);
  ndnHelper.InstallAll();
  std::cout << "Install stack\n";
 
//   std::random_device rd;
//   std::mt19937 gen(rd());
//   std::uniform_int_distribution<uint32_t> dist(0,N);
//   uint32_t consumerId = dist(gen);
//   uint32_t producerId = dist(gen);
  uint32_t consumerId = 0;
  uint32_t producerId = 49;
//   for(uint32_t nodeId = 0; nodeId< nodes.GetN(); ++nodeId){
//     if(nodeId != producerId)
//     ndn::StrategyChoiceHelper::Install(nodes.Get(nodeId), "/","/localhost/nfd/strategy/flood/%FD%01");
//   }
//   ndn::StrategyChoiceHelper::Install(nodes.Get(producerId),"/","/localhost/nfd/strategy/best-route/%FD%05");

  ndn::StrategyChoiceHelper::InstallAll("/","/localhost/nfd/strategy/bestTest/%FD%01");

  // Installing Consumer
  // ndn::AppHelper consumer("ns3::ndn::ConsumerCbr");
  // consumer.SetAttribute("Frequency", DoubleValue(100.0));
  // consumer.SetAttribute("Randomize", StringValue("none"));
  ndn::AppHelper consumerHelper("ns3::ndn::ConsumerZipfMandelbrot");
  consumerHelper.SetAttribute("Frequency", StringValue("100"));
  consumerHelper.SetAttribute("NumberOfContents", StringValue("100"));
  consumerHelper.SetAttribute("q", StringValue("0"));
  consumerHelper.SetAttribute("s", StringValue("0.7"));
  consumerHelper.SetPrefix("/ustc");
  ApplicationContainer consumercontainer = consumerHelper.Install(nodes[consumerId]);
  std::cout << "Install consumer\n";

  // Installing Producer
  ndn::AppHelper producer("ns3::ndn::Producer");
  producer.SetPrefix("/ustc");
  producer.SetAttribute("PayloadSize", UintegerValue(1024));
  auto producercontainer = producer.Install(nodes.Get(producerId));
  std::cout << "Install producer\n";
  std::cout << "Install consumers in " << consumercontainer.GetN()
            << " nodes and producers in " << producercontainer.GetN()
            << " nodes" << std::endl;

  ndn::AppDelayTracer::Install(nodes.Get(consumerId), "results/delay_flood.log");
  // ndn::CsTracer::InstallAll("results/cs_flood.log", MilliSeconds(1000));


  Simulator::Stop(Seconds(10));
  Simulator::Run();
  Simulator::Destroy();
  std::cout << "end" << std::endl;
  return 0;
    }
}

int main(int argc, char* argv[]) { 
    // int nodeNum;
    // ns3::CommandLine cmd (__FILE__);
    // cmd.AddValue("nodeNum", "Number of Vehicles", nodeNum);
    // cmd.Parse(argc, argv);
    // std::cout<<argc<<argv[0]<<"NUM="<<nodeNum<<std::endl;
    // if (nodeNum < 50) { std::cout<<"NOTE: Number of Vehicles should not less than 50"<<std::endl; return 0;}
    return ns3::main(argc, argv); }