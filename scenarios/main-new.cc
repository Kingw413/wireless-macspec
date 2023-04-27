#include "annotated-topology-reader-m.hpp"
#include "generic-link-service-m.hpp"
#include "ns3/core-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/ndnSIM/NFD/daemon/face/face-common.hpp"
#include "ns3/ndnSIM/apps/ndn-consumer-batches.hpp"
#include "ns3/ndnSIM/helper/ndn-link-control-helper.hpp"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-cs-tracer.hpp"
#include "ns3/network-module.h"
#include "ns3/node.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ptr.h"
#include "ns3/qos-txop.h"
#include "ns3/rectangle.h"
#include "ns3/ssid.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-helper.h"

namespace ns3 {

extern shared_ptr<::nfd::Face> WifiApStaDeviceCallback(Ptr<Node> node,
                                                       Ptr<ndn::L3Protocol> ndn,
                                                       Ptr<NetDevice> device);

YansWifiPhyHelper phy; 
NetDeviceContainer apDevices;
NodeContainer STAnodes;
NodeContainer CreateAP_STA(Ptr<Node> APnode) {
    WifiHelper wifi;

    STAnodes.Create(1);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    // channel.AddPropagationLoss ("ns3::RangePropagationLossModel");

    channel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (double(-50)));

 /*
    channel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel",
                    "Exponent", DoubleValue (3.0),
                    "ReferenceLoss", DoubleValue (40.0459));
*/

    phy.SetChannel(channel.Create());
    //phy.Set("DsssRate1Mbps", "OfdmRate6Mbps");
    //wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager");
    wifi.SetStandard (WIFI_STANDARD_80211n_5GHZ); 
    //2.4G,5G channeNumer is different
    phy.Set ("ChannelNumber", UintegerValue (38));
    phy.Set ("ChannelWidth", UintegerValue (40));



    // wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                            "DataMode", StringValue("HtMcs7"),
                            "ControlMode", StringValue("HtMcs7")); 


    //                         "MinSupportedTxDataRate", UintegerValue(24),
    //                         "MinSupportedRxDataRate", UintegerValue(24));
    //Config::SetDefault("ns3::WifiRemoteStationManager::MinSupportedTxDataRate", UintegerValue(24));

    //YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  
    // 设置wifi最低传输速率
    //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/SupportedRates", ns3::StringValue("OfdmRate24Mbps"));
  
 

    WifiMacHelper mac;
    Ssid ssid = Ssid("c0-ap");

   
     
     ////wifiPhy->SetAttribute ("ChannelNumber", UintegerValue (42));
    
     //phy.Set ("GreenfieldEnabled", BooleanValue(true));
    // wifi.SetRemoteStationManager ("ns3::ArfWifiManager"); 
    //    Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue ("OfdmRate6Mbps"));

    // wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
    // "DataMode", StringValue ("HtMcs0"), // 传输数据的速率
    // "ControlMode", StringValue ("HtMcs0")); // 传输控制信息的速率

    // phy.SetChannel (WifiChannel::CreateFrequency (2400));
    // phy.Set ("ChannelWidth", UintegerValue (20));

    // "Txop",PointerValue(CreateObject<ns3::Txop>());
    // mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing",
    //             BooleanValue(false));
    NetDeviceContainer staDevices;
    mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
            //    "BE_BlockAckThreshold", UintegerValue (4),
               "VO_MaxAmpduSize", UintegerValue (65535),
               "BK_MaxAmpduSize", UintegerValue (65535),
               "ShortSlotTimeSupported", BooleanValue (false));

   
    staDevices = wifi.Install(phy, mac, STAnodes);

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    
    
  //wifi.SetStandard (WIFI_STANDARD_80211n_5GHZ);
  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid),
               "EnableBeaconJitter", BooleanValue (false),
            //   "BE_BlockAckThreshold", UintegerValue (4),
                "VO_MaxAmpduSize", UintegerValue (65535),
               "BK_MaxAmpduSize", UintegerValue (65535),
               "EnableNonErpProtection", BooleanValue (false),
               "ShortSlotTimeSupported", BooleanValue (false));
  

    apDevices = wifi.Install(phy, mac, APnode);

    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported", BooleanValue(true));
    
    Ptr<NetDevice> dev = APnode->GetDevice (1);
      Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (dev);
      Ptr<WifiMac> wifi_mac = wifi_dev->GetMac ();
      PointerValue ptr;
      wifi_mac->GetAttribute ("BE_Txop", ptr);
    //   Ptr<QosTxop> edca = ptr.Get<QosTxop> ();
    //   edca->SetTxopLimit (MicroSeconds (3008)); affect qos duration and erase cf-end


        // Add the DSSS/CCK rates as the basic rates of our AP
 Ptr<WifiRemoteStationManager> apStationManager =  DynamicCast<WifiNetDevice>(apDevices.Get (0))->GetRemoteStationManager ();
 apStationManager->AddBasicMcs (WifiMode ("HtMcs7"));
//  apStationManager->AddBasicMode (WifiMode ("HtMcs1"));
//  apStationManager->AddBasicMode (WifiMode ("DsssRate5_5Mbps"));
//  apStationManager->AddBasicMode (WifiMode ("DsssRate11Mbps"));

    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 0.0));
    pos->Add(Vector(1.0, 0.0, 0.0));
    MobilityHelper mobility_STA;
    mobility_STA.SetPositionAllocator(pos);
    // mobility_STA.SetMobilityModel("ns3::RandomWalk2dMobilityModel", "Bounds",
    //                               RectangleValue(Rectangle(-50, 0, -50, 50)));
    mobility_STA.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility_STA.Install(STAnodes);

    return STAnodes;
}

int main(int argc, char *argv[]) {
    // Set ID and PoolSize of shared memory(shm) of ns3-ai
    // Config::SetGlobalFailSafe("SharedMemoryKey", UintegerValue(1234));
    // Config::SetGlobalFailSafe("SharedMemoryPoolSize", UintegerValue(1040));

    // Use system entropy source and mersenne twister engine
    // std::random_device rdev;
    // std::mt19937 reng(rdev());
    // std::uniform_int_distribution<> u0(0);
    // ns3::SeedManager smgr;
    // smgr.SetSeed(u0(reng));

    // GlobalValue::Bind("SimulatorImplementationType",
    //                   StringValue("ns3::DefaultSimulatorImpl"));

    CommandLine cmd;
    cmd.Parse(argc, argv);

    AnnotatedTopologyReaderM topologyReader("", 1);
    topologyReader.SetFileName(
        "/home/whd/ndnSIM2.8/wireless-macspec/scenarios/topo.txt");
    topologyReader.Read();
    NodeContainer allNodes = topologyReader.GetNodes();

    NodeContainer stanodes = CreateAP_STA(allNodes[0]);

    // Install NDN stack on all nodes
    ndn::StackHelper ndnHelper;
    ndnHelper.AddFaceCreateCallback(
        WifiNetDevice::GetTypeId(),
        MakeCallback(&ns3::WifiApStaDeviceCallback));
    ndnHelper.SetLinkDelayAsFaceMetric();
    ndnHelper.SetDefaultRoutes(true);
    ndnHelper.setCsSize(50);
    ndnHelper.InstallAll();
    // ndnHelper.SetDefaultRoutes(false);
    // ndnHelper.Install(allNodes);
    std::cout << "Install stack\n";

    // Routing strategy
    ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
    ndnGlobalRoutingHelper.InstallAll();
    ndnGlobalRoutingHelper.AddOrigin("/ustc", "p0");
    ndnGlobalRoutingHelper.CalculateRoutes();
    std::cout << "Install routing\n";

    // ndn::FibHelper::AddRoute(stanodes[0],"/",allNodes[0],0);
    // ndn::FibHelper::AddRoute(allNodes[0],"/",allNodes[1],0);
    // ndn::FibHelper::AddRoute(allNodes[1],"/",allNodes[2],0);

    ndn::StrategyChoiceHelper::InstallAll(
        "/ustc", "/localhost/nfd/strategy/best-route");
    std::cout << "Install strategy\n";

    // Installing Consumer
    ndn::AppHelper consumer("ns3::ndn::ConsumerCbr");
    consumer.SetAttribute("Frequency", DoubleValue(10000.0));
    // consumer.SetAttribute("Randomize", StringValue("exponential"));
    consumer.SetAttribute("Randomize", StringValue("none"));
    // consumer.SetAttribute("NumberOfContents", UintegerValue(500));
    // consumer.SetAttribute("MaxSeq", IntegerValue(1));
    consumer.SetPrefix("/ustc/1");
    ApplicationContainer consumercontainer = consumer.Install(stanodes[0]);
    std::cout << "Install consumer\n";
   // consumer.SetPrefix("/ustc/2");
    // consumercontainer.Add(consumer.Install(stanodes[1]));

    ndn::AppHelper producer("ns3::ndn::Producer");
    producer.SetPrefix("/ustc");
    producer.SetAttribute("PayloadSize", UintegerValue(1024));
    // producer.SetAttribute("Freshness", TimeValue(ns3::Seconds(0)));
    auto producercontainer = producer.Install(allNodes[2]);
    std::cout << "Install producer\n";

    std::cout << "Install consumers in " << consumercontainer.GetN()
              << " nodes and producers in " << producercontainer.GetN()
              << " nodes" << std::endl;

    ndn::AppDelayTracer::Install(stanodes[0], "delay0.log");
    //ndn::AppDelayTracer::Install(stanodes[1], "delay1.log");
    ndn::CsTracer::InstallAll("cs.log", MilliSeconds(1000));

    Simulator::Stop(Seconds(10));

     if (1)
    {
      phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      //pointToPoint.EnablePcapAll ("main");
      phy.EnablePcap ("main-sta", apDevices);
    //   phy.EnablePcap("test-c0-r0", stanodes);
      //csma.EnablePcap ("third", csmaDevices.Get (0), true);
    }

    Simulator::Run();
    Simulator::Destroy();
    std::cout << "end" << std::endl;
    return 0;
}

}  // namespace ns3

int main(int argc, char *argv[]) { return ns3::main(argc, argv); }
