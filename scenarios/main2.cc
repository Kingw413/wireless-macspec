#include "annotated-topology-reader-m.hpp"
#include "generic-link-service-m.hpp"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/ndnSIM/NFD/daemon/face/face-common.hpp"
#include "ns3/ndnSIM/apps/ndn-consumer-batches.hpp"
#include "ns3/ndnSIM/helper/ndn-link-control-helper.hpp"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-cs-tracer.hpp"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/node.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ptr.h"
#include "ns3/rectangle.h"
#include "ns3/ssid.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-module.h"
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
    STAnodes.Create(1);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();

    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    Ssid ssid = Ssid("c0-ap");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n_5GHZ);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                                 StringValue("HtMcs7"), "ControlMode",
                                 StringValue("HtMcs0"));
    // wifiPhy->SetAttribute ("ChannelNumber", UintegerValue (42));
    phy.Set("ChannelNumber", UintegerValue(42));
    // wifi.SetRemoteStationManager ("ns3::ArfWifiManager");
    //    Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
    //    StringValue ("OfdmRate6Mbps"));

    // wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
    // "DataMode", StringValue ("HtMcs0"), // 传输数据的速率
    // "ControlMode", StringValue ("HtMcs0")); // 传输控制信息的速率

    // phy.SetChannel (WifiChannel::CreateFrequency (2400));
    // phy.Set ("ChannelWidth", UintegerValue (20));

    // "Txop",PointerValue(CreateObject<ns3::Txop>());
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing",
                BooleanValue(true));
    NetDeviceContainer staDevices;
    staDevices = wifi.Install(phy, mac, STAnodes);

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));

    apDevices = wifi.Install(phy, mac, APnode);

    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 0.0));
    pos->Add(Vector(10.0, 0.0, 0.0));
    MobilityHelper mobility_STA;
    mobility_STA.SetPositionAllocator(pos);
    // mobility_STA.SetMobilityModel("ns3::RandomWalk2dMobilityModel", "Bounds",
    //                               RectangleValue(Rectangle(-50, 0, -50,
    //                               50)));
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
        "/home/user/ndnsim-2.8/wireless/scenarios/topo.txt");
    topologyReader.Read();
    NodeContainer allNodes = topologyReader.GetNodes();

    NodeContainer stanodes = CreateAP_STA(allNodes[0]);

    // 设置网络设备的带宽为50Mbps
    //   Config::SetDefault ("ns3::WifiRemoteStationManager::DataMode",
    //   StringValue ("HtMcs0")); Config::SetDefault
    //   ("ns3::WifiRemoteStationManager::ControlMode", StringValue ("HtMcs0"));
    //   Config::SetDefault ("ns3::WifiPhy::ChannelWidth", UintegerValue (20));
    //   Config::SetDefault ("ns3::WifiPhy::RxsenseThreshold", DoubleValue
    //   (-80.0)); Config::SetDefault ("ns3::WifiPhy::TxPowerStart", DoubleValue
    //   (20.0)); Config::SetDefault ("ns3::WifiPhy::TxPowerEnd", DoubleValue
    //   (20.0));

    // Install NDN stack on all nodes
    ndn::StackHelper ndnHelper;
    ndnHelper.AddFaceCreateCallback(
        WifiNetDevice::GetTypeId(),
        MakeCallback(&ns3::WifiApStaDeviceCallback));
    ndnHelper.SetLinkDelayAsFaceMetric();
    ndnHelper.SetDefaultRoutes(false);
    ndnHelper.setCsSize(50);
    ndnHelper.InstallAll();
    std::cout << "Install stack\n";

    // Routing strategy
    ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
    ndnGlobalRoutingHelper.InstallAll();
    ndnGlobalRoutingHelper.AddOrigin("/ustc", "p0");
    ndnGlobalRoutingHelper.CalculateRoutes();
    std::cout << "Install routing\n";

    ndn::StrategyChoiceHelper::InstallAll(
        "/ustc", "/localhost/nfd/strategy/best-route/%FD%05");
    std::cout << "Install strategy\n";

    // Installing Consumer
    ndn::AppHelper consumer("ns3::ndn::ConsumerCbr");
    consumer.SetAttribute("Frequency", DoubleValue(1000.0));
    consumer.SetAttribute("Randomize", StringValue("exponential"));
    // consumer.SetAttribute("NumberOfContents", UintegerValue(500));
    // consumer.SetAttribute("MaxSeq", IntegerValue(1));
    consumer.SetPrefix("/ustc/1");
    ApplicationContainer consumercontainer = consumer.Install(stanodes[0]);
    // consumer.SetPrefix("/ustc/2");
    // consumercontainer.Add(consumer.Install(stanodes[1]));

    ndn::AppHelper producer("ns3::ndn::Producer");
    producer.SetPrefix("/ustc");
    producer.SetAttribute("PayloadSize", UintegerValue(1024));
    // producer.SetAttribute("Freshness", TimeValue(ns3::Seconds(0)));
    auto producercontainer = producer.Install("p0");
    std::cout << "Install producer\n";

    std::cout << "Install consumers in " << consumercontainer.GetN()
              << " nodes and producers in " << producercontainer.GetN()
              << " nodes" << std::endl;

    ndn::AppDelayTracer::Install(stanodes[0], "delay0.log");
    // ndn::AppDelayTracer::Install(stanodes[1], "delay1.log");
    ndn::CsTracer::InstallAll("cs.log", MilliSeconds(1000));

    Simulator::Stop(Seconds(20));

    if (1) {
        phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        // pointToPoint.EnablePcapAll ("main");
        phy.EnablePcap("main", apDevices.Get(0));
        // csma.EnablePcap ("third", csmaDevices.Get (0), true);
    }

    // AnimationInterface::SetConstantPosition ( apDevices.Get (0), 10, 30); //
    // AnimationInterface::SetNodeDescription ( apDevices.Get (0), "AP-sta"); //
    // Optional
    // //AnimationInterface::SetNodeColor (csmaNodes, 0, 0, 255); // Optional
    // AnimationInterface::SetBoundary (0, 0, 35, 35); // Optional
    // AnimationInterface anim ("wireless-animation.xml"); // Mandator
    // anim.EnablePacketMetadata (); // Optional
    // anim.EnableIpv4RouteTracking ("main.xml", Seconds (0), Seconds (50),
    // Seconds (0.25));

    Simulator::Run();
    Simulator::Destroy();
    std::cout << "end" << std::endl;
    return 0;
}

}  // namespace ns3

int main(int argc, char *argv[]) { return ns3::main(argc, argv); }
