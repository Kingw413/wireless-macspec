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

NodeContainer CreateAP_STA(Ptr<Node> APnode) {
    NodeContainer STAnodes;
    STAnodes.Create(2);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.AddPropagationLoss("ns3::RangePropagationLossModel");

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("ShortPlcpPreambleSupported", BooleanValue(true));

    WifiMacHelper mac;
    Ssid ssid = Ssid("c0-ap");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n_2_4GHZ);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing",
                BooleanValue(true), "ShortSlotTimeSupported",
                BooleanValue(true));
    NetDeviceContainer staDevices;
    staDevices = wifi.Install(phy, mac, STAnodes);

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "EnableBeaconJitter",
                BooleanValue(false), "BE_BlockAckThreshold", UintegerValue(2),
                "EnableNonErpProtection", BooleanValue(true),
                "ShortSlotTimeSupported", BooleanValue(true));
    NetDeviceContainer apDevices;
    apDevices = wifi.Install(phy, mac, APnode);

    // 设置无线网络设备的标准为802.11n
    Ptr<WifiPhy> wifiPhy =
        DynamicCast<WifiNetDevice>(apDevices.Get(0))->GetPhy();
    // wifiPhy->;

    Ptr<NetDevice> dev = APnode->GetDevice(1);
    Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
    Ptr<WifiMac> wifi_mac = wifi_dev->GetMac();
    PointerValue ptr;
    wifi_mac->GetAttribute("BE_Txop", ptr);
    Ptr<QosTxop> edca = ptr.Get<QosTxop>();
    edca->SetTxopLimit(MicroSeconds(3008));

    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(-1, 0, 0));
    pos->Add(Vector(-1, 0, 0));
    MobilityHelper mobility_STA;
    mobility_STA.SetPositionAllocator(pos);
    // mobility_STA.SetMobilityModel("ns3::RandomWalk2dMobilityModel", "Bounds",
    //                               RectangleValue(Rectangle(-50, 0, -50,
    //                               50)));
    mobility_STA.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility_STA.Install(STAnodes);

    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
        UintegerValue(0));

    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    // pointToPoint.EnablePcapAll ("main");
    phy.EnablePcap("main", apDevices.Get(0));

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
    consumer.SetPrefix("/ustc/2");
    consumercontainer.Add(consumer.Install(stanodes[1]));

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
    ndn::AppDelayTracer::Install(stanodes[1], "delay1.log");
    ndn::CsTracer::InstallAll("cs.log", MilliSeconds(1000));

    Simulator::Stop(Seconds(10));
    Simulator::Run();
    Simulator::Destroy();
    std::cout << "end" << std::endl;
    return 0;
}

}  // namespace ns3

int main(int argc, char *argv[]) { return ns3::main(argc, argv); }
