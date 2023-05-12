#include "annotated-topology-reader-m.hpp"
#include "generic-link-service-m.hpp"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-module.h"
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

NS_LOG_COMPONENT_DEFINE("ndn.WirelessSimulation");

std::ofstream  outRx1("Rx0.log");
std::ofstream  outRx2("Rx1.log");

namespace ns3 {
    NetDeviceContainer staDevices;
    NetDeviceContainer apDevices;


//位置回调函数
void showPosition(NodeContainer nodes, double deltaTime) {
    cout.precision(3);
    std::cout.setf(std::ios::fixed);
    for(NodeContainer::Iterator i = nodes.Begin(); i != nodes.End(); ++i){
        Ptr<Node> node = *i;
        string nodeName = Names::FindName(node);
        Ptr<MobilityModel> mobModel = node->GetObject<MobilityModel>();
        Vector3D pos = mobModel->GetPosition();
        Vector3D speed = mobModel->GetVelocity();
        std::cout << "Time " << Simulator::Now().GetSeconds() << " Node "
                << nodeName << ": Position(" << pos.x << ", " << pos.y << ", "
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

int main(int argc, char* argv[]) {

    extern shared_ptr<::nfd::Face> WifiApStaDeviceCallback(
        Ptr<Node> node, Ptr<ndn::L3Protocol> ndn, Ptr<NetDevice> device);

    AnnotatedTopologyReaderM topologyReader("", 1);
    topologyReader.SetFileName("/home/whd/ndnSIM2.8/wireless-macspec/scenarios/topo.txt");
    topologyReader.Read();
    NodeContainer allNodes = topologyReader.GetNodes();

    YansWifiPhyHelper phyHelper;
    NodeContainer staNodes;
    Ptr<Node> apNode1 = allNodes[0];
    Ptr<Node> apNode2 = allNodes[1];
    NodeContainer apNodes;
    apNodes.Add("c0");
    apNodes.Add("c1");
    WifiHelper wifi;

    staNodes.Create(1);
    Names::Add("sta1", staNodes.Get(0));
    // Names::Add("sta2", staNodes.Get(1));
    // Names::Add("sta3", staNodes.Get(2));
    // Names::Add("sta4", staNodes.Get(3));

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default();
    Ptr<YansWifiChannel> channel = channelHelper.Create();
    Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
    // lossModel ->SetRss(-20);
    lossModel->SetReference(1, 60.00);
    lossModel -> SetPathLossExponent(1);
    channel -> SetPropagationLossModel(lossModel);

    phyHelper.Set("TxPowerStart", DoubleValue(0));
    phyHelper.Set("TxPowerEnd", DoubleValue(0));

    phyHelper.SetChannel(channel);
    wifi.SetStandard(WIFI_STANDARD_80211n_5GHZ);
    phyHelper.Set("ChannelNumber", UintegerValue(38));
    phyHelper.Set("ChannelWidth", UintegerValue(40));
// #pragma GCC endregion 物理层配置


    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                                 StringValue("HtMcs7"), "ControlMode",
                                 StringValue("HtMcs7"));

    WifiMacHelper staMac1, apMac1, apMac2;
    Ssid ssid1 = Ssid("c0-ap1");
    Ssid ssid2 = Ssid("c0-ap2");



    staMac1.SetType("ns3::StaWifiMac","Ssid", SsidValue(ssid2));

    apMac1.SetType(
        "ns3::ApWifiMac", "Ssid", SsidValue(ssid1), 
        "EnableBeaconJitter", BooleanValue(false), 
        // "VO_MaxAmpduSize", UintegerValue(65535),
        // "BK_MaxAmpduSize", UintegerValue(65535), 
        "EnableNonErpProtection", BooleanValue(false),  // 此项是必须的，否则无法收到包，原因暂时未知
        "ShortSlotTimeSupported", BooleanValue(false));

    apMac2.SetType(
        "ns3::ApWifiMac", "Ssid", SsidValue(ssid2), 
        "EnableBeaconJitter", BooleanValue(false), 
        // "VO_MaxAmpduSize", UintegerValue(65535),
        // "BK_MaxAmpduSize", UintegerValue(65535), 
        "EnableNonErpProtection", BooleanValue(false),  // 此项是必须的，否则无法收到包，原因暂时未知
        "ShortSlotTimeSupported", BooleanValue(false));

    staDevices = wifi.Install(phyHelper, staMac1, staNodes);
    apDevices = wifi.Install(phyHelper, apMac1, apNode1); 
    apDevices.Add(wifi.Install(phyHelper, apMac2, apNode2));

    //设置初始位置
    // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>(); 
    // positionAlloc->Add(Vector(0, 0, 0));
    // positionAlloc -> Add(Vector(1,0,0));
    // positionAlloc ->Add(Vector(0,1,0));
    // positionAlloc ->Add(Vector(0,-1,0));

    // MobilityHelper mobility_STA;
    // mobility_STA.SetPositionAllocator(positionAlloc);
    // mobility_STA.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    // mobility_STA.Install(staNodes);



    // ConstantPosition模型
    // Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel>( );
    // mobility->SetPosition(Vector(0, 0, 0));
    // staNodes[0]->AggregateObject(mobility); 


    // ConstantVelocity模型
    Ptr<ConstantVelocityMobilityModel> mobility = CreateObject<ConstantVelocityMobilityModel>();
    mobility->SetPosition(Vector(-50, 0, 0));
    mobility->SetVelocity(Vector(10, 0, 0));
    staNodes[0]->AggregateObject(mobility); 

    //Waypoint模型
    // Ptr<WaypointMobilityModel> mobility = CreateObject<WaypointMobilityModel>( );
    // for(int i=0;i<10;++i){
    //     double time = 0+2.001*i;
    //     double x = -pow(1.0233,i);
    //     mobility -> AddWaypoint(Waypoint(Seconds(time), Vector(x, 0, 0)));
    //     mobility -> AddWaypoint(Waypoint(Seconds(time+2.0), Vector(x, 0, 0)));
    // }
    // staNodes[1] -> AggregateObject(mobility);


    CommandLine cmd;
    cmd.Parse(argc, argv);

    // Install NDN stack on all nodes
    ndn::StackHelper ndnHelper;
    ndnHelper.AddFaceCreateCallback(WifiNetDevice::GetTypeId(),
                                    MakeCallback(&WifiApStaDeviceCallback));
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

    ndn::StrategyChoiceHelper::InstallAll("/ustc",
                                          "/localhost/nfd/strategy/best-route");

    std::cout << "Install strategy\n";

    // Installing Consumer
    ndn::AppHelper consumer("ns3::ndn::ConsumerCbr");
    consumer.SetAttribute("Frequency", DoubleValue(10000.0));
    consumer.SetAttribute("Randomize", StringValue("none"));
    consumer.SetPrefix("/ustc/1");
    ApplicationContainer consumercontainer = consumer.Install(staNodes[0]);
    std::cout << "Install consumer\n";

    // Installing Producer
    ndn::AppHelper producer("ns3::ndn::Producer");
    producer.SetPrefix("/ustc");
    producer.SetAttribute("PayloadSize", UintegerValue(1024));
    auto producercontainer = producer.Install("p0");
    std::cout << "Install producer\n";

    std::cout << "Install consumers in " << consumercontainer.GetN()
              << " nodes and producers in " << producercontainer.GetN()
              << " nodes" << std::endl;

    ndn::AppDelayTracer::Install(staNodes[0], "delay0.log");
    // ndn::AppDelayTracer::Install(staNodes[1], "delay1.log");
    // ndn::AppDelayTracer::Install(staNodes[2], "delay2.log");
    // ndn::AppDelayTracer::Install(staNodes[3], "delay3.log");

    ndn::CsTracer::InstallAll("cs.log", MilliSeconds(1000));


    //  staNodes[0] ->GetObject<RandomWalk2dMobilityModel>( )
    //  ->TraceConnectWithoutContext("CourseChange",
    //  MakeCallback(&PrintNodePosition));
    Simulator::Schedule(Seconds(0.0), &showPosition, staNodes, double(1.0));
    Simulator::Schedule(Seconds(0.0), &showPosition, apNodes, double(1.0));
    // Simulator::Schedule(Seconds(0.0), &MyRxCallback);

    // Ptr<WifiPhy> sta1Phy = staDevices.Get(0) -> GetObject<WifiNetDevice>( ) -> GetPhy( );
    Ptr<WifiPhy> ap1Phy = apDevices.Get(0) -> GetObject<WifiNetDevice>( ) -> GetPhy( );
    Ptr<WifiPhy> ap2Phy = apDevices.Get(1) -> GetObject<WifiNetDevice>( ) -> GetPhy( );



    ap1Phy -> TraceConnectWithoutContext("MonitorSnifferRx", MakeCallback (&MyRxCallback1));
    ap2Phy -> TraceConnectWithoutContext("MonitorSnifferRx", MakeCallback (&MyRxCallback2));

    Simulator::Stop(Seconds(10));
    Simulator::Run();
    Simulator::Destroy();
    std::cout << "end" << std::endl;
    return 0;
}
}  // namespace ns3

int main(int argc, char* argv[]) { return ns3::main(argc, argv); }