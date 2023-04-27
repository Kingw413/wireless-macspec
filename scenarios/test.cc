#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"

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

std::ofstream outRx("outRx.log");

namespace ns3 {
    NetDeviceContainer staDevices;
    NetDeviceContainer apDevices;

// 定义回调函数
/* void PrintNodePosition( Ptr<const MobilityModel> model) {
//   std::cout << "Node position at " << time.GetSeconds() << "s: " <<
model->GetPosition() << std::endl; NS_LOG_UNCOND ( " position=" <<
model->GetPosition ());
}
 */

//位置回调函数
void showPosition(Ptr<Node> node, double deltaTime) {
    string nodeName = Names::FindName(node);
    Ptr<MobilityModel> mobModel = node->GetObject<MobilityModel>();
    Vector3D pos = mobModel->GetPosition();
    Vector3D speed = mobModel->GetVelocity();
    std::cout << "Time " << Simulator::Now().GetSeconds() << " Node "
              << nodeName << ": Position(" << pos.x << ", " << pos.y << ", "
              << pos.z << ");   Speed(" << speed.x << ", " << speed.y << ", "
              << speed.z << ")" << std::endl;

    Simulator::Schedule(Seconds(deltaTime), &showPosition, node, deltaTime);
}


//接收功率回调函数
void MyRxCallback(Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{
    double rssi = signalNoise.signal;
    double noise = signalNoise.noise;
    double snr = rssi - noise;
    // uint8_t txPower = txVector.GetTxPowerLevel();
    // int txPower= static_cast<int>(txVector.GetTxPowerLevel());
    // double txPower = 0.0;
    // uint8_t txPowerLevel = txVector.GetTxPowerLevel();
    // std::vector<double> powers = mode.;
    // if (txPowerLevel < powers.size()) {
    //     txPower = powers[txPowerLevel];
    // }
    outRx.precision(2);
    outRx.setf(std::ios::fixed);
    outRx << "Time " << Simulator::Now().GetSeconds() <<"\t"
    // <<"Tx Power = "<< txPower << "dBm, "
    <<" Received packet with RSSI = " << rssi << " dBm, Noise = " << noise << " dBm, SNR = " << snr << " dB" << std::endl;
    //  Simulator::Schedule(Seconds(1.0), &MyRxCallback,  packet,  channelFreqMhz,  txVector,  aMpdu,  signalNoise,  staId);
}

// void MyTimerCallback()
// {
//     // 触发回调函数
//     Ptr<WifiPhy> staPhy = staDevices.Get(0)->GetObject<WifiNetDevice>()->GetPhy();
//     staPhy -> TraceConnectWithoutContext("MonitorSnifferRx", MakeCallback (&MyRxCallback));
//     // 设置下一次触发时间
//     Simulator::Schedule(Seconds(1.0), &MyTimerCallback);
// };

int main(int argc, char* argv[]) {

    extern shared_ptr<::nfd::Face> WifiApStaDeviceCallback(
        Ptr<Node> node, Ptr<ndn::L3Protocol> ndn, Ptr<NetDevice> device);

    AnnotatedTopologyReaderM topologyReader("", 1);
    topologyReader.SetFileName("/home/whd/ndnSIM2.8/wireless-macspec/scenarios/topo.txt");
    topologyReader.Read();
    NodeContainer allNodes = topologyReader.GetNodes();

    YansWifiPhyHelper phy;
    NodeContainer STAnodes;
    Ptr<Node> APnode = allNodes[0];
    WifiHelper wifi;

    STAnodes.Create(1);
    Names::Add("sta1", STAnodes.Get(0));
// Names::Add("sta2", STAnodes.Get(1));
#pragma region 物理层配置
    // Config::SetDefault("ns3::YansWifiChannel::PropagationLossModel",StringValue("ns3::FixedRssLossModel"));  // 这个写法无效

    //更改模型默认参数
    // Config::SetDefault("ns3::FixedRssLossModel::Rss", DoubleValue(-20.0)); 
    // Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceDistance", DoubleValue(1.0));
    // Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(40.0));
    // Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponet", DoubleValue(1.0));

    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default();
    // channelHelper.AddPropagationLoss("ns3::FixedRssLossModel","Rss",DoubleValue (-10));
    // channelHelper.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
    //                                                                 "Exponent", DoubleValue(1),
    //                                                                 "ReferenceDistance", DoubleValue(1),
    //                                                                 "ReferenceLoss", DoubleValue(40));

    Ptr<YansWifiChannel> channel = channelHelper.Create();
    Ptr<FixedRssLossModel> lossModel = CreateObject<FixedRssLossModel>();
    lossModel ->SetRss(-20);
    // // lossModel->SetReference(1, 40);
    // // lossModel -> SetPathLossExponent(1);
    channel -> SetPropagationLossModel(lossModel);

    // phy.Set("TxPowerStart", DoubleValue(60));
    // phy.Set("TxPowerEnd", DoubleValue(60));

    phy.SetChannel(channel);
    wifi.SetStandard(WIFI_STANDARD_80211n_5GHZ);
    phy.Set("ChannelNumber", UintegerValue(38));
    phy.Set("ChannelWidth", UintegerValue(40));
#pragma endregion 物理层配置


#pragma region RSM配置
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                                 StringValue("HtMcs7"), "ControlMode",
                                 StringValue("HtMcs7"));
#pragma endregion RSM配置

#pragma region MAC层配置
    WifiMacHelper staMac, apMac;
    Ssid ssid = Ssid("c0-ap");

    staMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid),
                   "VO_MaxAmpduSize", UintegerValue(65535), "BK_MaxAmpduSize",
                   UintegerValue(65535), "ShortSlotTimeSupported",
                   BooleanValue(false));

    // mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    apMac.SetType(
        "ns3::ApWifiMac", "Ssid", SsidValue(ssid), "EnableBeaconJitter",
        BooleanValue(false), "VO_MaxAmpduSize", UintegerValue(65535),
        "BK_MaxAmpduSize", UintegerValue(65535), "EnableNonErpProtection",
        BooleanValue(false),  // 此项是必须的，否则无法收到包，原因暂时未知
        "ShortSlotTimeSupported", BooleanValue(false));
#pragma endregion MAC层配置

#pragma region 安装WiFi设备
    staDevices = wifi.Install(phy, staMac, STAnodes);
    apDevices = wifi.Install(phy, apMac, APnode); 
#pragma endregion 安装WiFi设备

    //设置初始位置
    // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>(); 
    // positionAlloc->Add(Vector(5, 0, 0));
    // MobilityHelper mobility_STA;
    // mobility_STA.SetPositionAllocator(positionAlloc);
    // mobility_STA.Install(APnode);

    //设置AP位置
    Ptr<MobilityModel> mobility_AP = APnode->GetObject<MobilityModel>();
    mobility_AP->SetPosition(Vector(5, 0, 0));


    // ConstantPosition模型
    Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel>( );
    mobility->SetPosition(Vector(0, 0, 0));
    STAnodes[0]->AggregateObject(mobility); 


    // ConstantVelocity模型
    // Ptr<ConstantVelocityMobilityModel> mobility = CreateObject<ConstantVelocityMobilityModel>();
    // mobility->SetPosition(Vector(0, 0, 0));
    // mobility->SetVelocity(Vector(-2, 0, 0));
    // STAnodes[0]->AggregateObject(mobility); 

    //Waypoint模型
    // Ptr<WaypointMobilityModel> mobility = CreateObject<WaypointMobilityModel>( );
    // mobility -> AddWaypoint(Waypoint(Seconds(0.0), Vector(10, -10, 0)));
    // mobility -> AddWaypoint(Waypoint(Seconds(1.0), Vector(10, 10, 0)));
    // mobility -> AddWaypoint(Waypoint(Seconds(2.0), Vector(-10, 10, 0)));
    // mobility -> AddWaypoint(Waypoint(Seconds(3.0), Vector(-10, -10, 0)));
    // STAnodes[0] -> AggregateObject(mobility);

    /*RandomWlak2d模型(法1)
        Ptr<RandomWalk2dMobilityModel> mobility =
       CreateObject<RandomWalk2dMobilityModel>();
        mobility->SetAttribute("Speed",StringValue("ns3::UniformRandomVariable[Min=2|Max=4]")),
        STAnodes[0]->AggregateObject(mobility);
    */

    /* RandomWlak2d模型(法2)
    mobility_STA.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                            //   "Bounds",RectangleValue(Rectangle(-50, 50, -50,
    50)),
                            //   "Time", StringValue("3s"),
                            //   "Direction",
    StringValue("ns3::ConstantRandomVariable[Constant=0]"), "Speed",
    StringValue("ns3::ConstantRandomVariable[Constant=10.0]")
                            //   "Speed",
    StringValue("ns3::UniformRandomVariable[Min=10|Max=11]")
                              );
        mobility_STA.Install(STAnodes[0]);
    */

    std::cout.precision(2);
    std::cout.setf(std::ios::fixed);

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
    ApplicationContainer consumercontainer = consumer.Install(STAnodes[0]);
    // consumer.SetPrefix("ustc/2");
    // consumercontainer.Add(consumer.Install(stanodes[1]));
    std::cout << "Install consumer\n";

    // Installing Producer
    ndn::AppHelper producer("ns3::ndn::Producer");
    producer.SetPrefix("/ustc");
    producer.SetAttribute("PayloadSize", UintegerValue(1024));
    auto producercontainer = producer.Install(allNodes[2]);
    std::cout << "Install producer\n";

    std::cout << "Install consumers in " << consumercontainer.GetN()
              << " nodes and producers in " << producercontainer.GetN()
              << " nodes" << std::endl;

    ndn::AppDelayTracer::Install(STAnodes[0], "delay0.log");
    // ndn::AppDelayTracer::Install(stanodes[1], "delay1.log");
    ndn::CsTracer::InstallAll("cs.log", MilliSeconds(1000));

    //  STAnodes[0] ->GetObject<RandomWalk2dMobilityModel>( )
    //  ->TraceConnectWithoutContext("CourseChange",
    //  MakeCallback(&PrintNodePosition));
    // Simulator::Schedule(Seconds(0.0), &showPosition, STAnodes[0], double(0.5));
    // Simulator::Schedule(Seconds(0.0), &showPosition, APnode, double(0.5));
    // Simulator::Schedule(Seconds(0.0), &MyRxCallback);

    Ptr<WifiPhy> staPhy = staDevices.Get(0) -> GetObject<WifiNetDevice>( ) -> GetPhy( );

    cout<<"TxPowerStart: "<<staPhy ->GetTxPowerStart()<<endl;
    cout<<"TxPowerEnd: "<<staPhy -> GetTxPowerEnd()<<endl;
    staPhy -> TraceConnectWithoutContext("MonitorSnifferRx", MakeCallback (&MyRxCallback));

    Simulator::Stop(Seconds(4));
    Simulator::Run();
    Simulator::Destroy();
    std::cout << "end" << std::endl;
    return 0;
}
}  // namespace ns3

int main(int argc, char* argv[]) { return ns3::main(argc, argv); }
