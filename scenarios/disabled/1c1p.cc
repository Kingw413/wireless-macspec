#include "consumerCCs.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/ndnSIM/helper/ndn-link-control-helper.hpp"
#include "ns3/ptr.h"
#include "string"
#include "random"
#define LOG_DATA 0b1
#define LOG_TIMEOUT 0b10
#define LOG_NACK 0b100
#define LOG_LEARNING 0b1000
#define LOG_REQ 0b10000
double globalbw = 0;

namespace ns3
{
    /*
    void changeBWFunc(Ptr<Node> n0, Ptr<Node> n1, int cur_BW, const int to_BW, Watchdog *wd)
    {
        if (cur_BW > to_BW)
            cur_BW -= 1;
        else if (cur_BW < to_BW)
            cur_BW += 1;
        else
            return;

        std::random_device rdev;
        std::mt19937 reng(rdev());
        std::uniform_int_distribution<> u2(25, 50);
        ChannelList allLinks;
        for (int i = 0; i < allLinks.GetNChannels(); i++)
        {
            Ptr<ns3::PointToPointChannel> p2plink = StaticCast<ns3::PointToPointChannel>(allLinks.GetChannel(i));
            if (p2plink->GetNDevices() != 2)
                continue;
            auto dev0 = p2plink->GetDevice(0), dev1 = p2plink->GetDevice(1);
            if ((dev0->GetNode() == n0 && dev1->GetNode() == n1) || (dev0->GetNode() == n1 && dev1->GetNode() == n0))
            {
                string randBW = to_string(cur_BW) + "Mbps";
                if (dev0->SetAttributeFailSafe("DataRate", StringValue(randBW)))
                {
                    std::cout << ns3::Simulator::Now().GetSeconds() << std::endl;
                    std::cout << "\033[31;43m"
                              << "set link-" << p2plink->GetId() << " device-" << dev0 << " BW-" << randBW << "\033[0m" << std::endl;
                }
                if (dev1->SetAttributeFailSafe("DataRate", StringValue(randBW)))
                    std::cout << "\033[31;43m"
                              << "set link-" << p2plink->GetId() << " device-" << dev1 << " BW-" << randBW << "\033[0m" << std::endl;
            }
        }
        wd->Ping(Seconds(0.1));
        wd->SetArguments(n0, n1, cur_BW, to_BW, wd);
    }
    */

    void changeDLFunc(int cur_BW, const int to_BW, Watchdog *wd)
    {
        if (cur_BW > to_BW)
            cur_BW -= 1;
        else if (cur_BW < to_BW)
            cur_BW += 1;
        else
            return;

        ChannelList allLinks;
        Ptr<ns3::PointToPointChannel> p2plink = StaticCast<ns3::PointToPointChannel>(allLinks.GetChannel(1));
        std::string dl = to_string(cur_BW) + "ms";
        if (p2plink->SetAttributeFailSafe("Delay", StringValue(dl)))
            std::cout << "set link-" << p2plink->GetId() << " delay-" << dl << std::endl;

        wd->Ping(ns3::Seconds(0.2));
        wd->SetArguments(cur_BW, to_BW, wd);
    }

    int main(int argc, char *argv[])
    {
        /*
        // setting default parameters for PointToPoint links and channels
        Config::SetDefault("ns3::PointToPointNetDevice::DataRate", StringValue("2Mbps"));
        Config::SetDefault("ns3::PointToPointChannel::Delay", StringValue("10ms"));
        Config::SetDefault("ns3::QueueBase::MaxSize", StringValue("20p"));
        // Creating nodes
        NodeContainer nodes;
        nodes.Create(3);
        // Connecting nodes using two links
        PointToPointHelper p2p;
        p2p.Install(nodes.Get(0), nodes.Get(1));
        p2p.Install(nodes.Get(1), nodes.Get(2));
        */
        // Read optional command-line parameters (e.g., enable visualizer with ./waf --run=<> --visualize

        //Set ID and PoolSize of shared memory(shm) of ns3-ai
        Config::SetGlobalFailSafe("SharedMemoryKey", UintegerValue(1234));
        Config::SetGlobalFailSafe("SharedMemoryPoolSize", UintegerValue(1040));

        //Use system entropy source and mersenne twister engine
        std::random_device rdev;
        //Randomize initial window
        std::mt19937 reng(rdev());
        std::uniform_int_distribution<> u0(1, 10);
        string str = std::to_string(u0(reng));
        //Randomize seed
        reng.seed(rdev());
        std::uniform_int_distribution<> u1(0);
        ns3::SeedManager smgr;
        smgr.SetSeed(u1(reng));
        //Ransomize BW
        reng.seed(rdev());
        std::uniform_int_distribution<> u2(25, 50);

        CommandLine cmd;
        cmd.Parse(argc, argv);

        Watchdog changedelay;
        changedelay.Ping(Seconds(30));
        changedelay.SetFunction(changeDLFunc);
        changedelay.SetArguments(10, 30, &changedelay);

        AnnotatedTopologyReader topologyReader("", 0.2);
        topologyReader.SetFileName("/root/ndn/proj-sep/scenarios/topo_baseline.txt");
        topologyReader.Read();
        NodeContainer allNodes = topologyReader.GetNodes();
        Ptr<Node> c0 = allNodes[0];
        Ptr<Node> r0 = allNodes[1];
        Ptr<Node> r1 = allNodes[2];
        Ptr<Node> p0 = allNodes[3];

        // Install NDN stack on all nodes
        ndn::StackHelper ndnHelper;
        //ndnHelper.SetDefaultRoutes(true);
        ndnHelper.InstallAll();

        // Routing strategy
        ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
        ndnGlobalRoutingHelper.InstallAll();
        ndnGlobalRoutingHelper.AddOrigins("/ustc", p0);
        ndnGlobalRoutingHelper.CalculateRoutes();

        // Forwarding strategy
        //ndn::StrategyChoiceHelper::Install(allNodes[3], "/ustc", "/localhost/nfd/strategy/best-route2-conges/%FD%01");
        ndn::StrategyChoiceHelper::Install(allNodes[1], "/ustc", "/localhost/nfd/strategy/best-route2-conges/%FD%01");
        //ndn::StrategyChoiceHelper::Install(allNodes[1], "/ustc", "/localhost/nfd/strategy/best-route/%FD%05");
        ndn::StrategyChoiceHelper::Install(allNodes[2], "/ustc", "/localhost/nfd/strategy/best-route/%FD%05");
        ndn::StrategyChoiceHelper::Install(c0, "/ustc", "/localhost/nfd/strategy/best-route/%FD%05");
        ndn::StrategyChoiceHelper::Install(p0, "/ustc", "/localhost/nfd/strategy/best-route/%FD%05");

        // Installing Consumer
        ndn::AppHelper consumerHelper("ns3::ndn::ConsumerCCs");
        consumerHelper.SetPrefix("/ustc/0");
        consumerHelper.SetAttribute("RetxTimer", StringValue("10ms"));
        consumerHelper.SetAttribute("Window", StringValue("2"));
        consumerHelper.SetAttribute("CcAlgorithm", EnumValue(ndn::CCType::ECP));
        consumerHelper.SetAttribute("InitialWindowOnTimeout", BooleanValue(true));
        consumerHelper.SetAttribute("Frequency", DoubleValue(0)); //10000 ecp
        consumerHelper.SetAttribute("Randomize", StringValue("none"));
        consumerHelper.SetAttribute("ShmID", IntegerValue(500));
        consumerHelper.SetAttribute("WatchDog", DoubleValue(0.2));
        consumerHelper.SetAttribute("LogMask", IntegerValue(LOG_DATA | LOG_TIMEOUT | LOG_NACK | LOG_REQ));
        consumerHelper.SetAttribute("Log2File", StringValue("/root/ndn/proj-sep/log/spec/delay-ecp-2.log"));
        consumerHelper.Install(c0);

        ndn::AppHelper backgroudC("ns3::ndn::ConsumerCCs");
        backgroudC.SetPrefix("/ustc/backgroud");
        backgroudC.SetAttribute("CcAlgorithm", EnumValue(ndn::CCType::none));
        backgroudC.SetAttribute("Window", StringValue("30000"));
        backgroudC.SetAttribute("Frequency", StringValue("300"));
        backgroudC.SetAttribute("Randomize", StringValue("uniform"));
        backgroudC.SetAttribute("LogMask", IntegerValue()); //LOG_DATA | LOG_TIMEOUT | LOG_NACK | LOG_REQ));
        backgroudC.SetAttribute("Log2File", StringValue(""));
        backgroudC.Install(c0);

        // Installing Producer
        ndn::AppHelper producerHelper("ns3::ndn::Producer");
        producerHelper.SetPrefix("/ustc");
        producerHelper.SetAttribute("PayloadSize", StringValue("1024"));
        producerHelper.Install(p0);

        /*
        int BWs[] = {50, 30, 48, 28, 50, 36, 48, 31, 49, 35, 28, 48, 38, 49, 40, 31, 49, 40, 29, 38, 47};
        double TPs[] = {0, 12, 20, 30, 42, 50, 58, 70, 80, 84, 90, 105, 110, 116, 130, 140, 157, 170, 180, 185, 193};
        Watchdog changeBW[20];
        for (int i = 0; i < 20; i++)
        {
            //changeBW[i].Ping(Seconds(TPs[i + 1]));
            changeBW[i].SetFunction(changeBWFunc);
            changeBW[i].SetArguments(r0, r1, BWs[i], BWs[i + 1], &changeBW[i]);
        }
        */

        Simulator::Stop(Seconds(150.0));
        Simulator::Run();
        Simulator::Destroy();
        return 0;
    }

} // namespace ns3

int main(int argc, char *argv[])
{
    return ns3::main(argc, argv);
}
