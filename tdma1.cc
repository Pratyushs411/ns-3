#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TdmaRoundRobinSim");

const uint32_t kNumUes = 10;
const double kSlotDuration = 0.1; // seconds
const double kSimDuration = 10.0; // seconds
const uint32_t kPacketSize = 1024;
const std::string kDataRate = "2Mbps";

void ScheduleUeTraffic(Ptr<Application> app, double startTime, double duration) {
    app->SetStartTime(Seconds(startTime));
    app->SetStopTime(Seconds(startTime + duration));
}

int main(int argc, char *argv[]) {
    CommandLine cmd;
    cmd.Parse(argc, argv);

    NodeContainer bsNode, ueNodes;
    bsNode.Create(1);
    ueNodes.Create(kNumUes);

    // Configure WiFi channel
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    WifiMacHelper mac;

    Ssid ssid = Ssid("tdma-ssid");

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));

    NetDeviceContainer ueDevices = wifi.Install(phy, mac, ueNodes);

    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid));
    NetDeviceContainer bsDevice = wifi.Install(phy, mac, bsNode);

    // Mobility model
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(bsNode);
    mobility.Install(ueNodes);

    // Install Internet stack
    InternetStackHelper internet;
    internet.Install(bsNode);
    internet.Install(ueNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");

    Ipv4InterfaceContainer bsInterface = ipv4.Assign(bsDevice);
    Ipv4InterfaceContainer ueInterfaces = ipv4.Assign(ueDevices);

    // Create UDP server on base station
    uint16_t port = 5000;
    UdpServerHelper server(port);
    ApplicationContainer serverApps = server.Install(bsNode.Get(0));
    serverApps.Start(Seconds(0.0));
    serverApps.Stop(Seconds(kSimDuration));

    // Create UDP clients on each UE with TDMA slotting
    ApplicationContainer clientApps;
    for (uint32_t i = 0; i < kNumUes; ++i) {
        UdpClientHelper client(bsInterface.GetAddress(0), port);
        client.SetAttribute("PacketSize", UintegerValue(kPacketSize));
        client.SetAttribute("MaxPackets", UintegerValue(100000));
        client.SetAttribute("Interval", TimeValue(Seconds(0.01)));

        ApplicationContainer app = client.Install(ueNodes.Get(i));
        clientApps.Add(app);

        // Schedule UE traffic in round-robin fashion
        double currentTime = i * kSlotDuration;
        while (currentTime < kSimDuration) {
            ScheduleUeTraffic(app.Get(0), currentTime, kSlotDuration);
            currentTime += kSlotDuration * kNumUes;
        }
    }

    // Flow Monitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(kSimDuration));
    Simulator::Run();

    // Output results
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    auto stats = monitor->GetFlowStats();

    std::ofstream outFile("tdma_results.csv");
    outFile << "FlowId,Src,Dest,Delay(s),Jitter(s),Throughput(bps),LossRate(%)\n";

    for (const auto& flow : stats) {
        auto t = classifier->FindFlow(flow.first);
        double delay = flow.second.delaySum.GetSeconds() / flow.second.rxPackets;
        double jitter = flow.second.jitterSum.GetSeconds() / flow.second.rxPackets;
        double throughput = flow.second.rxBytes * 8.0 / kSimDuration;
        double lossRate = 100.0 * (flow.second.txPackets - flow.second.rxPackets) / flow.second.txPackets;

        outFile << flow.first << "," << t.sourceAddress << "," << t.destinationAddress << ","
                << delay << "," << jitter << "," << throughput << "," << lossRate << "\n";
    }

    outFile.close();

    Simulator::Destroy();
    return 0;
}

