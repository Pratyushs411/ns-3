#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TdmaDuplex2Bs");

const uint32_t kNumUes = 250;          // total UEs
const double kSlotDuration = 0.1;     // seconds
const double kSimDuration = 60.0;     // seconds
const uint32_t kPacketSize = 1024;
const std::string kDataRate = "2Mbps";

int main(int argc, char *argv[]) {
    CommandLine cmd;
    cmd.Parse(argc, argv);

    NodeContainer bsNodes, ueNodes;
    bsNodes.Create(2);       // 2 BS
    ueNodes.Create(kNumUes); // total UEs

    // WiFi Channel + PHY
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    WifiMacHelper mac;

    // Split UEs into 2 groups
    uint32_t half = kNumUes / 2;

    // AP for BS1
    Ssid ssid1 = Ssid("tdma-bs1");
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid1));
    NetDeviceContainer bsDev1 = wifi.Install(phy, mac, bsNodes.Get(0));

    // AP for BS2
    Ssid ssid2 = Ssid("tdma-bs2");
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid2));
    NetDeviceContainer bsDev2 = wifi.Install(phy, mac, bsNodes.Get(1));

    // UEs for BS1
    NodeContainer ueGroup1;
    for (uint32_t i = 0; i < half; i++) {
        ueGroup1.Add(ueNodes.Get(i));
    }
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid1),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer ueDev1 = wifi.Install(phy, mac, ueGroup1);

    // UEs for BS2
    NodeContainer ueGroup2;
    for (uint32_t i = half; i < kNumUes; i++) {
        ueGroup2.Add(ueNodes.Get(i));
    }
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid2),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer ueDev2 = wifi.Install(phy, mac, ueGroup2);

    // Mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(bsNodes);
    mobility.Install(ueNodes);

    // Internet stack
    InternetStackHelper internet;
    internet.Install(bsNodes);
    internet.Install(ueNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifBs1 = ipv4.Assign(bsDev1);
    Ipv4InterfaceContainer ifUe1 = ipv4.Assign(ueDev1);

    ipv4.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer ifBs2 = ipv4.Assign(bsDev2);
    Ipv4InterfaceContainer ifUe2 = ipv4.Assign(ueDev2);

    uint16_t uplinkPort = 5000;
    uint16_t downlinkPort = 5001;

    // Install servers on BS
    UdpServerHelper server1(uplinkPort);
    ApplicationContainer serverApp1 = server1.Install(bsNodes.Get(0));
    serverApp1.Start(Seconds(0.0));
    serverApp1.Stop(Seconds(kSimDuration));

    UdpServerHelper server2(uplinkPort);
    ApplicationContainer serverApp2 = server2.Install(bsNodes.Get(1));
    serverApp2.Start(Seconds(0.0));
    serverApp2.Stop(Seconds(kSimDuration));

    // Install servers on UEs (downlink receivers)
    ApplicationContainer ueServers;
    for (uint32_t i = 0; i < kNumUes; i++) {
        UdpServerHelper ueServer(downlinkPort);
        auto app = ueServer.Install(ueNodes.Get(i));
        app.Start(Seconds(0.0));
        app.Stop(Seconds(kSimDuration));
        ueServers.Add(app);
    }

    // TDMA clients
    ApplicationContainer allClients;

    // UEs for BS1
    for (uint32_t i = 0; i < half; i++) {
        double t = i * 2 * kSlotDuration;
        while (t < kSimDuration) {
            // Uplink: UE -> BS1
            UdpClientHelper uplink(ifBs1.GetAddress(0), uplinkPort);
            uplink.SetAttribute("PacketSize", UintegerValue(kPacketSize));
            uplink.SetAttribute("Interval", TimeValue(Seconds(0.01)));
            uplink.SetAttribute("MaxPackets", UintegerValue(100000));

            auto uplinkApp = uplink.Install(ueNodes.Get(i));
            uplinkApp.Start(Seconds(t));
            uplinkApp.Stop(Seconds(t + kSlotDuration));
            allClients.Add(uplinkApp);

            // Downlink: BS1 -> UE
            UdpClientHelper downlink(ifUe1.GetAddress(i), downlinkPort);
            downlink.SetAttribute("PacketSize", UintegerValue(kPacketSize));
            downlink.SetAttribute("Interval", TimeValue(Seconds(0.01)));
            downlink.SetAttribute("MaxPackets", UintegerValue(100000));

            auto downlinkApp = downlink.Install(bsNodes.Get(0));
            downlinkApp.Start(Seconds(t + kSlotDuration));
            downlinkApp.Stop(Seconds(t + 2 * kSlotDuration));
            allClients.Add(downlinkApp);

            t += 2 * kSlotDuration * half; // cycle over group
        }
    }

    // UEs for BS2
    for (uint32_t i = 0; i < half; i++) {
        double t = i * 2 * kSlotDuration;
        while (t < kSimDuration) {
            // Uplink: UE -> BS2
            UdpClientHelper uplink(ifBs2.GetAddress(0), uplinkPort);
            uplink.SetAttribute("PacketSize", UintegerValue(kPacketSize));
            uplink.SetAttribute("Interval", TimeValue(Seconds(0.01)));
            uplink.SetAttribute("MaxPackets", UintegerValue(100000));

            auto uplinkApp = uplink.Install(ueNodes.Get(half + i));
            uplinkApp.Start(Seconds(t));
            uplinkApp.Stop(Seconds(t + kSlotDuration));
            allClients.Add(uplinkApp);

            // Downlink: BS2 -> UE
            UdpClientHelper downlink(ifUe2.GetAddress(i), downlinkPort);
            downlink.SetAttribute("PacketSize", UintegerValue(kPacketSize));
            downlink.SetAttribute("Interval", TimeValue(Seconds(0.01)));
            downlink.SetAttribute("MaxPackets", UintegerValue(100000));

            auto downlinkApp = downlink.Install(bsNodes.Get(1));
            downlinkApp.Start(Seconds(t + kSlotDuration));
            downlinkApp.Stop(Seconds(t + 2 * kSlotDuration));
            allClients.Add(downlinkApp);

            t += 2 * kSlotDuration * half;
        }
    }

    // Flow monitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(kSimDuration));
    Simulator::Run();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    auto stats = monitor->GetFlowStats();

    std::ofstream outFile("tdma_2bs_results.csv");
    outFile << "FlowId,BS,Src,Dest,Delay(s),Jitter(s),Throughput(bps),LossRate(%)\n";

    for (auto &flow : stats) {
        auto t = classifier->FindFlow(flow.first);

        std::string bsId = "Unknown";
        if (t.sourceAddress == ifBs1.GetAddress(0) || t.destinationAddress == ifBs1.GetAddress(0)) {
            bsId = "BS1";
        } else if (t.sourceAddress == ifBs2.GetAddress(0) || t.destinationAddress == ifBs2.GetAddress(0)) {
            bsId = "BS2";
        }

        double delay = flow.second.rxPackets > 0 ? flow.second.delaySum.GetSeconds() / flow.second.rxPackets : 0;
        double jitter = flow.second.rxPackets > 0 ? flow.second.jitterSum.GetSeconds() / flow.second.rxPackets : 0;
        double duration = (flow.second.rxPackets > 0)
                            ? (flow.second.timeLastRxPacket - flow.second.timeFirstRxPacket).GetSeconds()
                            : 0;
        double throughput = (duration > 0) ? flow.second.rxBytes * 8.0 / duration : 0;
        double lossRate = flow.second.txPackets > 0
                            ? 100.0 * (flow.second.txPackets - flow.second.rxPackets) / flow.second.txPackets
                            : 0;

        outFile << flow.first << "," << bsId << ","
                << t.sourceAddress << "," << t.destinationAddress << ","
                << delay << "," << jitter << "," << throughput << "," << lossRate << "\n";
    }

    outFile.close();
    Simulator::Destroy();
    return 0;
}

