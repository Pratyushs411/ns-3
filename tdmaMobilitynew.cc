#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TdmaDuplexSimImproved");

// Configuration parameters
const uint32_t kNumUes = 10;
const double kSlotDuration = 0.1; // seconds
const double kSimDuration = 20.0; // seconds
const uint32_t kPacketSize = 1024;
const uint32_t kPacketsPerSlot = 10;
const double kGuardTime = 0.001; // 1ms guard time

// Global variables
std::vector<std::vector<Ptr<Application>>> uplinkApps;
std::vector<std::vector<Ptr<Application>>> downlinkApps;

class TdmaClientApp : public Application {
public:
    TdmaClientApp();
    virtual ~TdmaClientApp();

    void Setup(Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets);
    void SetStartStopTime(Time startTime, Time stopTime);

private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void SendPacket(void);
    void ScheduleNextTx(void);

    Ptr<Socket> m_socket;
    Address m_peer;
    uint32_t m_packetSize;
    uint32_t m_nPackets;
    uint32_t m_count;
    EventId m_sendEvent;
    Time m_interval;
    Time m_startTime;
    Time m_stopTime;
};

TdmaClientApp::TdmaClientApp()
    : m_socket(0), m_packetSize(0), m_nPackets(0), m_count(0) {}

TdmaClientApp::~TdmaClientApp() { m_socket = 0; }

void TdmaClientApp::Setup(Ptr<Socket> socket, Address address,
                          uint32_t packetSize, uint32_t nPackets) {
    m_socket = socket;
    m_peer = address;
    m_packetSize = packetSize;
    m_nPackets = nPackets;
    m_interval = Seconds((kSlotDuration - kGuardTime) / nPackets);
}

void TdmaClientApp::SetStartStopTime(Time startTime, Time stopTime) {
    m_startTime = startTime;
    m_stopTime = stopTime;
    Application::SetStartTime(startTime);
    Application::SetStopTime(stopTime);
}

void TdmaClientApp::StartApplication(void) {
    if (!m_socket) return;

    // Bind only if not already bound (avoid conflicts)
        if (m_socket->GetBoundNetDevice() == nullptr) {

        m_socket->Bind();
    }
    m_socket->Connect(m_peer);

    m_count = 0;
    if (Simulator::Now() < m_startTime) {
        Simulator::Schedule(m_startTime - Simulator::Now(),
                            &TdmaClientApp::StartApplication, this);
        return;
    }
    if (Simulator::Now() > m_stopTime) {
        return;
    }
    m_sendEvent = Simulator::ScheduleNow(&TdmaClientApp::SendPacket, this);
}

void TdmaClientApp::StopApplication(void) {
    if (m_sendEvent.IsPending()) {
        Simulator::Cancel(m_sendEvent);
    }
    if (m_socket) {
        m_socket->Close();
    }
}

void TdmaClientApp::SendPacket(void) {
    NS_LOG_INFO("Sending packet at " << Simulator::Now().GetSeconds());
    if (Simulator::Now() >= m_stopTime) {
        return;
    }

    Ptr<Packet> packet = Create<Packet>(m_packetSize);
    m_socket->Send(packet);
    m_count++;

    if (m_count < m_nPackets && Simulator::Now() + m_interval < m_stopTime) {
        ScheduleNextTx();
    }
}

void TdmaClientApp::ScheduleNextTx(void) {
    m_sendEvent = Simulator::Schedule(m_interval, &TdmaClientApp::SendPacket, this);
}

int main(int argc, char *argv[]) {
    uint32_t numUes = kNumUes;
    double slotDuration = kSlotDuration;
    double simDuration = kSimDuration;
    uint32_t packetSize = kPacketSize;
    bool enableRtsCts = false;
    bool enableAnimation = true;
    std::string animationFile = "tdma-animation.xml";

    CommandLine cmd;
    cmd.AddValue("numUes", "Number of UE nodes", numUes);
    cmd.AddValue("slotDuration", "Duration of each TDMA slot (seconds)", slotDuration);
    cmd.AddValue("simDuration", "Total simulation duration (seconds)", simDuration);
    cmd.AddValue("packetSize", "Size of each packet (bytes)", packetSize);
    cmd.AddValue("enableRtsCts", "Enable RTS/CTS for WiFi", enableRtsCts);
    cmd.AddValue("enableAnimation", "Enable NetAnim animation", enableAnimation);
    cmd.AddValue("animationFile", "NetAnim XML output file", animationFile);
    cmd.Parse(argc, argv);

    LogComponentEnable("TdmaDuplexSimImproved", LOG_LEVEL_INFO);

    NodeContainer bsNode, ueNodes;
    bsNode.Create(1);
    ueNodes.Create(numUes);

    // WiFi channel with guaranteed coverage
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    // FIX: ensure coverage for mobility
    channel.AddPropagationLoss("ns3::RangePropagationLossModel",
                               "MaxRange", DoubleValue(120.0)); // 120m range

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(20.0));
    phy.Set("TxPowerEnd", DoubleValue(20.0));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("DsssRate11Mbps"),
                                 "ControlMode", StringValue("DsssRate1Mbps"));

    WifiMacHelper mac;
    Ssid ssid = Ssid("tdma-improved");

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(true),  // allow reassociation
                "QosSupported", BooleanValue(false));

    NetDeviceContainer ueDevices = wifi.Install(phy, mac, ueNodes);

    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid),
                "QosSupported", BooleanValue(false),
                "EnableBeaconJitter", BooleanValue(false));

    NetDeviceContainer bsDevice = wifi.Install(phy, mac, bsNode);

    if (enableRtsCts) {
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/RtsCtsThreshold",
                    UintegerValue(100));
    }

    // Mobility
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(0.0),
                                  "DeltaY", DoubleValue(0.0),
                                  "GridWidth", UintegerValue(1),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(bsNode);

    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                    "X", StringValue("ns3::UniformRandomVariable[Min=-50.0|Max=50.0]"),
                                    "Y", StringValue("ns3::UniformRandomVariable[Min=-50.0|Max=50.0]"));

    ueMobility.SetMobilityModel("ns3::SteadyStateRandomWaypointMobilityModel",
                                "MinX", DoubleValue(-50.0),
                                "MaxX", DoubleValue(50.0),
                                "MinY", DoubleValue(-50.0),
                                "MaxY", DoubleValue(50.0),
                                "MinSpeed", StringValue("1.0"),
                                "MaxSpeed", StringValue("3.0"),
                                "MinPause", StringValue("0.5"),
                                "MaxPause", StringValue("2.0"));
    ueMobility.Install(ueNodes);

    // Internet stack
    InternetStackHelper internet;
    internet.Install(bsNode);
    internet.Install(ueNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer bsInterface = ipv4.Assign(bsDevice);
    Ipv4InterfaceContainer ueInterfaces = ipv4.Assign(ueDevices);

    uint16_t uplinkPort = 5000;
    uint16_t downlinkPort = 5001;

    UdpServerHelper server(uplinkPort);
    ApplicationContainer serverApp = server.Install(bsNode.Get(0));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(simDuration));

    ApplicationContainer ueServers;
    for (uint32_t i = 0; i < numUes; ++i) {
        UdpServerHelper ueServer(downlinkPort);
        ApplicationContainer app = ueServer.Install(ueNodes.Get(i));
        app.Start(Seconds(0.0));
        app.Stop(Seconds(simDuration));
        ueServers.Add(app);
    }

    // TDMA scheduling
    uplinkApps.resize(numUes);
    downlinkApps.resize(numUes);

    double cycleDuration = 2 * slotDuration * numUes;
    uint32_t numCycles = static_cast<uint32_t>(simDuration / cycleDuration);

    NS_LOG_INFO("TDMA Configuration:");
    NS_LOG_INFO("  Number of UEs: " << numUes);
    NS_LOG_INFO("  Slot Duration: " << slotDuration << "s");
    NS_LOG_INFO("  Cycle Duration: " << cycleDuration << "s");
    NS_LOG_INFO("  Number of Cycles: " << numCycles);

    for (uint32_t cycle = 0; cycle < numCycles; ++cycle) {
        double cycleStartTime = cycle * cycleDuration;

        for (uint32_t i = 0; i < numUes; ++i) {
            double uplinkStart = cycleStartTime + i * 2 * slotDuration;
            double downlinkStart = uplinkStart + slotDuration;

            TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

            // uplink
            Ptr<Socket> uplinkSocket = Socket::CreateSocket(ueNodes.Get(i), tid);
            InetSocketAddress uplinkDest = InetSocketAddress(bsInterface.GetAddress(0), uplinkPort);

            Ptr<TdmaClientApp> uplinkApp = CreateObject<TdmaClientApp>();
            uplinkApp->Setup(uplinkSocket, uplinkDest, packetSize, kPacketsPerSlot);
            uplinkApp->SetStartStopTime(Seconds(uplinkStart),
                                        Seconds(uplinkStart + slotDuration - kGuardTime));
            ueNodes.Get(i)->AddApplication(uplinkApp);
            uplinkApps[i].push_back(uplinkApp);

            // downlink
            Ptr<Socket> downlinkSocket = Socket::CreateSocket(bsNode.Get(0), tid);
            InetSocketAddress downlinkDest = InetSocketAddress(ueInterfaces.GetAddress(i), downlinkPort);

            Ptr<TdmaClientApp> downlinkApp = CreateObject<TdmaClientApp>();
            downlinkApp->Setup(downlinkSocket, downlinkDest, packetSize, kPacketsPerSlot);
            downlinkApp->SetStartStopTime(Seconds(downlinkStart),
                                          Seconds(downlinkStart + slotDuration - kGuardTime));
            bsNode.Get(0)->AddApplication(downlinkApp);
            downlinkApps[i].push_back(downlinkApp);
        }
    }

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    AnimationInterface* anim = nullptr;
    if (enableAnimation) {
        anim = new AnimationInterface(animationFile);
        Ptr<Node> bs = bsNode.Get(0);
        anim->UpdateNodeDescription(bs, "Base Station");
        anim->UpdateNodeColor(bs, 255, 0, 0);
        anim->UpdateNodeSize(bs, 5.0, 5.0);

        for (uint32_t i = 0; i < numUes; ++i) {
            Ptr<Node> ue = ueNodes.Get(i);
            anim->UpdateNodeDescription(ue, "UE-" + std::to_string(i));
            anim->UpdateNodeColor(ue, 0, 255, 0);
            anim->UpdateNodeSize(ue, 3.0, 3.0);
        }

        anim->EnablePacketMetadata(true);
        anim->EnableIpv4RouteTracking("tdma-packets", Seconds(0), Seconds(simDuration));
    }

    Simulator::Stop(Seconds(simDuration));
    NS_LOG_INFO("Starting simulation...");
    Simulator::Run();
    NS_LOG_INFO("Simulation completed.");

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    std::string filename = "tdma_improved_results_" + std::to_string(numUes) + "ues.csv";
    std::ofstream outFile(filename);
    outFile << "FlowId,UeId,Direction,SrcAddr,DestAddr,TxPackets,RxPackets,TxBytes,RxBytes,"
            << "Delay(ms),Jitter(ms),Throughput(kbps),LossRate(%),Duration(s)\n";

    double totalThroughput = 0, totalDelay = 0, totalJitter = 0, totalLossRate = 0;
    uint32_t validFlows = 0;

    for (const auto& flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        uint32_t ueId = 0;
        std::string direction = "Unknown";

        for (uint32_t i = 0; i < numUes; ++i) {
            if (t.sourceAddress == ueInterfaces.GetAddress(i)) {
                ueId = i;
                direction = "Uplink";
                break;
            } else if (t.destinationAddress == ueInterfaces.GetAddress(i)) {
                ueId = i;
                direction = "Downlink";
                break;
            }
        }

        double delay = flow.second.rxPackets > 0 ?
                       flow.second.delaySum.GetSeconds() * 1000 / flow.second.rxPackets : 0;
        double jitter = flow.second.rxPackets > 0 ?
                        flow.second.jitterSum.GetSeconds() * 1000 / flow.second.rxPackets : 0;
        double duration = (flow.second.timeLastRxPacket - flow.second.timeFirstTxPacket).GetSeconds();
        double throughput = (duration > 0) ?
                            flow.second.rxBytes * 8.0 / duration / 1000 : 0;
        double lossRate = flow.second.txPackets > 0 ?
                          100.0 * (flow.second.txPackets - flow.second.rxPackets) / flow.second.txPackets : 0;

        outFile << flow.first << "," << ueId << "," << direction << ","
                << t.sourceAddress << "," << t.destinationAddress << ","
                << flow.second.txPackets << "," << flow.second.rxPackets << ","
                << flow.second.txBytes << "," << flow.second.rxBytes << ","
                << delay << "," << jitter << "," << throughput << "," << lossRate << ","
                << duration << "\n";

        if (flow.second.rxPackets > 0) {
            totalThroughput += throughput;
            totalDelay += delay;
            totalJitter += jitter;
            totalLossRate += lossRate;
            validFlows++;
        }
    }

    outFile.close();

    if (validFlows > 0) {
        NS_LOG_INFO("=== SIMULATION RESULTS ===");
        NS_LOG_INFO("Total Flows: " << stats.size());
        NS_LOG_INFO("Valid Flows: " << validFlows);
        NS_LOG_INFO("Average Delay: " << totalDelay / validFlows << " ms");
        NS_LOG_INFO("Average Jitter: " << totalJitter / validFlows << " ms");
        NS_LOG_INFO("Total Throughput: " << totalThroughput << " kbps");
        NS_LOG_INFO("Average Loss Rate: " << totalLossRate / validFlows << " %");
        NS_LOG_INFO("Results saved to: " << filename);
    }

    Simulator::Destroy();
    if (anim) delete anim;
    return 0;
}

