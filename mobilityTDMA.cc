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
const uint32_t kNumUes = 50;
const double kSlotDuration = 0.1; // seconds
const double kSimDuration = 50.0; // seconds
const uint32_t kPacketSize = 1024;
const uint32_t kPacketsPerSlot = 10; // Reduced from 100 packets/second
const double kGuardTime = 0.001; // 1ms guard time between slots

// Global variables for coordinated scheduling
std::vector<std::vector<Ptr<Application>>> uplinkApps;
std::vector<std::vector<Ptr<Application>>> downlinkApps;

// Custom application for controlled packet transmission
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
    : m_socket(0),
      m_packetSize(0),
      m_nPackets(0),
      m_count(0) {
}

TdmaClientApp::~TdmaClientApp() {
    m_socket = 0;
}

void TdmaClientApp::Setup(Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets) {
    m_socket = socket;
    m_peer = address;
    m_packetSize = packetSize;
    m_nPackets = nPackets;
    m_interval = Seconds((kSlotDuration - kGuardTime) / nPackets);
}

void TdmaClientApp::SetStartStopTime(Time startTime, Time stopTime) {
    m_startTime = startTime;
    m_stopTime = stopTime;
}

void TdmaClientApp::StartApplication(void) {
    m_count = 0;
    m_sendEvent = Simulator::Schedule(Seconds(0.0), &TdmaClientApp::SendPacket, this);
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
    Ptr<Packet> packet = Create<Packet>(m_packetSize);
    m_socket->Send(packet);
    m_count++;
    
    if (m_count < m_nPackets) {
        ScheduleNextTx();
    }
}

void TdmaClientApp::ScheduleNextTx(void) {
    Time nextTime = m_interval;
    m_sendEvent = Simulator::Schedule(nextTime, &TdmaClientApp::SendPacket, this);
}

int main(int argc, char *argv[]) {
    // Enhanced command line parsing
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

    // Enable logging for debugging
    LogComponentEnable("TdmaDuplexSimImproved", LOG_LEVEL_INFO);

    // Create nodes
    NodeContainer bsNode, ueNodes;
    bsNode.Create(1);
    ueNodes.Create(numUes);

    // Configure WiFi with optimized settings for TDMA
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    // Use a more controlled channel model
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    
    // Configure for better TDMA performance
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("DsssRate11Mbps"),
                                 "ControlMode", StringValue("DsssRate1Mbps"));

    WifiMacHelper mac;
    Ssid ssid = Ssid("tdma-improved");

    // Configure MAC with TDMA-friendly settings
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false),
                "QosSupported", BooleanValue(false)); // Disable QoS for simpler scheduling

    NetDeviceContainer ueDevices = wifi.Install(phy, mac, ueNodes);

    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid),
                "QosSupported", BooleanValue(false),
                "EnableBeaconJitter", BooleanValue(false)); // Disable beacon jitter

    NetDeviceContainer bsDevice = wifi.Install(phy, mac, bsNode);

    // Set RTS/CTS if enabled
    if (enableRtsCts) {
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/RtsCtsThreshold",
                   UintegerValue(100));
    }

    // Configure mobility with realistic positioning
    MobilityHelper mobility;
    
    // Place BS at center
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(0.0),
                                  "DeltaY", DoubleValue(0.0),
                                  "GridWidth", UintegerValue(1),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(bsNode);

    // Place UEs in a circle around BS
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                  "X", DoubleValue(0.0),
                                  "Y", DoubleValue(0.0),
                                  "Rho", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=50.0]"));
    mobility.Install(ueNodes);

    // Install Internet stack
    InternetStackHelper internet;
    internet.Install(bsNode);
    internet.Install(ueNodes);

    // Assign IP addresses
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer bsInterface = ipv4.Assign(bsDevice);
    Ipv4InterfaceContainer ueInterfaces = ipv4.Assign(ueDevices);

    uint16_t uplinkPort = 5000;
    uint16_t downlinkPort = 5001;

    // Install UDP servers
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

    // Create TDMA applications with improved scheduling
    uplinkApps.resize(numUes);
    downlinkApps.resize(numUes);

    // Calculate number of TDMA cycles that fit in simulation duration
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
            
            // Create uplink application (UE -> BS)
            TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
            Ptr<Socket> uplinkSocket = Socket::CreateSocket(ueNodes.Get(i), tid);
            InetSocketAddress uplinkDest = InetSocketAddress(bsInterface.GetAddress(0), uplinkPort);
            uplinkSocket->Connect(uplinkDest);
            
            Ptr<TdmaClientApp> uplinkApp = CreateObject<TdmaClientApp>();
            uplinkApp->Setup(uplinkSocket, uplinkDest, packetSize, kPacketsPerSlot);
            uplinkApp->SetStartStopTime(Seconds(uplinkStart), Seconds(uplinkStart + slotDuration - kGuardTime));
            ueNodes.Get(i)->AddApplication(uplinkApp);
            uplinkApps[i].push_back(uplinkApp);
            
            // Create downlink application (BS -> UE)
            Ptr<Socket> downlinkSocket = Socket::CreateSocket(bsNode.Get(0), tid);
            InetSocketAddress downlinkDest = InetSocketAddress(ueInterfaces.GetAddress(i), downlinkPort);
            downlinkSocket->Connect(downlinkDest);
            
            Ptr<TdmaClientApp> downlinkApp = CreateObject<TdmaClientApp>();
            downlinkApp->Setup(downlinkSocket, downlinkDest, packetSize, kPacketsPerSlot);
            downlinkApp->SetStartStopTime(Seconds(downlinkStart), Seconds(downlinkStart + slotDuration - kGuardTime));
            bsNode.Get(0)->AddApplication(downlinkApp);
            downlinkApps[i].push_back(downlinkApp);
        }
    }


    // Install Flow Monitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // Configure NetAnim animation
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

        NS_LOG_INFO("NetAnim animation enabled. Output file: " << animationFile);
    }


    // Enable packet capture for debugging (optional)
    // phy.EnablePcapAll("tdma-improved");

    // Run simulation
    Simulator::Stop(Seconds(simDuration));
    
    NS_LOG_INFO("Starting simulation...");
    Simulator::Run();
    NS_LOG_INFO("Simulation completed.");

    // Analyze results
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    // Create output file with timestamp
    std::string filename = "tdma_improved_results_" + std::to_string(numUes) + "ues.csv";
    std::ofstream outFile(filename);
    outFile << "FlowId,UeId,Direction,SrcAddr,DestAddr,TxPackets,RxPackets,TxBytes,RxBytes,"
            << "Delay(ms),Jitter(ms),Throughput(kbps),LossRate(%),Duration(s)\n";

    double totalThroughput = 0;
    double totalDelay = 0;
    double totalJitter = 0;
    double totalLossRate = 0;
    uint32_t validFlows = 0;

    for (const auto& flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        
        // Determine UE ID and direction
        uint32_t ueId = 0;
        std::string direction = "Unknown";
        
        // Find which UE this flow belongs to
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

        // Calculate metrics
        double delay = flow.second.rxPackets > 0 ?
                       flow.second.delaySum.GetSeconds() * 1000 / flow.second.rxPackets : 0;
        double jitter = flow.second.rxPackets > 0 ?
                        flow.second.jitterSum.GetSeconds() * 1000 / flow.second.rxPackets : 0;
        double duration = (flow.second.timeLastRxPacket - flow.second.timeFirstTxPacket).GetSeconds();
        double throughput = (duration > 0) ? flow.second.rxBytes * 8.0 / duration / 1000 : 0; // kbps
        double lossRate = flow.second.txPackets > 0 ?
                          100.0 * (flow.second.txPackets - flow.second.rxPackets) / flow.second.txPackets : 0;

        outFile << flow.first << "," << ueId << "," << direction << ","
                << t.sourceAddress << "," << t.destinationAddress << ","
                << flow.second.txPackets << "," << flow.second.rxPackets << ","
                << flow.second.txBytes << "," << flow.second.rxBytes << ","
                << delay << "," << jitter << "," << throughput << "," << lossRate << ","
                << duration << "\n";

        // Accumulate statistics
        if (flow.second.rxPackets > 0) {
            totalThroughput += throughput;
            totalDelay += delay;
            totalJitter += jitter;
            totalLossRate += lossRate;
            validFlows++;
        }
    }

    outFile.close();

    // Print summary statistics
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
    
    // Clean up animation interface
    if (anim) {
        delete anim;
    }
    
    return 0;
}
