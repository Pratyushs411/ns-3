#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/netanim-module.h"

#include <fstream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TdmaDuplexSim2BS");

const uint32_t kNumUes        = 200;     // must always match your UE count
const double   kSlotDuration  = 0.1;   // 50 ms slot
const double   kSimDuration   = 60.0;   // total sim time
const uint32_t kPacketsPerSlot = 5;     // reduce burstiness
const uint32_t kPacketSize    = 1024;   // bytes
const double   kGuardTime     = 0.004;  // 1 ms guard



std::vector<std::vector<Ptr<Application>>> uplinkApps;
std::vector<std::vector<Ptr<Application>>> downlinkApps;

class TdmaClientApp : public Application {
public:
  TdmaClientApp() : m_socket(0) {}
  ~TdmaClientApp() override { m_socket = 0; }

  void Setup(Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets) {
    m_socket = socket;
    m_peer = address;
    m_packetSize = packetSize;
    m_nPackets = nPackets;
    m_interval = Seconds((kSlotDuration - kGuardTime) / nPackets);
  }
  void SetStartStopTime(Time startTime, Time stopTime) {
    m_startTime = startTime;
    m_stopTime  = stopTime;
    Application::SetStartTime(startTime);
    Application::SetStopTime(stopTime);
  }

private:
  void StartApplication(void) override {
    if (!m_socket) return;
    if (m_socket->GetBoundNetDevice() == nullptr) {
      m_socket->Bind();
    }
    m_socket->Connect(m_peer);

    m_count = 0;
    if (Simulator::Now() < m_startTime) {
      Simulator::Schedule(m_startTime - Simulator::Now(), &TdmaClientApp::StartApplication, this);
      return;
    }
    if (Simulator::Now() > m_stopTime) return;

    m_sendEvent = Simulator::ScheduleNow(&TdmaClientApp::SendPacket, this);
  }

  void StopApplication(void) override {
    if (m_sendEvent.IsPending()) Simulator::Cancel(m_sendEvent);
    if (m_socket) m_socket->Close();
  }

  void SendPacket(void) {
    if (Simulator::Now() >= m_stopTime) return;
    Ptr<Packet> packet = Create<Packet>(m_packetSize);
    m_socket->Send(packet);
    m_count++;
    if (m_count < m_nPackets && Simulator::Now() + m_interval < m_stopTime) {
      ScheduleNextTx();
    }
  }

  void ScheduleNextTx(void) {
    m_sendEvent = Simulator::Schedule(m_interval, &TdmaClientApp::SendPacket, this);
  }

  Ptr<Socket> m_socket;
  Address     m_peer;
  uint32_t    m_packetSize{0};
  uint32_t    m_nPackets{0};
  uint32_t    m_count{0};
  EventId     m_sendEvent;
  Time        m_interval;
  Time        m_startTime;
  Time        m_stopTime;
};

int main(int argc, char *argv[]) {
  uint32_t    numUes        = kNumUes;
  double      slotDuration  = kSlotDuration;
  double      simDuration   = kSimDuration;
  uint32_t    packetSize    = kPacketSize;
  bool        enableAnimation = true;
  std::string animationFile = "tdma-2bs.xml";

  CommandLine cmd;
  cmd.AddValue("numUes", "Number of UE nodes", numUes);
  cmd.AddValue("slotDuration", "Duration of each TDMA slot (seconds)", slotDuration);
  cmd.AddValue("simDuration", "Total simulation duration (seconds)", simDuration);
  cmd.AddValue("packetSize", "Size of each packet (bytes)", packetSize);
  cmd.AddValue("enableAnimation", "Enable NetAnim animation", enableAnimation);
  cmd.AddValue("animationFile", "NetAnim XML output file", animationFile);
  cmd.Parse(argc, argv);

  LogComponentEnable("TdmaDuplexSim2BS", LOG_LEVEL_INFO);

  // --- Topology ---
  NodeContainer bsNodes, ueNodes;
  bsNodes.Create(2);
  ueNodes.Create(numUes);

  // Channel/PHY
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(150.0));

  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());
  phy.Set("TxPowerStart", DoubleValue(20.0));
  phy.Set("TxPowerEnd",   DoubleValue(20.0));

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211g);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode", StringValue("DsssRate11Mbps"),
                               "ControlMode", StringValue("DsssRate1Mbps"));

  WifiMacHelper mac;
  Ssid ssid = Ssid("tdma-2bs");

  // UEs
  mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid),
              "ActiveProbing", BooleanValue(true), "QosSupported", BooleanValue(false));
  NetDeviceContainer ueDevices = wifi.Install(phy, mac, ueNodes);

  // BSs
  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "QosSupported", BooleanValue(false));
  NetDeviceContainer bsDevices = wifi.Install(phy, mac, bsNodes);

  // Mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                "MinX", DoubleValue(-100.0),
                                "MinY", DoubleValue(0.0),
                                "DeltaX", DoubleValue(200.0),
                                "DeltaY", DoubleValue(0.0),
                                "GridWidth", UintegerValue(2),
                                "LayoutType", StringValue("RowFirst"));
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(bsNodes);

  MobilityHelper ueMobility;
  ueMobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                  "X", StringValue("ns3::UniformRandomVariable[Min=-100.0|Max=100.0]"),
                                  "Y", StringValue("ns3::UniformRandomVariable[Min=-50.0|Max=50.0]"));
  ueMobility.SetMobilityModel("ns3::SteadyStateRandomWaypointMobilityModel",
                              "MinX", DoubleValue(-100.0),
                              "MaxX", DoubleValue(100.0),
                              "MinY", DoubleValue(-50.0),
                              "MaxY", DoubleValue(50.0),
                              "MinSpeed", StringValue("1.0"),
                              "MaxSpeed", StringValue("3.0"),
                              "MinPause", StringValue("0.5"),
                              "MaxPause", StringValue("2.0"));
  ueMobility.Install(ueNodes);

  // Internet
  InternetStackHelper internet;
  internet.Install(bsNodes);
  internet.Install(ueNodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer bsIfs = ipv4.Assign(bsDevices);
  Ipv4InterfaceContainer ueIfs = ipv4.Assign(ueDevices);

  // Applications
  const uint16_t uplinkPort   = 5000; // UE -> BS
  const uint16_t downlinkPort = 5001; // BS -> UE

  // Uplink servers on BSs
  UdpServerHelper bsServer(uplinkPort);
  ApplicationContainer bsServerApps = bsServer.Install(bsNodes);
  bsServerApps.Start(Seconds(0.0));
  bsServerApps.Stop(Seconds(simDuration));

  // Downlink servers on each UE
  ApplicationContainer ueServers;
  for (uint32_t i = 0; i < numUes; ++i) {
    UdpServerHelper ueServer(downlinkPort);
    auto app = ueServer.Install(ueNodes.Get(i));
    app.Start(Seconds(0.0));
    app.Stop(Seconds(simDuration));
    ueServers.Add(app);
  }

  uplinkApps.resize(numUes);
  downlinkApps.resize(numUes);

  const double   cycleDuration = 2 * slotDuration * numUes;
  const uint32_t numCycles     = static_cast<uint32_t>(simDuration / cycleDuration);

  for (uint32_t cycle = 0; cycle < numCycles; ++cycle) {
    const double cycleStartTime = cycle * cycleDuration;
    for (uint32_t i = 0; i < numUes; ++i) {
      const uint32_t bsIndex       = (i < numUes / 2 ? 0 : 1);
      const double   uplinkStart   = cycleStartTime + i * 2 * slotDuration;
      const double   downlinkStart = uplinkStart + slotDuration;

      const double   uplinkStop    = uplinkStart   + slotDuration - kGuardTime;
      const double   downlinkStop  = downlinkStart + slotDuration - kGuardTime;

      if (uplinkStart   >= simDuration || uplinkStop   <= uplinkStart)   continue;
      if (downlinkStart >= simDuration || downlinkStop <= downlinkStart) continue;

      TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

      // Uplink UE -> BS
      Ptr<Socket> uplinkSocket = Socket::CreateSocket(ueNodes.Get(i), tid);
      InetSocketAddress uplinkDst = InetSocketAddress(bsIfs.GetAddress(bsIndex), uplinkPort);
      Ptr<TdmaClientApp> uplinkApp = CreateObject<TdmaClientApp>();
      uplinkApp->Setup(uplinkSocket, uplinkDst, packetSize, kPacketsPerSlot);
      uplinkApp->SetStartStopTime(Seconds(uplinkStart), Seconds(uplinkStop));
      ueNodes.Get(i)->AddApplication(uplinkApp);
      uplinkApps[i].push_back(uplinkApp);

      // Downlink BS -> UE
      Ptr<Socket> downlinkSocket = Socket::CreateSocket(bsNodes.Get(bsIndex), tid);
      InetSocketAddress downlinkDst = InetSocketAddress(ueIfs.GetAddress(i), downlinkPort);
      Ptr<TdmaClientApp> downlinkApp = CreateObject<TdmaClientApp>();
      downlinkApp->Setup(downlinkSocket, downlinkDst, packetSize, kPacketsPerSlot);
      downlinkApp->SetStartStopTime(Seconds(downlinkStart), Seconds(downlinkStop));
      bsNodes.Get(bsIndex)->AddApplication(downlinkApp);
      downlinkApps[i].push_back(downlinkApp);
    }
  }

  // FlowMonitor
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  // NetAnim
  AnimationInterface* anim = nullptr;
  if (enableAnimation) {
    anim = new AnimationInterface(animationFile);

    // Either raise the packet cap...
    anim->SetMaxPktsPerTraceFile(100000000ULL); // 1e8 packets per file

    // ...or disable packet-level tracing entirely to avoid the message:
    // anim->SkipPacketTracing();

    anim->UpdateNodeDescription(bsNodes.Get(0), "Base Station 0");
    anim->UpdateNodeColor(bsNodes.Get(0), 255, 0, 0);
    anim->UpdateNodeSize(bsNodes.Get(0), 6.0, 6.0);

    anim->UpdateNodeDescription(bsNodes.Get(1), "Base Station 1");
    anim->UpdateNodeColor(bsNodes.Get(1), 0, 0, 255);
    anim->UpdateNodeSize(bsNodes.Get(1), 6.0, 6.0);

    for (uint32_t i = 0; i < numUes; ++i) {
      anim->UpdateNodeDescription(ueNodes.Get(i), "UE-" + std::to_string(i));
      anim->UpdateNodeColor(ueNodes.Get(i), 0, 255, 0);
      anim->UpdateNodeSize(ueNodes.Get(i), 3.0, 3.0);
    }
  }

  Simulator::Stop(Seconds(simDuration));
  Simulator::Run();

  // ---- Outputs ----
  monitor->CheckForLostPackets();

  // 1) XML
  monitor->SerializeToXmlFile("tdma-flowmon.xml", true, true);

  // 2) CSV (native: iterate FlowMonitor stats)
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
  auto stats = monitor->GetFlowStats();

  std::ofstream allCsv("tdma-flows-all.csv");
  std::ofstream ulCsv ("tdma-uplink.csv");
  std::ofstream dlCsv ("tdma-downlink.csv");

  auto writeHeader = [](std::ostream& os) {
    os << "flowId,srcAddr,srcPort,dstAddr,dstPort,direction,txPackets,rxPackets,lostPackets,"
          "txBytes,rxBytes,duration_s,throughput_Mbps,mean_delay_ms,mean_jitter_ms,loss_rate\n";
  };
  writeHeader(allCsv); writeHeader(ulCsv); writeHeader(dlCsv);


  for (const auto& kv : stats) {
    FlowId fid = kv.first;
    const FlowMonitor::FlowStats& st = kv.second;
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(fid);

      double t_first_tx = st.timeFirstTxPacket.IsZero() ? 0.0 : st.timeFirstTxPacket.GetSeconds();
      double t_last_tx  = st.timeLastTxPacket.IsZero()  ? 0.0 : st.timeLastTxPacket.GetSeconds();
      double t_last_rx  = st.timeLastRxPacket.IsZero()  ? 0.0 : st.timeLastRxPacket.GetSeconds();

      double duration;
      if (t_last_rx > t_first_tx) {
          duration = t_last_rx - t_first_tx;              // normal case
      } else {
          duration = t_last_tx - t_first_tx;              // fallback if no Rx
          if (duration < 0) duration = 0.0;               // safety guard
      }


    double thrMbps    = (duration > 0.0) ? (st.rxBytes * 8.0 / duration / 1e6) : 0.0;
    double meanDelay  = (st.rxPackets > 0) ? (st.delaySum.GetSeconds()  / st.rxPackets * 1000.0) : 0.0;
    double meanJitter = (st.rxPackets > 1) ? (st.jitterSum.GetSeconds() / (st.rxPackets - 1) * 1000.0) : 0.0;
    double lossRate   = (st.txPackets > 0) ? (double(st.lostPackets) / double(st.txPackets)) : 0.0;

    std::string dir = "other";
    if (t.destinationPort == uplinkPort)     dir = "uplink";
    else if (t.destinationPort == downlinkPort) dir = "downlink";

    auto writeLine = [&](std::ostream& os){
      os << fid << ","
         << t.sourceAddress << "," << t.sourcePort << ","
         << t.destinationAddress << "," << t.destinationPort << ","
         << dir << ","
         << st.txPackets << "," << st.rxPackets << "," << st.lostPackets << ","
         << st.txBytes << "," << st.rxBytes << ","
         << std::fixed << std::setprecision(6) << duration << ","
         << std::setprecision(6) << thrMbps << ","
         << std::setprecision(6) << meanDelay << ","
         << std::setprecision(6) << meanJitter << ","
         << std::setprecision(6) << lossRate << "\n";
    };

    writeLine(allCsv);
    if (dir == "uplink")   writeLine(ulCsv);
    if (dir == "downlink") writeLine(dlCsv);
  }

  allCsv.close(); ulCsv.close(); dlCsv.close();

  Simulator::Destroy();
  if (anim) delete anim;
  return 0;
}

