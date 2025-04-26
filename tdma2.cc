#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/config-store-module.h"
#include "ns3/nr-module.h"
#include<vector>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("5GNR_TDMA_Simulation");

// Function to write metrics to CSV
void WriteMetricsToCSV(FlowMonitorHelper& flowMonHelper, Ptr<FlowMonitor> flowMon, std::ofstream& csvFile, Time interval, Time simTime) {
  static uint32_t sampleCount = 0;
  
  // Collect metrics at the specified interval
  Simulator::Schedule(interval, &WriteMetricsToCSV, std::ref(flowMonHelper), flowMon, std::ref(csvFile), interval, simTime);
  
  // Check if we've reached the end of simulation
  if (Simulator::Now() >= simTime) {
    return;
  }
  
  // Get metrics from the flow monitor
  flowMon->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowMonHelper.GetClassifier());
  std::map<FlowId, FlowMonitor::FlowStats> stats = flowMon->GetFlowStats();
  
  for (auto it = stats.begin(); it != stats.end(); ++it) {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(it->first);
    
    // Calculate QoS metrics
    double throughput = it->second.rxBytes * 8.0 / (interval.GetSeconds()) / 1000.0; // kbps
    double jitter = it->second.jitterSum.GetSeconds() / it->second.rxPackets;
    double delay = it->second.delaySum.GetSeconds() / it->second.rxPackets;
    double packetLoss = 100.0 * (it->second.txPackets - it->second.rxPackets) / it->second.txPackets;
    
    // Write to CSV
    csvFile << sampleCount << ","
            << Simulator::Now().GetSeconds() << ","
            << it->first << ","
            << t.sourceAddress << ","
            << t.destinationAddress << ","
            << throughput << ","
            << jitter << ","
            << delay << ","
            << packetLoss << std::endl;
  }
  
  sampleCount++;
}

int main(int argc, char *argv[]) {
  // Simulation parameters
  uint16_t numUes = 10;
  double simTime = 10.0; // seconds
  double interval = 0.5; // interval for collecting metrics in seconds
  
  // Enable command-line arguments
  CommandLine cmd;
  cmd.AddValue("numUes", "Number of UE devices", numUes);
  cmd.AddValue("simTime", "Total simulation time", simTime);
  cmd.AddValue("interval", "Interval for collecting metrics", interval);
  cmd.Parse(argc, argv);
  
  // Set simulation time resolution
  Time::SetResolution(Time::NS);
  
  // Enable NR logs
  LogComponentEnable("NrGnbPhy", LOG_LEVEL_INFO);
  LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
  LogComponentEnable("NrHelper", LOG_LEVEL_INFO);
  
  // Create NR helpers
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
  nrHelper->SetEpcHelper(epcHelper);
  
  // Set scheduler to TDMA
  nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
  
  // Configure TDD pattern before creating devices
  Config::SetDefault("ns3::NrGnbPhy::Pattern", StringValue("DL|DL|DL|DL|DL|DL|F|UL|UL|UL|"));
  
  // Configure transmission power
  Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(40.0)); // 40 dBm for sub-6 GHz
  Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23.0)); // 23 dBm
  
  // Create gNB and UE nodes
  NodeContainer gnbNodes;
  NodeContainer ueNodes;
  gnbNodes.Create(1);
  ueNodes.Create(numUes);
  
  // Configure mobility model for gNB and UEs
  MobilityHelper mobility;
  
  // Set gNB to fixed position
  Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
  gnbPositionAlloc->Add(Vector(0.0, 0.0, 30.0)); // gNB height of 30m for standard 5G
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(gnbPositionAlloc);
  mobility.Install(gnbNodes);
  
  // Set UEs to random positions within a circle around gNB
  mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                               "X", DoubleValue(0.0),
                               "Y", DoubleValue(0.0),
                               "Rho", StringValue("ns3::UniformRandomVariable[Min=50.0|Max=500.0]")); // Increased coverage range
  mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                           "Bounds", RectangleValue(Rectangle(-550, 550, -550, 550)),
                           "Speed", StringValue("ns3::ConstantRandomVariable[Constant=3.0]"));
  mobility.Install(ueNodes);
  
  // Enable shadowing
  Config::SetDefault("ns3::ThreeGppPropagationLossModel::ShadowingEnabled", BooleanValue(true));
  
  // Create bandwidth part helpers
  // Changed to sub-6 GHz frequency (3.5 GHz) for standard 5G
  double centralFrequency = 3.5e9; // 3.5 GHz
  double bandwidth = 40e6; // 40 MHz (typical sub-6 GHz bandwidth)

  // Create the configuration for the CcBwpHelper
  CcBwpCreator ccBwpCreator;
  
  const uint8_t numCcPerBand = 1;  // Number of component carriers per band
  CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numCcPerBand);
  
  // By using the configuration created, it is time to make the operation band
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
  
  // New approach that works with your version of ns-3.42
    // Create a vector to hold all bandwidth parts
    std::vector<std::reference_wrapper<BandwidthPartInfoPtr>> allBwps;

    // Make sure we have a valid BWP pointer before continuing
    NS_ASSERT_MSG(band.GetBwpAt(0, 0) != nullptr, "Bandwidth part pointer is null");

    // Get the BWP pointers from the band and add them to the vector
    BandwidthPartInfoPtr& dlBwp = band.GetBwpAt(0, 0);
    allBwps.push_back(std::ref(dlBwp));

    // Print debug info
    NS_LOG_INFO("Added BWP to vector: " << (dlBwp != nullptr));

    // Install NR devices using the vector of bandwidth part references
    NetDeviceContainer gnbNetDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    
  // Install Internet stack on UE nodes
  InternetStackHelper internet;
  internet.Install(ueNodes);
  
  // Assign IP addresses to UEs
  Ipv4InterfaceContainer ueIpIfaces;
  ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDevs));
  
  // Set default routing for UEs
  for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
    Ptr<Node> ueNode = ueNodes.Get(u);
    Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting>(ueNode->GetObject<Ipv4>()->GetRoutingProtocol());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
  }
  
  // Attach UEs to gNB
  nrHelper->AttachToClosestGnb(ueNetDevs, gnbNetDevs);
  
  // Setup dedicated bearers with QoS parameters
  Ptr<NrEpcTft> tft = Create<NrEpcTft>();
  NrEpcTft::PacketFilter dlpf;
  dlpf.localPortStart = 1234;
  dlpf.localPortEnd = 1234;
  tft->Add(dlpf);
  
  // Using EpsBearer for QoS
  NrEpsBearer bearer(NrEpsBearer::GBR_CONV_VIDEO);
  
  // Activate the bearer for all UEs
  for (uint32_t i = 0; i < ueNetDevs.GetN(); ++i) {
    nrHelper->ActivateDedicatedEpsBearer(ueNetDevs.Get(i), bearer, tft);
  }
  
  // Install applications
  uint16_t serverPort = 5000;
  ApplicationContainer serverApps;
  
  // Define application parameters
  uint32_t packetSize = 1400;
  uint32_t maxPackets = 0xFFFFFFFF;
  Time interPacketInterval = Seconds(0.001);
  
  // Create applications for each UE
  for (uint16_t i = 0; i < ueNodes.GetN(); i++) {
    // Set up server on the remote host
    UdpServerHelper server(serverPort + i);
    serverApps.Add(server.Install(epcHelper->GetPgwNode()));
    
    // Set up client on UE
    UdpClientHelper client(epcHelper->GetUeDefaultGatewayAddress(), serverPort + i);
    client.SetAttribute("MaxPackets", UintegerValue(maxPackets));
    client.SetAttribute("Interval", TimeValue(interPacketInterval));
    client.SetAttribute("PacketSize", UintegerValue(packetSize));
    
    ApplicationContainer clientApps = client.Install(ueNodes.Get(i));
    clientApps.Start(Seconds(1.0));
    clientApps.Stop(Seconds(simTime - 0.5));
  }
  
  serverApps.Start(Seconds(0.5));
  serverApps.Stop(Seconds(simTime));
  
  // Install and configure Flow Monitor
  FlowMonitorHelper flowMonHelper;
  Ptr<FlowMonitor> flowMonitor = flowMonHelper.InstallAll();
  flowMonitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
  flowMonitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
  
  // Create CSV file for metrics
  std::ofstream csvFile;
  csvFile.open("5g_qos_metrics.csv");
  csvFile << "Sample,Time,FlowID,SourceIP,DestinationIP,Throughput(kbps),Jitter(s),Delay(s),PacketLoss(%)" << std::endl;
  
  // Schedule periodic metric collection
  Time metricInterval = Seconds(interval);
  Simulator::Schedule(metricInterval, &WriteMetricsToCSV, std::ref(flowMonHelper), flowMonitor, std::ref(csvFile), metricInterval, Seconds(simTime));
  
  // Run simulation
  NS_LOG_INFO("Starting simulation...");
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  
  // Close CSV file
  csvFile.close();
  
  // Clean up
  Simulator::Destroy();
  
  NS_LOG_INFO("Simulation completed successfully!");
  
  return 0;
}
