#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ideal-beamforming-algorithm.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/three-gpp-propagation-loss-model.h"

using namespace ns3; // imports ns-3 namespace

NS_LOG_COMPONENT_DEFINE("3gppChannelFdmComponentCarriersBandwidthPartsExample"); //defines a logging component in NS-3 for debugging and tracking simulation events.
int main(int argc, char* argv[])
{
    uint16_t gNbNum = 1; // no of base stations
    uint16_t ueNumPergNb = 10;
    uint16_t numFlowsUe = 3;
    uint8_t numBands = 1;
    double centralFrequencyBand = 28e9;
    double bandwidthBand = 400e6;  // Reduce bandwidth to 400 MHz for realism

    bool contiguousCc = true;// Use contiguous component carriers (simpler setup)
    uint16_t numerology = 0;

    std::string pattern = "UL|DL|UL|DL|UL|DL|UL|DL|UL|DL|";
    double totalTxPower = 8;
    bool cellScan = true;  // Enable scanning for better beamforming
    double beamSearchAngleStep = 5.0;  // Finer beam search for mmWave

    bool udpFullBuffer = false; //Full Buffer Traffic
    uint32_t udpPacketSizeUll = 128;  // Slightly increased for practical scenarios
    uint32_t udpPacketSizeBe = 1400;  // Closer to Ethernet MTU
    uint32_t lambdaUll = 5000;  // Reduce to prevent excessive queuing
    uint32_t lambdaBe = 1500;   // Slightly increased BE traffic

    bool logging = true;

    bool disableDl = false;
    bool disableUl = false;

    std::string simTag = "TDMA_5G";
    std::string outputDir = "./results/";

    double simTime = 5;           // seconds
    double udpAppStartTime = 0.2; // seconds

    CommandLine cmd(__FILE__);

    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("gNbNum", "The number of gNbs in multiple-ue topology", gNbNum);
    cmd.AddValue("ueNumPergNb", "The number of UE per gNb in multiple-ue topology", ueNumPergNb);
    cmd.AddValue("numBands", "Number of operation bands. More than one implies non-contiguous CC",numBands);
    cmd.AddValue("centralFrequencyBand","The system frequency to be used in band 1",centralFrequencyBand);
    cmd.AddValue("bandwidthBand", "The system bandwidth to be used in band 1", bandwidthBand);
    cmd.AddValue("contiguousCc","Simulate with contiguous CC or non-contiguous CC example",contiguousCc);
    cmd.AddValue("numerology", "Numerlogy to be used in contiguous case", numerology);
    cmd.AddValue("tddPattern","LTE TDD pattern to use",pattern);
    cmd.AddValue("totalTxPower","total tx power that will be proportionally assigned to bandwidth parts depending on each BWP bandwidth ",totalTxPower);
    cmd.AddValue("cellScan","Use beam search method to determine beamforming vector,""true to use cell scanning method",cellScan);
    cmd.AddValue("beamSearchAngleStep","Beam search angle step for beam search method",beamSearchAngleStep);
    cmd.AddValue("udpFullBuffer","Whether to set the full buffer traffic; if this parameter is set then the udpInterval neglected.",udpFullBuffer);
    cmd.AddValue("packetSizeUll","packet size in bytes to be used by ultra low latency traffic",udpPacketSizeUll);
    cmd.AddValue("packetSizeBe","packet size in bytes to be used by best effort traffic",udpPacketSizeBe);
    cmd.AddValue("lambdaUll","Number of UDP packets in one second for ultra low latency traffic",lambdaUll);
    cmd.AddValue("lambdaBe","Number of UDP packets in one second for best effort traffic",lambdaBe);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("disableDl", "Disable DL flow", disableDl);
    cmd.AddValue("disableUl", "Disable UL flow", disableUl);
    cmd.AddValue("simTag","tag to be appended to output filenames to distinguish simulation campaigns",simTag);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);

    cmd.Parse(argc, argv);

    NS_ABORT_IF(numBands < 1);
    NS_ABORT_MSG_IF(disableDl == true && disableUl == true, "Enable one of the flows");
        LogComponentEnable("ThreeGppPropagationLossModel", LOG_LEVEL_ALL);
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("NrPdcp", LOG_LEVEL_INFO);

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(50000));
    // create base stations and mobile terminals
   // This part of the code creates nodes (gNBs and UEs), sets up mobility, and initializes position allocators to define the network topology.
    NodeContainer gNbNodes;//Holds all gNB (Base Station) nodes.
    NodeContainer ueNodes;//Holds all UE (User Equipment) nodes.
    MobilityHelper mobility;//Manages node movement and positioning.

    double gNbHeight = 10;
    double ueHeight = 1.5;

    gNbNodes.Create(gNbNum);
    ueNodes.Create(ueNumPergNb * gNbNum);

    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();

    // Positioning
    double interGnbDistance = 100.0; // Increased spacing between gNBs to prevent interference
    double ueRadius = 50.0;          // Maximum distance UEs can be from the gNB
    double angleStep = 360.0 / ueNumPergNb; // Distribute UEs in a circle

    for (uint32_t i = 0; i < gNbNodes.GetN(); ++i)
    {
        double gNbX = i * interGnbDistance; // Spread out gNBs along the x-axis
        double gNbY = 0.0;

        apPositionAlloc->Add(Vector(gNbX, gNbY, gNbHeight)); // Place gNBs

        // Place UEs around the gNB in a circular pattern
        for (uint32_t j = 0; j < ueNumPergNb; ++j)
        {
            double angle = j * angleStep * M_PI / 180.0; // Convert to radians
            double ueX = gNbX + ueRadius * cos(angle);
            double ueY = gNbY + ueRadius * sin(angle);

            staPositionAlloc->Add(Vector(ueX, ueY, ueHeight));
        }
    }

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.Install(gNbNodes);

    mobility.SetPositionAllocator(staPositionAlloc);
    mobility.Install(ueNodes);
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper); // Corrected - no casting needed

    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;

    OperationBandInfo band;

    // Define a single Component Carrier (CC)
    std::unique_ptr<ComponentCarrierInfo> cc0(new ComponentCarrierInfo());
    std::unique_ptr<BandwidthPartInfo> bwp0(new BandwidthPartInfo());

    if (contiguousCc)
    {
        const uint8_t numContiguousCcs = 1; // Only 1 CC

        // Create the configuration for the CcBwpHelper
        CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequencyBand,100e6, numContiguousCcs);

        bandConf.m_numBwp = 1; // 1 BWP per CC

        // Create the operation band
        band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    }
    else
    {
        band.m_centralFrequency = centralFrequencyBand;
        band.m_channelBandwidth = 100e6;  // 100 MHz total
        band.m_lowerFrequency = band.m_centralFrequency - band.m_channelBandwidth / 2;
        band.m_higherFrequency = band.m_centralFrequency + band.m_channelBandwidth / 2;

        // Component Carrier 0
        cc0->m_ccId = 0;
        cc0->m_centralFrequency = centralFrequencyBand;
        cc0->m_channelBandwidth = 100e6;  // Use only 100 MHz
        cc0->m_lowerFrequency = cc0->m_centralFrequency - cc0->m_channelBandwidth / 2;
        cc0->m_higherFrequency = cc0->m_centralFrequency + cc0->m_channelBandwidth / 2;

        // Bandwidth Part 0
        bwp0->m_bwpId = 0;
        bwp0->m_centralFrequency = cc0->m_centralFrequency;
        bwp0->m_channelBandwidth = cc0->m_channelBandwidth;  // Single BWP covers full CC
        bwp0->m_lowerFrequency = bwp0->m_centralFrequency - bwp0->m_channelBandwidth / 2;
        bwp0->m_higherFrequency = bwp0->m_centralFrequency + bwp0->m_channelBandwidth / 2;
        cc0->AddBwp(std::move(bwp0));
        // Add CC to the operation band.
        band.AddCc(std::move(cc0));
    }
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false)); //Disables shadow fading, meaning signals won’t experience random
    channelHelper->AssignChannelsToBands({band});//Assigns the operation band to the configured channel.
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(10)));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));//This sets the TDMA Round Robin (TDMA-RR) scheduler.
    // Beamforming method
    if (cellScan)
    {
        idealBeamformingHelper->SetAttribute("BeamformingMethod",TypeIdValue(CellScanBeamforming::GetTypeId()));
        idealBeamformingHelper->SetBeamformingAlgorithmAttribute("BeamSearchAngleStep",DoubleValue(beamSearchAngleStep));
    }
    else
    {
        idealBeamformingHelper->SetAttribute("BeamformingMethod",TypeIdValue(DirectPathBeamforming::GetTypeId()));
    }
    allBwps = CcBwpCreator::GetAllBwps({band});
    double x = pow(10, totalTxPower / 10);
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",PointerValue(CreateObject<IsotropicAntennaModel>()));
    uint32_t bwpIdForAllUsers = 0;
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB",UintegerValue(bwpIdForAllUsers));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForAllUsers));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_PREMIUM",UintegerValue(bwpIdForAllUsers));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VOICE_VIDEO_GAMING",UintegerValue(bwpIdForAllUsers));
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    // Set the default gateway for the UEs
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // attach UEs to the closest gNB before creating the dedicated flows
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    // install UDP applications
    uint16_t dlPort = 1234;
    uint16_t ulPort = dlPort + gNbNum * ueNumPergNb * numFlowsUe + 1;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        for (uint16_t flow = 0; flow < numFlowsUe; ++flow)
        {
            if (!disableDl)
            {
                PacketSinkHelper dlPacketSinkHelper(
                    "ns3::UdpSocketFactory",
                    InetSocketAddress(Ipv4Address::GetAny(), dlPort));
                serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));

                UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
                dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSizeBe));
                dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambdaUll)));
                dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
                clientApps.Add(dlClient.Install(remoteHost));

                Ptr<NrEpcTft> tft = Create<NrEpcTft>();
                NrEpcTft::PacketFilter dlpf;
                dlpf.localPortStart = dlPort;
                dlpf.localPortEnd = dlPort;
                ++dlPort;
                tft->Add(dlpf);

                enum NrEpsBearer::Qci q;
                if (flow == 0)
                {
                    q = NrEpsBearer::NGBR_LOW_LAT_EMBB;
                }
                else if (flow == 1)
                {
                    q = NrEpsBearer::GBR_CONV_VOICE;
                }
                else if (flow == 2)
                {
                    q = NrEpsBearer::NGBR_VIDEO_TCP_PREMIUM;
                }
                else if (flow == 3)
                {
                    q = NrEpsBearer::NGBR_VOICE_VIDEO_GAMING;
                }
                else
                {
                    q = NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT;
                }
                NrEpsBearer bearer(q);
                nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(u), bearer, tft);
            }

            if (!disableUl)
            {
                PacketSinkHelper ulPacketSinkHelper(
                    "ns3::UdpSocketFactory",
                    InetSocketAddress(Ipv4Address::GetAny(), ulPort));
                serverApps.Add(ulPacketSinkHelper.Install(remoteHost));

                UdpClientHelper ulClient(remoteHostAddr, ulPort);
                ulClient.SetAttribute("PacketSize", UintegerValue(udpPacketSizeBe));
                ulClient.SetAttribute("Interval", TimeValue(Seconds(1.2 / lambdaUll)));
                ulClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
                clientApps.Add(ulClient.Install(ueNodes.Get(u)));

                Ptr<NrEpcTft> tft = Create<NrEpcTft>();
                NrEpcTft::PacketFilter ulpf;
                ulpf.remotePortStart = ulPort;
                ulpf.remotePortEnd = ulPort;
                ++ulPort;
                tft->Add(ulpf);

                enum NrEpsBearer::Qci q;
                if (flow == 0)
                {
                    q = NrEpsBearer::NGBR_LOW_LAT_EMBB;
                }
                else if (flow == 1)
                {
                    q = NrEpsBearer::GBR_CONV_VOICE;
                }
                else if (flow == 2)
                {
                    q = NrEpsBearer::NGBR_VIDEO_TCP_PREMIUM;
                }
                else if (flow == 3)
                {
                    q = NrEpsBearer::NGBR_VOICE_VIDEO_GAMING;
                }
                else{
                    q = NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT;
                }
                NrEpsBearer bearer(q);
                nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(u), bearer, tft);
            }
        }
    }
    serverApps.Start(Seconds(udpAppStartTime));
    clientApps.Start(Seconds(udpAppStartTime));
    serverApps.Stop(Seconds(simTime));
    clientApps.Stop(Seconds(simTime));
    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(ueNodes);
    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.005));  // Increase bin width for TDMA
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;
    int numSlots = 10;
    std::ofstream outFile;
    std::string filename = "results/outputs.txt";
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << filename << std::endl;
        return 1;
    }
    outFile.setf(std::ios_base::fixed);
    std::cout << "Writing results to " << filename << std::endl;
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end();
         ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        std::stringstream protoStream;
        protoStream << (uint16_t)t.protocol;
        if (t.protocol == 6)
        {
            protoStream.str("TCP");
        }
        if (t.protocol == 17)
        {
            protoStream.str("UDP");
        }
        std::cout<< "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "<< t.destinationAddress << ":" << t.destinationPort << ") proto "
<< protoStream.str() << "\n";
        std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
        std::cout<< "  Tx Bytes:   " << i->second.txBytes << "\n";
        std::cout<< "  TxOffered:  "
                << i->second.txBytes * 8.0 / (simTime - udpAppStartTime) / 1000 / 1000 << " Mbps\n";
        std::cout<< "  Rx Bytes:   " << i->second.rxBytes << "\n";
        if (i->second.rxPackets > 0)
        {
            double rxDuration = (simTime - udpAppStartTime);
            averageFlowThroughput += i->second.rxBytes * 8.0 / rxDuration / 1000 / 1000;
            averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;
            double slotDuration = rxDuration / numSlots;
            for (int slot = 0; slot < numSlots; slot++)
            {
                double slotThroughput = i->second.rxBytes * 8.0 / slotDuration / 1000 / 1000;
                std::cout<< "  TDMA Slot " << slot + 1 << " Throughput: " << slotThroughput << " Mbps\n";
            }
            double slotDelay = (i->second.rxPackets > 0) ? (1000 * i->second.delaySum.GetSeconds()) / i->second.rxPackets : 0;
            std::cout<< "  TDMA Slot Mean Delay: " << slotDelay << " ms\n";
            std::cout<< "  Mean jitter:  "
                    << 1000 * i->second.jitterSum.GetSeconds() / i->second.rxPackets << " ms\n";
        }
        else
        {
            std::cout<< "  Throughput:  0 Mbps\n";
            std::cout << "  Mean delay:  0 ms\n";
            std::cout << "  Mean jitter: 0 ms\n";
        }
        std::cout<< "  Rx Packets: " << i->second.rxPackets << "\n";
    }

    std::cout<< "\n\n  Mean flow throughput: " << averageFlowThroughput / stats.size() << "\n";
    std::cout<< "  Mean flow delay: " << averageFlowDelay / stats.size() << "\n";
    std::cout<< "\n  Total TDMA Slots Used: " << numSlots << "\n";

    outFile.close();

    std::ifstream f(filename.c_str());

    if (f.is_open())
    {
        std::cout << f.rdbuf();
    }
    Simulator::Stop(Seconds(simTime));
    Simulator::Destroy();
    return 0;
}
