/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * NB-IoT SPS vs Dynamic Scheduling Latency Simulation
 *
 * This simulation compares Semi-Persistent Scheduling (SPS) with Dynamic
 * Scheduling (DS) for uplink latency in NB-IoT MTC/URLLC applications.
 *
 * Based on: "Latency Reduction for Narrowband LTE with Semi-Persistent 
 * Scheduling" (IEEE IDAACS 2018)
 *
 * Key Parameters:
 * - Bandwidth: 1.4 MHz (6 PRBs for NB-IoT)
 * - UE Category: Cat-M1 equivalent
 * - Traffic: Periodic uplink MTC
 * - Metric: MAC-to-MAC latency
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/applications-module.h"

// NB-IoT module
#include "ns3/nbiot-common.h"
#include "ns3/nbiot-helper.h"
#include "ns3/nbiot-net-device.h"
#include "ns3/nbiot-enb-mac.h"
#include "ns3/nbiot-ue-mac.h"
#include "ns3/nbiot-sps-scheduler.h"
#include "ns3/nbiot-latency-tag.h"

#include <fstream>
#include <iomanip>
#include <vector>
#include <map>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NbIotSpsSimulation");

//=============================================================================
// Global Variables
//=============================================================================
static NbIotLatencyStats g_latencyStats;        // Global latency statistics
static uint32_t g_packetSequence = 0;            // Global packet sequence number
static bool g_useSps = false;                    // Current scheduling mode
static std::ofstream g_packetLog;                // Per-packet log file
static Time g_simulationStartTime = Seconds(1);  // Warmup period

//=============================================================================
// Simulation Parameters
//=============================================================================
struct SimulationConfig
{
    uint32_t numUes = 1;                    // Number of UEs
    Time simulationTime = Seconds(10);       // Total simulation time
    Time warmupTime = Seconds(1);            // Warmup period
    Time packetInterval = MilliSeconds(20);  // Inter-packet interval
    uint32_t packetSize = 40;                // Packet size in bytes
    Time spsInterval = MilliSeconds(20);     // SPS period
    bool useSps = false;                     // Use SPS or DS
    std::string outputPrefix = "nbiot-sps";  // Output file prefix
    uint32_t runNumber = 1;                  // Random seed run number
    bool verbose = false;                    // Verbose logging
};

//=============================================================================
// Callback: UE MAC layer - packet ready for transmission
//=============================================================================
void
UePacketReadyCallback(Ptr<Packet> packet, uint16_t rnti, bool isSps)
{
    Time now = Simulator::Now();
    
    if (now < g_simulationStartTime)
    {
        return; // Skip warmup
    }
    
    // Tag packet with timestamp
    NbIotLatencyTag tag(now);
    tag.SetRnti(rnti);
    tag.SetSequenceNumber(++g_packetSequence);
    tag.SetIsSps(isSps);
    packet->AddPacketTag(tag);
    
    NS_LOG_DEBUG("Packet " << g_packetSequence << " queued at UE " << rnti 
                 << " at " << now.GetMilliSeconds() << " ms");
}

//=============================================================================
// Callback: eNB MAC layer - packet received
//=============================================================================
void
EnbPacketReceivedCallback(Ptr<const Packet> packet, uint16_t rnti)
{
    Time now = Simulator::Now();
    
    if (now < g_simulationStartTime)
    {
        return; // Skip warmup
    }
    
    NbIotLatencyTag tag;
    if (packet->PeekPacketTag(tag))
    {
        Time latency = now - tag.GetTimestamp();
        
        g_latencyStats.RecordLatency(
            tag.GetRnti(),
            latency,
            tag.IsSps(),
            tag.GetSequenceNumber()
        );
        
        NS_LOG_DEBUG("Packet " << tag.GetSequenceNumber() 
                     << " received from UE " << rnti
                     << ", latency=" << latency.GetMilliSeconds() << " ms"
                     << ", SPS=" << (tag.IsSps() ? "yes" : "no"));
        
        // Log to CSV
        if (g_packetLog.is_open())
        {
            g_packetLog << now.GetMilliSeconds() << ","
                       << tag.GetRnti() << ","
                       << tag.GetSequenceNumber() << ","
                       << latency.GetMicroSeconds() / 1000.0 << ","
                       << (tag.IsSps() ? "SPS" : "DS") << "\n";
        }
    }
}

//=============================================================================
// Helper: Install periodic traffic generator
//=============================================================================
void
InstallPeriodicTraffic(Ptr<Node> ueNode, Ptr<NbIotUeNetDevice> ueDev,
                       Time interval, uint32_t packetSize, Time startTime, Time stopTime)
{
    // Schedule periodic packet generation
    // This simulates an application generating uplink data periodically
    
    class PeriodicGenerator
    {
    public:
        Ptr<NbIotUeNetDevice> device;
        uint32_t packetSize;
        Time interval;
        Time stopTime;
        uint16_t rnti;
        bool isSps;
        
        void Send()
        {
            Time now = Simulator::Now();
            if (now < stopTime)
            {
                // Create packet
                Ptr<Packet> packet = Create<Packet>(packetSize);
                
                // Trigger packet ready callback for latency tracking
                UePacketReadyCallback(packet, rnti, isSps);
                
                // Send through UE MAC
                Ptr<NbIotUeMac> mac = device->GetMac();
                if (mac)
                {
                    mac->TransmitSdu(1, packet);  // LCID 1 for data
                }
                
                // Schedule next
                Simulator::Schedule(interval, &PeriodicGenerator::Send, this);
            }
        }
    };
    
    auto* gen = new PeriodicGenerator();
    gen->device = ueDev;
    gen->packetSize = packetSize;
    gen->interval = interval;
    gen->stopTime = stopTime;
    // Get RNTI from MAC layer
    Ptr<NbIotUeMac> mac = ueDev->GetMac();
    gen->rnti = mac ? mac->GetRnti() : 0;
    gen->isSps = g_useSps;
    
    Simulator::Schedule(startTime, &PeriodicGenerator::Send, gen);
}

//=============================================================================
// Run a single simulation scenario
//=============================================================================
void
RunScenario(const SimulationConfig& config)
{
    NS_LOG_INFO("Running " << (config.useSps ? "SPS" : "DS") 
                << " scenario with " << config.numUes << " UEs");
    
    g_useSps = config.useSps;
    g_packetSequence = 0;
    g_latencyStats.Reset();
    
    // Create nodes
    NodeContainer enbNodes;
    enbNodes.Create(1);
    
    NodeContainer ueNodes;
    ueNodes.Create(config.numUes);
    
    // Mobility - fixed positions
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    Ptr<ListPositionAllocator> enbPositions = CreateObject<ListPositionAllocator>();
    enbPositions->Add(Vector(0.0, 0.0, 30.0));  // eNB at center, 30m height
    mobility.SetPositionAllocator(enbPositions);
    mobility.Install(enbNodes);
    
    // UEs at random positions around eNB
    Ptr<ListPositionAllocator> uePositions = CreateObject<ListPositionAllocator>();
    double radius = 500.0;  // 500m cell radius
    for (uint32_t i = 0; i < config.numUes; i++)
    {
        double angle = 2.0 * M_PI * i / config.numUes;
        double r = radius * 0.5;  // Place UEs at half radius
        uePositions->Add(Vector(r * cos(angle), r * sin(angle), 1.5));
    }
    mobility.SetPositionAllocator(uePositions);
    mobility.Install(ueNodes);
    
    // Create NB-IoT helper
    NbIotHelper nbiotHelper;
    
    // Configure scheduler
    if (config.useSps)
    {
        nbiotHelper.SetSchedulerType("ns3::NbIotSpsScheduler");
        nbiotHelper.SetSchedulerAttribute("DefaultSpsInterval", TimeValue(config.spsInterval));
        nbiotHelper.SetSchedulerAttribute("AutoActivateSps", BooleanValue(true));
    }
    else
    {
        nbiotHelper.SetSchedulerType("ns3::NbIotRoundRobinScheduler");
    }
    
    // Install NB-IoT devices
    NetDeviceContainer enbDevices = nbiotHelper.InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevices = nbiotHelper.InstallUeDevice(ueNodes);
    
    // Attach all UEs to eNB
    nbiotHelper.Attach(ueDevices, enbDevices.Get(0));
    
    // Connect trace callbacks for latency measurement
    for (uint32_t i = 0; i < enbDevices.GetN(); i++)
    {
        Ptr<NbIotEnbNetDevice> enbDev = DynamicCast<NbIotEnbNetDevice>(enbDevices.Get(i));
        Ptr<NbIotEnbMac> enbMac = enbDev->GetMac();
        
        // Connect to UL reception trace
        // enbMac->TraceConnectWithoutContext("UlReception", MakeCallback(&EnbPacketReceivedCallback));
    }
    
    // Install traffic generators
    for (uint32_t i = 0; i < ueDevices.GetN(); i++)
    {
        Ptr<NbIotUeNetDevice> ueDev = DynamicCast<NbIotUeNetDevice>(ueDevices.Get(i));
        
        InstallPeriodicTraffic(
            ueNodes.Get(i),
            ueDev,
            config.packetInterval,
            config.packetSize,
            config.warmupTime,
            config.simulationTime
        );
    }
    
    // Set simulation start time for latency measurement
    g_simulationStartTime = config.warmupTime;
    
    // Run simulation
    Simulator::Stop(config.simulationTime);
    Simulator::Run();
    Simulator::Destroy();
}

//=============================================================================
// Export aggregated results to CSV
//=============================================================================
void
ExportAggregatedResults(const std::string& filename,
                        const std::map<uint32_t, std::pair<Time, Time>>& dsResults,
                        const std::map<uint32_t, std::pair<Time, Time>>& spsResults)
{
    std::ofstream file(filename);
    
    file << "num_ues,ds_avg_latency_ms,ds_stddev_ms,sps_avg_latency_ms,sps_stddev_ms,latency_reduction_pct\n";
    
    for (const auto& ds : dsResults)
    {
        uint32_t numUes = ds.first;
        Time dsAvg = ds.second.first;
        Time dsStd = ds.second.second;
        
        auto spsIt = spsResults.find(numUes);
        if (spsIt != spsResults.end())
        {
            Time spsAvg = spsIt->second.first;
            Time spsStd = spsIt->second.second;
            
            double reduction = 100.0 * (dsAvg.GetMicroSeconds() - spsAvg.GetMicroSeconds()) 
                             / dsAvg.GetMicroSeconds();
            
            file << numUes << ","
                 << dsAvg.GetMicroSeconds() / 1000.0 << ","
                 << dsStd.GetMicroSeconds() / 1000.0 << ","
                 << spsAvg.GetMicroSeconds() / 1000.0 << ","
                 << spsStd.GetMicroSeconds() / 1000.0 << ","
                 << reduction << "\n";
        }
    }
    
    file.close();
    NS_LOG_INFO("Exported aggregated results to " << filename);
}

//=============================================================================
// Export DS vs SPS comparison to CSV
//=============================================================================
void
ExportComparisonResults(const std::string& filename,
                        Time dsAvg, Time spsAvg,
                        uint32_t dsCount, uint32_t spsCount)
{
    std::ofstream file(filename);
    
    file << "metric,dynamic_scheduling,semi_persistent_scheduling,improvement\n";
    
    double dsMs = dsAvg.GetMicroSeconds() / 1000.0;
    double spsMs = spsAvg.GetMicroSeconds() / 1000.0;
    double improvement = (dsMs - spsMs) / dsMs * 100.0;
    
    file << "average_latency_ms," << dsMs << "," << spsMs << "," << improvement << "%\n";
    file << "packet_count," << dsCount << "," << spsCount << ",N/A\n";
    file << "latency_ratio," << dsMs / spsMs << ",1.0,N/A\n";
    
    file.close();
    NS_LOG_INFO("Exported comparison results to " << filename);
}

//=============================================================================
// Main function
//=============================================================================
int
main(int argc, char* argv[])
{
    // Default configuration
    SimulationConfig config;
    uint32_t minUes = 1;
    uint32_t maxUes = 100;
    uint32_t ueStep = 10;
    bool runScalability = false;
    
    // Parse command line
    CommandLine cmd;
    cmd.AddValue("numUes", "Number of UEs", config.numUes);
    cmd.AddValue("simTime", "Simulation time in seconds", config.simulationTime);
    cmd.AddValue("warmupTime", "Warmup time in seconds", config.warmupTime);
    cmd.AddValue("packetInterval", "Packet interval in milliseconds", config.packetInterval);
    cmd.AddValue("packetSize", "Packet size in bytes", config.packetSize);
    cmd.AddValue("spsInterval", "SPS interval in milliseconds", config.spsInterval);
    cmd.AddValue("useSps", "Use SPS (true) or DS (false)", config.useSps);
    cmd.AddValue("outputPrefix", "Output file prefix", config.outputPrefix);
    cmd.AddValue("runNumber", "Run number for random seed", config.runNumber);
    cmd.AddValue("verbose", "Enable verbose logging", config.verbose);
    cmd.AddValue("runScalability", "Run scalability analysis (1-100 UEs)", runScalability);
    cmd.AddValue("minUes", "Minimum UEs for scalability test", minUes);
    cmd.AddValue("maxUes", "Maximum UEs for scalability test", maxUes);
    cmd.AddValue("ueStep", "UE step for scalability test", ueStep);
    cmd.Parse(argc, argv);
    
    // Set random seed
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(config.runNumber);
    
    // Configure logging
    if (config.verbose)
    {
        LogComponentEnable("NbIotSpsSimulation", LOG_LEVEL_DEBUG);
        LogComponentEnable("NbIotSpsScheduler", LOG_LEVEL_DEBUG);
        LogComponentEnable("NbIotLatencyTag", LOG_LEVEL_DEBUG);
    }
    else
    {
        LogComponentEnable("NbIotSpsSimulation", LOG_LEVEL_INFO);
    }
    
    std::cout << "======================================\n";
    std::cout << "NB-IoT SPS vs DS Latency Simulation\n";
    std::cout << "======================================\n\n";
    
    if (runScalability)
    {
        // Run scalability analysis
        std::map<uint32_t, std::pair<Time, Time>> dsResults;
        std::map<uint32_t, std::pair<Time, Time>> spsResults;
        
        // Open detailed packet log
        g_packetLog.open(config.outputPrefix + "-packet-latency.csv");
        g_packetLog << "timestamp_ms,rnti,sequence_number,latency_ms,scheduling_type\n";
        
        for (uint32_t numUes = minUes; numUes <= maxUes; numUes += ueStep)
        {
            if (numUes == 0) numUes = 1;  // Start with at least 1 UE
            
            std::cout << "Testing with " << numUes << " UEs...\n";
            
            // Run DS scenario
            config.numUes = numUes;
            config.useSps = false;
            RunScenario(config);
            
            Time dsAvg = g_latencyStats.GetAverageLatency();
            Time dsStd = g_latencyStats.GetStdDevLatency();
            dsResults[numUes] = std::make_pair(dsAvg, dsStd);
            
            std::cout << "  DS:  Avg=" << dsAvg.GetMicroSeconds() / 1000.0 
                      << " ms, StdDev=" << dsStd.GetMicroSeconds() / 1000.0 << " ms\n";
            
            // Run SPS scenario
            config.useSps = true;
            RunScenario(config);
            
            Time spsAvg = g_latencyStats.GetAverageLatency();
            Time spsStd = g_latencyStats.GetStdDevLatency();
            spsResults[numUes] = std::make_pair(spsAvg, spsStd);
            
            std::cout << "  SPS: Avg=" << spsAvg.GetMicroSeconds() / 1000.0 
                      << " ms, StdDev=" << spsStd.GetMicroSeconds() / 1000.0 << " ms\n";
            
            double reduction = 100.0 * (dsAvg.GetMicroSeconds() - spsAvg.GetMicroSeconds()) 
                             / dsAvg.GetMicroSeconds();
            std::cout << "  Improvement: " << reduction << "%\n\n";
        }
        
        g_packetLog.close();
        
        // Export aggregated results
        ExportAggregatedResults(config.outputPrefix + "-aggregated.csv", dsResults, spsResults);
        
        // Export to individual latency CSVs
        g_latencyStats.ExportToCsv(config.outputPrefix + "-detailed.csv");
    }
    else
    {
        // Single scenario run
        std::cout << "Configuration:\n";
        std::cout << "  Number of UEs: " << config.numUes << "\n";
        std::cout << "  Simulation time: " << config.simulationTime.GetSeconds() << " s\n";
        std::cout << "  Packet interval: " << config.packetInterval.GetMilliSeconds() << " ms\n";
        std::cout << "  Packet size: " << config.packetSize << " bytes\n";
        std::cout << "  Scheduling: " << (config.useSps ? "SPS" : "DS") << "\n";
        if (config.useSps)
        {
            std::cout << "  SPS interval: " << config.spsInterval.GetMilliSeconds() << " ms\n";
        }
        std::cout << "\n";
        
        // Open packet log
        g_packetLog.open(config.outputPrefix + "-packet-latency.csv");
        g_packetLog << "timestamp_ms,rnti,sequence_number,latency_ms,scheduling_type\n";
        
        RunScenario(config);
        
        g_packetLog.close();
        
        // Print results
        std::cout << "\n======================================\n";
        std::cout << "Results:\n";
        std::cout << "======================================\n";
        std::cout << "Total packets: " << g_latencyStats.GetPacketCount() << "\n";
        std::cout << "Average latency: " << g_latencyStats.GetAverageLatency().GetMicroSeconds() / 1000.0 << " ms\n";
        std::cout << "Std deviation: " << g_latencyStats.GetStdDevLatency().GetMicroSeconds() / 1000.0 << " ms\n";
        std::cout << "Min latency: " << g_latencyStats.GetMinLatency().GetMicroSeconds() / 1000.0 << " ms\n";
        std::cout << "Max latency: " << g_latencyStats.GetMaxLatency().GetMicroSeconds() / 1000.0 << " ms\n";
        std::cout << "95th percentile: " << g_latencyStats.Get95thPercentileLatency().GetMicroSeconds() / 1000.0 << " ms\n";
        
        // Export results
        g_latencyStats.ExportToCsv(config.outputPrefix + "-detailed.csv");
    }
    
    std::cout << "\nSimulation complete.\n";
    
    return 0;
}
