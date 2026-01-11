/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * NB-IoT SPS vs Dynamic Scheduling Latency Simulation
 *
 * Complete implementation based on:
 * "Latency Reduction for Narrowband LTE with Semi-Persistent Scheduling"
 * by Zubair Amjad et al., IEEE IDAACS 2018
 *
 * ============================================================================
 * SIMULATION SPECIFICATIONS
 * ============================================================================
 *
 * System Configuration:
 * - Bandwidth: 1.4 MHz (6 PRBs for NB-IoT)
 * - UE Category: Cat-M1 equivalent
 * - Subframe duration: 1 ms (LTE standard)
 * - TTI: 1 ms
 *
 * Dynamic Scheduling (DS) Latency Components:
 * - T_sr: Scheduling Request delay (~1-3 ms average)
 * - T_sg: Scheduling Grant delay (~4 ms)
 * - T_ul: Uplink transmission delay (~1 ms)
 * Total DS latency: ~10.83 ms (single UE)
 *
 * Semi-Persistent Scheduling (SPS) Latency Components:
 * - T_wait: Wait for pre-allocated slot (~T_sps/2 average)
 * - T_ul: Uplink transmission delay (~1 ms)
 * Total SPS latency: ~4.82 ms (single UE, 20ms period)
 *
 * Traffic Model:
 * - Periodic uplink MTC traffic
 * - Inter-arrival time: 20 ms (configurable)
 * - Packet size: 40 bytes (typical sensor data)
 *
 * Metrics:
 * - MAC-to-MAC latency (from packet ready to received at eNB)
 * - Average, min, max, std deviation
 * - 95th percentile latency
 * - Per-UE statistics
 *
 * Expected Results (from paper):
 * - Single UE DS: 10.83 ms
 * - Single UE SPS: 4.82 ms
 * - Latency reduction: ~55%
 * ============================================================================
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
#include <random>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NbIotSpsLatencySimulation");

//=============================================================================
// LATENCY MODEL CONSTANTS (from 3GPP specifications)
//=============================================================================
namespace LatencyModel
{
    // Time intervals (in milliseconds)
    constexpr double TTI_MS = 1.0;              // Transmission Time Interval
    constexpr double SUBFRAME_MS = 1.0;         // LTE subframe duration
    
    // Dynamic Scheduling Components
    constexpr double SR_PERIODICITY_MS = 5.0;   // SR opportunity period
    constexpr double SR_PROCESSING_MS = 1.0;    // SR processing at eNB
    constexpr double SG_DELAY_MS = 4.0;         // Scheduling Grant delay
    constexpr double UL_TX_MS = 1.0;            // Uplink transmission
    constexpr double HARQ_RTT_MS = 8.0;         // HARQ round-trip time
    
    // Average DS latency = SR_wait + SR_process + SG_delay + UL_tx
    // = SR_PERIODICITY/2 + SR_PROCESSING + SG_DELAY + UL_TX
    // = 2.5 + 1 + 4 + 1 = 8.5 ms (ideal)
    // With contention: ~10.83 ms
    
    // SPS latency = SPS_wait + UL_tx = SPS_PERIOD/2 + UL_TX
    // = 10 + 1 = 11 ms for 20ms period... 
    // Actually paper shows 4.82ms because data arrives synchronized
}

//=============================================================================
// Global Variables
//=============================================================================
static NbIotLatencyStats g_dsStats;             // DS latency statistics
static NbIotLatencyStats g_spsStats;            // SPS latency statistics
static NbIotLatencyStats* g_currentStats = nullptr;
static uint32_t g_packetSequence = 0;           // Global packet sequence number
static bool g_useSps = false;                   // Current scheduling mode
static std::ofstream g_packetLog;               // Per-packet log file
static Time g_simulationStartTime = Seconds(1); // Warmup period

//=============================================================================
// Simulation Configuration
//=============================================================================
struct SimulationConfig
{
    // Network topology
    uint32_t numUes = 1;                        // Number of UEs (1-100)
    double cellRadius = 500.0;                  // Cell radius in meters
    
    // Timing
    Time simulationTime = Seconds(30);          // Total simulation time
    Time warmupTime = Seconds(2);               // Warmup period (no stats)
    
    // Traffic model
    Time packetInterval = MilliSeconds(20);     // Inter-packet interval
    uint32_t packetSize = 40;                   // Packet size in bytes
    
    // SPS configuration
    Time spsInterval = MilliSeconds(20);        // SPS period
    bool useSps = false;                        // Use SPS or DS
    
    // Channel model
    double dlBandwidth = 1.4e6;                 // 1.4 MHz (6 PRBs)
    double ulBandwidth = 1.4e6;                 // 1.4 MHz
    double dlFrequency = 900e6;                 // 900 MHz band
    double ulFrequency = 850e6;                 // 900 MHz band (separate UL)
    
    // Simulation control
    std::string outputPrefix = "nbiot-sps";     // Output file prefix
    uint32_t runNumber = 1;                     // Random seed run number
    bool verbose = false;                       // Verbose logging
    bool enableTraces = true;                   // Enable trace output
    
    // Scalability analysis
    bool runScalability = false;                // Run scalability test
    uint32_t minUes = 1;                        // Min UEs for scalability
    uint32_t maxUes = 100;                      // Max UEs for scalability
    uint32_t ueStep = 10;                       // Step size
    
    // Multiple runs for confidence
    uint32_t numRuns = 1;                       // Number of simulation runs
};

//=============================================================================
// Latency Record Structure
//=============================================================================
struct LatencyRecord
{
    Time timestamp;         // When packet was received
    Time latency;           // End-to-end latency
    uint16_t rnti;          // UE identifier
    uint32_t seqNum;        // Packet sequence number
    bool isSps;             // SPS or DS
    uint32_t numUes;        // Number of UEs in simulation
};

static std::vector<LatencyRecord> g_allRecords;

//=============================================================================
// Callback: Packet ready at UE MAC
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
                 << " at " << now.GetMilliSeconds() << " ms"
                 << " [" << (isSps ? "SPS" : "DS") << "]");
}

//=============================================================================
// Callback: Packet received at eNB MAC
//=============================================================================
void
EnbPacketReceivedCallback(Ptr<const Packet> packet, uint16_t rnti, uint32_t numUes)
{
    Time now = Simulator::Now();
    
    if (now < g_simulationStartTime)
    {
        return;
    }
    
    NbIotLatencyTag tag;
    if (packet->PeekPacketTag(tag))
    {
        Time latency = now - tag.GetTimestamp();
        
        // Record to appropriate stats
        if (g_currentStats)
        {
            g_currentStats->RecordLatency(
                tag.GetRnti(),
                latency,
                tag.IsSps(),
                tag.GetSequenceNumber()
            );
        }
        
        // Store detailed record
        LatencyRecord record;
        record.timestamp = now;
        record.latency = latency;
        record.rnti = tag.GetRnti();
        record.seqNum = tag.GetSequenceNumber();
        record.isSps = tag.IsSps();
        record.numUes = numUes;
        g_allRecords.push_back(record);
        
        NS_LOG_DEBUG("Packet " << tag.GetSequenceNumber() 
                     << " from UE " << rnti
                     << ", latency=" << latency.GetMilliSeconds() << " ms");
        
        // Log to CSV
        if (g_packetLog.is_open())
        {
            g_packetLog << std::fixed << std::setprecision(3)
                       << now.GetMilliSeconds() << ","
                       << tag.GetRnti() << ","
                       << tag.GetSequenceNumber() << ","
                       << latency.GetMicroSeconds() / 1000.0 << ","
                       << (tag.IsSps() ? "SPS" : "DS") << ","
                       << numUes << "\n";
        }
    }
}

//=============================================================================
// Periodic Traffic Generator Class
//=============================================================================
class PeriodicTrafficGenerator : public Object
{
public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::PeriodicTrafficGenerator")
            .SetParent<Object>()
            .SetGroupName("NbIot");
        return tid;
    }
    
    PeriodicTrafficGenerator()
        : m_device(nullptr)
        , m_packetSize(40)
        , m_interval(MilliSeconds(20))
        , m_stopTime(Seconds(10))
        , m_rnti(0)
        , m_isSps(false)
        , m_numUes(1)
        , m_running(false)
    {
    }
    
    void Setup(Ptr<NbIotUeNetDevice> device, uint32_t packetSize,
               Time interval, Time stopTime, uint16_t rnti, bool isSps, uint32_t numUes)
    {
        m_device = device;
        m_packetSize = packetSize;
        m_interval = interval;
        m_stopTime = stopTime;
        m_rnti = rnti;
        m_isSps = isSps;
        m_numUes = numUes;
    }
    
    void Start(Time startTime)
    {
        m_running = true;
        Simulator::Schedule(startTime, &PeriodicTrafficGenerator::SendPacket, this);
    }
    
    void Stop()
    {
        m_running = false;
    }
    
private:
    void SendPacket()
    {
        if (!m_running || Simulator::Now() >= m_stopTime)
        {
            return;
        }
        
        // Create packet
        Ptr<Packet> packet = Create<Packet>(m_packetSize);
        
        // Add latency tag
        UePacketReadyCallback(packet, m_rnti, m_isSps);
        
        // Send through UE MAC
        if (m_device)
        {
            Ptr<NbIotUeMac> mac = m_device->GetMac();
            if (mac)
            {
                mac->TransmitSdu(1, packet);  // LCID 1 for data
            }
        }
        
        // Schedule next packet
        Simulator::Schedule(m_interval, &PeriodicTrafficGenerator::SendPacket, this);
    }
    
    Ptr<NbIotUeNetDevice> m_device;
    uint32_t m_packetSize;
    Time m_interval;
    Time m_stopTime;
    uint16_t m_rnti;
    bool m_isSps;
    uint32_t m_numUes;
    bool m_running;
};

//=============================================================================
// Run a single simulation scenario
//=============================================================================
void
RunScenario(const SimulationConfig& config, NbIotLatencyStats& stats)
{
    NS_LOG_INFO("Running " << (config.useSps ? "SPS" : "DS") 
                << " scenario with " << config.numUes << " UEs");
    
    g_useSps = config.useSps;
    g_currentStats = &stats;
    g_packetSequence = 0;
    stats.Reset();
    
    // Create nodes
    NodeContainer enbNodes;
    enbNodes.Create(1);
    
    NodeContainer ueNodes;
    ueNodes.Create(config.numUes);
    
    // Configure mobility - fixed positions
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    // Place eNB at center
    Ptr<ListPositionAllocator> enbPositions = CreateObject<ListPositionAllocator>();
    enbPositions->Add(Vector(0.0, 0.0, 30.0));  // 30m height (typical small cell)
    mobility.SetPositionAllocator(enbPositions);
    mobility.Install(enbNodes);
    
    // Place UEs uniformly around eNB
    Ptr<ListPositionAllocator> uePositions = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < config.numUes; i++)
    {
        double angle = 2.0 * M_PI * i / config.numUes;
        double r = config.cellRadius * 0.5;  // Place at half radius
        uePositions->Add(Vector(r * cos(angle), r * sin(angle), 1.5));
    }
    mobility.SetPositionAllocator(uePositions);
    mobility.Install(ueNodes);
    
    // Create NB-IoT helper
    NbIotHelper nbiotHelper;
    
    // Configure scheduler based on mode
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
    
    // Set simulation start time for latency measurement
    g_simulationStartTime = config.warmupTime;
    
    // Install traffic generators on each UE
    std::vector<Ptr<PeriodicTrafficGenerator>> generators;
    for (uint32_t i = 0; i < ueDevices.GetN(); i++)
    {
        Ptr<NbIotUeNetDevice> ueDev = DynamicCast<NbIotUeNetDevice>(ueDevices.Get(i));
        
        Ptr<PeriodicTrafficGenerator> gen = CreateObject<PeriodicTrafficGenerator>();
        
        // Get RNTI from MAC
        Ptr<NbIotUeMac> mac = ueDev->GetMac();
        uint16_t rnti = mac ? mac->GetRnti() : (i + 1);
        
        gen->Setup(
            ueDev,
            config.packetSize,
            config.packetInterval,
            config.simulationTime,
            rnti,
            config.useSps,
            config.numUes
        );
        
        // Stagger start times slightly to avoid synchronized arrivals
        Time startOffset = MilliSeconds(i % 10);
        gen->Start(config.warmupTime + startOffset);
        
        generators.push_back(gen);
    }
    
    // Run simulation
    Simulator::Stop(config.simulationTime);
    Simulator::Run();
    
    // Stop generators
    for (auto& gen : generators)
    {
        gen->Stop();
    }
    
    Simulator::Destroy();
}

//=============================================================================
// Export detailed packet latency data to CSV
//=============================================================================
void
ExportPacketLatencyData(const std::string& filename)
{
    std::ofstream file(filename);
    
    file << "timestamp_ms,rnti,sequence_number,latency_ms,scheduling_type,num_ues\n";
    
    for (const auto& record : g_allRecords)
    {
        file << std::fixed << std::setprecision(3)
             << record.timestamp.GetMilliSeconds() << ","
             << record.rnti << ","
             << record.seqNum << ","
             << record.latency.GetMicroSeconds() / 1000.0 << ","
             << (record.isSps ? "SPS" : "DS") << ","
             << record.numUes << "\n";
    }
    
    file.close();
    NS_LOG_INFO("Exported " << g_allRecords.size() << " records to " << filename);
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
    
    file << "num_ues,ds_avg_latency_ms,ds_stddev_ms,sps_avg_latency_ms,sps_stddev_ms,"
         << "latency_reduction_ms,latency_reduction_pct,throughput_ratio\n";
    
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
            
            double reductionMs = (dsAvg.GetMicroSeconds() - spsAvg.GetMicroSeconds()) / 1000.0;
            double reductionPct = 0;
            if (dsAvg.GetMicroSeconds() > 0)
            {
                reductionPct = 100.0 * reductionMs / (dsAvg.GetMicroSeconds() / 1000.0);
            }
            double throughputRatio = spsAvg.GetMicroSeconds() > 0 ? 
                dsAvg.GetMicroSeconds() / (double)spsAvg.GetMicroSeconds() : 1.0;
            
            file << std::fixed << std::setprecision(3)
                 << numUes << ","
                 << dsAvg.GetMicroSeconds() / 1000.0 << ","
                 << dsStd.GetMicroSeconds() / 1000.0 << ","
                 << spsAvg.GetMicroSeconds() / 1000.0 << ","
                 << spsStd.GetMicroSeconds() / 1000.0 << ","
                 << reductionMs << ","
                 << reductionPct << ","
                 << throughputRatio << "\n";
        }
    }
    
    file.close();
    NS_LOG_INFO("Exported aggregated results to " << filename);
}

//=============================================================================
// Export DS vs SPS comparison summary
//=============================================================================
void
ExportComparisonSummary(const std::string& filename,
                        const NbIotLatencyStats& dsStats,
                        const NbIotLatencyStats& spsStats)
{
    std::ofstream file(filename);
    
    file << "Metric,Dynamic_Scheduling,Semi_Persistent_Scheduling,Improvement\n";
    
    double dsAvg = dsStats.GetAverageLatency().GetMicroSeconds() / 1000.0;
    double spsAvg = spsStats.GetAverageLatency().GetMicroSeconds() / 1000.0;
    double dsMin = dsStats.GetMinLatency().GetMicroSeconds() / 1000.0;
    double spsMin = spsStats.GetMinLatency().GetMicroSeconds() / 1000.0;
    double dsMax = dsStats.GetMaxLatency().GetMicroSeconds() / 1000.0;
    double spsMax = spsStats.GetMaxLatency().GetMicroSeconds() / 1000.0;
    double dsStd = dsStats.GetStdDevLatency().GetMicroSeconds() / 1000.0;
    double spsStd = spsStats.GetStdDevLatency().GetMicroSeconds() / 1000.0;
    double ds95 = dsStats.Get95thPercentileLatency().GetMicroSeconds() / 1000.0;
    double sps95 = spsStats.Get95thPercentileLatency().GetMicroSeconds() / 1000.0;
    
    auto formatImprovement = [](double ds, double sps) -> std::string {
        if (ds > 0) {
            double pct = 100.0 * (ds - sps) / ds;
            return std::to_string(pct) + "%";
        }
        return "N/A";
    };
    
    file << std::fixed << std::setprecision(3);
    file << "Average Latency (ms)," << dsAvg << "," << spsAvg << "," << formatImprovement(dsAvg, spsAvg) << "\n";
    file << "Min Latency (ms)," << dsMin << "," << spsMin << "," << formatImprovement(dsMin, spsMin) << "\n";
    file << "Max Latency (ms)," << dsMax << "," << spsMax << "," << formatImprovement(dsMax, spsMax) << "\n";
    file << "Std Deviation (ms)," << dsStd << "," << spsStd << "," << formatImprovement(dsStd, spsStd) << "\n";
    file << "95th Percentile (ms)," << ds95 << "," << sps95 << "," << formatImprovement(ds95, sps95) << "\n";
    file << "Packet Count," << dsStats.GetPacketCount() << "," << spsStats.GetPacketCount() << ",N/A\n";
    
    file.close();
    NS_LOG_INFO("Exported comparison summary to " << filename);
}

//=============================================================================
// Print simulation banner
//=============================================================================
void
PrintBanner()
{
    std::cout << "\n";
    std::cout << "================================================================\n";
    std::cout << "  NB-IoT SPS vs Dynamic Scheduling Latency Simulation\n";
    std::cout << "================================================================\n";
    std::cout << "  Based on: 'Latency Reduction for Narrowband LTE with\n";
    std::cout << "            Semi-Persistent Scheduling' (IEEE IDAACS 2018)\n";
    std::cout << "================================================================\n\n";
}

//=============================================================================
// Print configuration summary
//=============================================================================
void
PrintConfig(const SimulationConfig& config)
{
    std::cout << "Configuration:\n";
    std::cout << "  - Number of UEs: " << config.numUes << "\n";
    std::cout << "  - Simulation time: " << config.simulationTime.GetSeconds() << " s\n";
    std::cout << "  - Warmup time: " << config.warmupTime.GetSeconds() << " s\n";
    std::cout << "  - Packet interval: " << config.packetInterval.GetMilliSeconds() << " ms\n";
    std::cout << "  - Packet size: " << config.packetSize << " bytes\n";
    std::cout << "  - Cell radius: " << config.cellRadius << " m\n";
    std::cout << "  - SPS interval: " << config.spsInterval.GetMilliSeconds() << " ms\n";
    std::cout << "  - Output prefix: " << config.outputPrefix << "\n";
    std::cout << "\n";
}

//=============================================================================
// Print results summary
//=============================================================================
void
PrintResults(const std::string& label, const NbIotLatencyStats& stats)
{
    std::cout << label << " Results:\n";
    std::cout << "  - Total packets: " << stats.GetPacketCount() << "\n";
    std::cout << "  - Average latency: " << std::fixed << std::setprecision(2)
              << stats.GetAverageLatency().GetMicroSeconds() / 1000.0 << " ms\n";
    std::cout << "  - Std deviation: " << std::fixed << std::setprecision(2)
              << stats.GetStdDevLatency().GetMicroSeconds() / 1000.0 << " ms\n";
    std::cout << "  - Min latency: " << std::fixed << std::setprecision(2)
              << stats.GetMinLatency().GetMicroSeconds() / 1000.0 << " ms\n";
    std::cout << "  - Max latency: " << std::fixed << std::setprecision(2)
              << stats.GetMaxLatency().GetMicroSeconds() / 1000.0 << " ms\n";
    std::cout << "  - 95th percentile: " << std::fixed << std::setprecision(2)
              << stats.Get95thPercentileLatency().GetMicroSeconds() / 1000.0 << " ms\n";
    std::cout << "\n";
}

//=============================================================================
// Main function
//=============================================================================
int
main(int argc, char* argv[])
{
    // Default configuration
    SimulationConfig config;
    
    // Parse command line arguments
    CommandLine cmd;
    cmd.AddValue("numUes", "Number of UEs", config.numUes);
    cmd.AddValue("simTime", "Simulation time in seconds", config.simulationTime);
    cmd.AddValue("warmupTime", "Warmup time in seconds", config.warmupTime);
    cmd.AddValue("packetInterval", "Packet interval in milliseconds", config.packetInterval);
    cmd.AddValue("packetSize", "Packet size in bytes", config.packetSize);
    cmd.AddValue("spsInterval", "SPS interval in milliseconds", config.spsInterval);
    cmd.AddValue("cellRadius", "Cell radius in meters", config.cellRadius);
    cmd.AddValue("outputPrefix", "Output file prefix", config.outputPrefix);
    cmd.AddValue("runNumber", "Run number for random seed", config.runNumber);
    cmd.AddValue("verbose", "Enable verbose logging", config.verbose);
    cmd.AddValue("runScalability", "Run scalability analysis", config.runScalability);
    cmd.AddValue("minUes", "Minimum UEs for scalability test", config.minUes);
    cmd.AddValue("maxUes", "Maximum UEs for scalability test", config.maxUes);
    cmd.AddValue("ueStep", "UE step for scalability test", config.ueStep);
    cmd.Parse(argc, argv);
    
    // Set random seed
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(config.runNumber);
    
    // Configure logging
    if (config.verbose)
    {
        LogComponentEnable("NbIotSpsLatencySimulation", LOG_LEVEL_DEBUG);
        LogComponentEnable("NbIotSpsScheduler", LOG_LEVEL_DEBUG);
        LogComponentEnable("NbIotLatencyTag", LOG_LEVEL_DEBUG);
    }
    else
    {
        LogComponentEnable("NbIotSpsLatencySimulation", LOG_LEVEL_INFO);
    }
    
    PrintBanner();
    
    if (config.runScalability)
    {
        // ===== SCALABILITY ANALYSIS MODE =====
        std::cout << "Running Scalability Analysis\n";
        std::cout << "  UE range: " << config.minUes << " to " << config.maxUes 
                  << " (step: " << config.ueStep << ")\n\n";
        
        std::map<uint32_t, std::pair<Time, Time>> dsResults;
        std::map<uint32_t, std::pair<Time, Time>> spsResults;
        
        // Open packet log
        g_packetLog.open(config.outputPrefix + "-packet-latency.csv");
        g_packetLog << "timestamp_ms,rnti,sequence_number,latency_ms,scheduling_type,num_ues\n";
        
        for (uint32_t numUes = config.minUes; numUes <= config.maxUes; numUes += config.ueStep)
        {
            if (numUes == 0) numUes = 1;
            
            std::cout << "Testing with " << numUes << " UEs...\n";
            
            // Run Dynamic Scheduling scenario
            config.numUes = numUes;
            config.useSps = false;
            RunScenario(config, g_dsStats);
            
            Time dsAvg = g_dsStats.GetAverageLatency();
            Time dsStd = g_dsStats.GetStdDevLatency();
            dsResults[numUes] = std::make_pair(dsAvg, dsStd);
            
            std::cout << "  DS:  Avg=" << std::fixed << std::setprecision(2)
                      << dsAvg.GetMicroSeconds() / 1000.0 << " ms\n";
            
            // Run SPS scenario
            config.useSps = true;
            RunScenario(config, g_spsStats);
            
            Time spsAvg = g_spsStats.GetAverageLatency();
            Time spsStd = g_spsStats.GetStdDevLatency();
            spsResults[numUes] = std::make_pair(spsAvg, spsStd);
            
            std::cout << "  SPS: Avg=" << std::fixed << std::setprecision(2)
                      << spsAvg.GetMicroSeconds() / 1000.0 << " ms\n";
            
            // Calculate improvement
            if (dsAvg.GetMicroSeconds() > 0)
            {
                double reduction = 100.0 * (dsAvg.GetMicroSeconds() - spsAvg.GetMicroSeconds()) 
                                 / dsAvg.GetMicroSeconds();
                std::cout << "  Improvement: " << std::fixed << std::setprecision(1) 
                          << reduction << "%\n";
            }
            std::cout << "\n";
        }
        
        g_packetLog.close();
        
        // Export results
        ExportAggregatedResults(config.outputPrefix + "-aggregated.csv", dsResults, spsResults);
        ExportPacketLatencyData(config.outputPrefix + "-detailed.csv");
        
        std::cout << "Results exported to:\n";
        std::cout << "  - " << config.outputPrefix << "-aggregated.csv\n";
        std::cout << "  - " << config.outputPrefix << "-detailed.csv\n";
        std::cout << "  - " << config.outputPrefix << "-packet-latency.csv\n";
    }
    else
    {
        // ===== SINGLE COMPARISON MODE =====
        PrintConfig(config);
        
        // Open packet log
        g_packetLog.open(config.outputPrefix + "-packet-latency.csv");
        g_packetLog << "timestamp_ms,rnti,sequence_number,latency_ms,scheduling_type,num_ues\n";
        
        std::cout << "Running Dynamic Scheduling scenario...\n";
        config.useSps = false;
        RunScenario(config, g_dsStats);
        
        std::cout << "Running Semi-Persistent Scheduling scenario...\n";
        config.useSps = true;
        RunScenario(config, g_spsStats);
        
        g_packetLog.close();
        
        // Print results
        std::cout << "\n================================================================\n";
        std::cout << "  RESULTS\n";
        std::cout << "================================================================\n\n";
        
        PrintResults("Dynamic Scheduling (DS)", g_dsStats);
        PrintResults("Semi-Persistent Scheduling (SPS)", g_spsStats);
        
        // Calculate improvement
        double dsAvg = g_dsStats.GetAverageLatency().GetMicroSeconds() / 1000.0;
        double spsAvg = g_spsStats.GetAverageLatency().GetMicroSeconds() / 1000.0;
        
        std::cout << "================================================================\n";
        std::cout << "  COMPARISON\n";
        std::cout << "================================================================\n";
        std::cout << "  DS Average:  " << std::fixed << std::setprecision(2) << dsAvg << " ms\n";
        std::cout << "  SPS Average: " << std::fixed << std::setprecision(2) << spsAvg << " ms\n";
        std::cout << "  Reduction:   " << std::fixed << std::setprecision(2) << (dsAvg - spsAvg) << " ms\n";
        if (dsAvg > 0)
        {
            std::cout << "  Improvement: " << std::fixed << std::setprecision(1) 
                      << (100.0 * (dsAvg - spsAvg) / dsAvg) << "%\n";
        }
        std::cout << "================================================================\n\n";
        
        // Export results
        ExportComparisonSummary(config.outputPrefix + "-comparison.csv", g_dsStats, g_spsStats);
        ExportPacketLatencyData(config.outputPrefix + "-detailed.csv");
        
        std::cout << "Results exported to:\n";
        std::cout << "  - " << config.outputPrefix << "-comparison.csv\n";
        std::cout << "  - " << config.outputPrefix << "-detailed.csv\n";
        std::cout << "  - " << config.outputPrefix << "-packet-latency.csv\n";
    }
    
    std::cout << "\nSimulation complete.\n";
    
    return 0;
}
