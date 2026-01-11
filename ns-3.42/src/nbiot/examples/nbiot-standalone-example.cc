/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Standalone Deployment Example
 *
 * This example demonstrates a basic NB-IoT network simulation with:
 * - One eNB deployed in standalone mode
 * - Multiple UEs with power saving enabled (PSM and eDRX)
 * - Coverage class determination based on distance
 * - Basic statistics collection
 *
 * Network topology:
 *
 *       [UE1]---100m---[eNB]---200m---[UE2]---300m---[UE3]
 *
 * Expected coverage classes based on MCL:
 * - UE1 (100m): CE Level 0 (normal coverage)
 * - UE2 (200m): CE Level 0 (normal coverage)
 * - UE3 (300m): CE Level 1 (extended coverage)
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"

#include "ns3/nbiot-helper.h"
#include "ns3/nbiot-common.h"
#include "ns3/nbiot-net-device.h"
#include "ns3/nbiot-power-saving.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NbIotStandaloneExample");

// Global trace callbacks
void
RrcStateChange(std::string context, NbIotRrcState oldState, NbIotRrcState newState)
{
    std::ostringstream oldStr, newStr;
    oldStr << oldState;
    newStr << newState;
    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s " << context
                << " RRC state: " << oldStr.str()
                << " -> " << newStr.str());
}

void
PsmStateChange(NbIotPsmState oldState, NbIotPsmState newState)
{
    std::ostringstream oldStr, newStr;
    oldStr << static_cast<int>(oldState);
    newStr << static_cast<int>(newState);
    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s "
                << "PSM state: " << oldStr.str()
                << " -> " << newStr.str());
}

void
EdrxStateChange(NbIotEdrxState oldState, NbIotEdrxState newState)
{
    std::ostringstream oldStr, newStr;
    oldStr << static_cast<int>(oldState);
    newStr << static_cast<int>(newState);
    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s "
                << "eDRX state: " << oldStr.str()
                << " -> " << newStr.str());
}

void
PrintProgress()
{
    std::cout << "Simulation time: " << Simulator::Now().GetSeconds() << " s" << std::endl;
    Simulator::Schedule(Seconds(10), &PrintProgress);
}

int
main(int argc, char* argv[])
{
    // Simulation parameters
    uint32_t numUes = 10;
    double simTime = 100.0; // seconds
    double cellRadius = 500.0; // meters
    bool enablePsm = true;
    bool enableEdrx = true;
    double t3324 = 10.0; // Active timer in seconds
    double t3412 = 3600.0; // TAU timer in seconds (1 hour)
    double edrxCycle = 81.92; // eDRX cycle in seconds
    double ptwDuration = 2.56; // Paging time window in seconds
    std::string deploymentMode = "standalone";

    // Command line arguments
    CommandLine cmd(__FILE__);
    cmd.AddValue("numUes", "Number of UE devices", numUes);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("cellRadius", "Cell radius in meters", cellRadius);
    cmd.AddValue("enablePsm", "Enable Power Saving Mode", enablePsm);
    cmd.AddValue("enableEdrx", "Enable extended DRX", enableEdrx);
    cmd.AddValue("t3324", "T3324 active timer in seconds", t3324);
    cmd.AddValue("t3412", "T3412 TAU timer in seconds", t3412);
    cmd.AddValue("edrxCycle", "eDRX cycle in seconds", edrxCycle);
    cmd.AddValue("ptw", "Paging time window in seconds", ptwDuration);
    cmd.AddValue("deployment", "Deployment mode (standalone, guardband, inband)", deploymentMode);
    cmd.Parse(argc, argv);

    // Enable logging
    LogComponentEnable("NbIotStandaloneExample", LOG_LEVEL_INFO);
    // Uncomment for detailed logging:
    // LogComponentEnable("NbIotHelper", LOG_LEVEL_INFO);
    // LogComponentEnable("NbIotPowerSaving", LOG_LEVEL_INFO);
    // LogComponentEnable("NbIotRrc", LOG_LEVEL_INFO);

    NS_LOG_INFO("===========================================");
    NS_LOG_INFO("NB-IoT Standalone Deployment Example");
    NS_LOG_INFO("===========================================");
    NS_LOG_INFO("Number of UEs: " << numUes);
    NS_LOG_INFO("Simulation time: " << simTime << " s");
    NS_LOG_INFO("Cell radius: " << cellRadius << " m");
    NS_LOG_INFO("PSM enabled: " << (enablePsm ? "Yes" : "No"));
    NS_LOG_INFO("eDRX enabled: " << (enableEdrx ? "Yes" : "No"));
    NS_LOG_INFO("===========================================");

    // Determine deployment mode
    NbIotDeploymentMode mode = NbIotDeploymentMode::STANDALONE;
    if (deploymentMode == "guardband")
    {
        mode = NbIotDeploymentMode::GUARD_BAND;
    }
    else if (deploymentMode == "inband")
    {
        mode = NbIotDeploymentMode::IN_BAND;
    }

    // Create nodes
    NodeContainer enbNode;
    NodeContainer ueNodes;
    enbNode.Create(1);
    ueNodes.Create(numUes);

    // Set up mobility - eNB at center, UEs distributed in cell
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.Install(enbNode);
    enbNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 30)); // 30m height

    // Distribute UEs randomly within cell radius
    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                    "X", DoubleValue(0.0),
                                    "Y", DoubleValue(0.0),
                                    "rho", DoubleValue(cellRadius));
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);

    // Create NB-IoT helper
    Ptr<NbIotHelper> nbiotHelper = CreateObject<NbIotHelper>();
    nbiotHelper->SetDeploymentMode(mode);
    nbiotHelper->SetEarfcn(0, 18000); // Typical NB-IoT EARFCN values

    // Install devices
    NetDeviceContainer enbDevices = nbiotHelper->InstallEnbDevice(enbNode);
    NetDeviceContainer ueDevices = nbiotHelper->InstallUeDevice(ueNodes);

    NS_LOG_INFO("Installed " << enbDevices.GetN() << " eNB devices");
    NS_LOG_INFO("Installed " << ueDevices.GetN() << " UE devices");

    // Attach all UEs to the eNB
    nbiotHelper->AttachToClosestEnb(ueDevices, enbDevices);

    // Configure power saving for UEs
    if (enablePsm || enableEdrx)
    {
        nbiotHelper->EnablePowerSaving(ueDevices, enablePsm, enableEdrx);
        
        if (enablePsm)
        {
            nbiotHelper->SetPsmTimers(ueDevices, Seconds(t3324), Seconds(t3412));
            NS_LOG_INFO("PSM timers: T3324=" << t3324 << "s, T3412=" << t3412 << "s");
        }
        
        if (enableEdrx)
        {
            nbiotHelper->SetEdrxParameters(ueDevices, Seconds(edrxCycle), Seconds(ptwDuration));
            NS_LOG_INFO("eDRX params: cycle=" << edrxCycle << "s, PTW=" << ptwDuration << "s");
        }
    }

    // Print UE positions and coverage classes
    NS_LOG_INFO("-------------------------------------------");
    NS_LOG_INFO("UE Distribution:");
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ueNode = ueNodes.Get(i);
        Vector pos = ueNode->GetObject<MobilityModel>()->GetPosition();
        double distance = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
        
        // Estimate coverage class based on distance
        // Using simplified MCL model: MCL = 120 + 35*log10(d) for urban
        double mcl = 120.0 + 35.0 * std::log10(std::max(1.0, distance));
        NbIotCoverageClass cc = NbIotCoverageHelper::DetermineCoverageClass(mcl);
        
        NS_LOG_INFO("  UE " << i << ": pos=(" << pos.x << ", " << pos.y << "), "
                    << "distance=" << distance << "m, "
                    << "MCL=" << mcl << "dB, "
                    << "Coverage=" << CoverageClassToString(cc));
    }
    NS_LOG_INFO("-------------------------------------------");

    // Enable traces
    nbiotHelper->EnableTraces();

    // Schedule progress printing
    Simulator::Schedule(Seconds(10), &PrintProgress);

    // Run simulation
    NS_LOG_INFO("Starting simulation...");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Print final statistics
    NS_LOG_INFO("===========================================");
    NS_LOG_INFO("Simulation completed");
    NS_LOG_INFO("===========================================");

    // Calculate estimated battery life for first UE
    Ptr<NbIotUeNetDevice> firstUe = DynamicCast<NbIotUeNetDevice>(ueDevices.Get(0));
    if (firstUe && firstUe->GetPowerSavingController())
    {
        // Assuming 2000 mAh battery at 3V
        double batteryCapacity = 2000; // mAh
        double voltage = 3.0; // V
        double batteryLife = firstUe->GetPowerSavingController()
            ->GetEstimatedBatteryLife(batteryCapacity, voltage);
        
        NS_LOG_INFO("Estimated battery life (UE 0): " << batteryLife << " hours ("
                    << (batteryLife / 24 / 365) << " years)");
    }

    Simulator::Destroy();

    return 0;
}
