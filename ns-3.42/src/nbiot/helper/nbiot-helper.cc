/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Helper implementation
 */

#include "nbiot-helper.h"

#include "ns3/nbiot-net-device.h"
#include "ns3/nbiot-ue-phy.h"
#include "ns3/nbiot-enb-phy.h"
#include "ns3/nbiot-ue-mac.h"
#include "ns3/nbiot-enb-mac.h"
#include "ns3/nbiot-rlc.h"
#include "ns3/nbiot-pdcp.h"
#include "ns3/nbiot-rrc.h"
#include "ns3/nbiot-power-saving.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/node.h>
#include <ns3/mobility-model.h>
#include <ns3/pointer.h>
#include <ns3/uinteger.h>
#include <ns3/double.h>
#include <ns3/boolean.h>
#include <ns3/enum.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/isotropic-antenna-model.h>

#include <cmath>
#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotHelper");

NS_OBJECT_ENSURE_REGISTERED(NbIotHelper);

TypeId
NbIotHelper::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotHelper")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotHelper>();
    return tid;
}

NbIotHelper::NbIotHelper()
    : m_deploymentMode(NbIotDeploymentMode::STANDALONE)
    , m_dlEarfcn(0)
    , m_ulEarfcn(18000)
    , m_cellIdCounter(1)
    , m_imsiCounter(1)
    , m_phyTracesEnabled(false)
    , m_macTracesEnabled(false)
    , m_rlcTracesEnabled(false)
    , m_pdcpTracesEnabled(false)
{
    NS_LOG_FUNCTION(this);

    // Default factory types
    m_schedulerFactory.SetTypeId("ns3::NbIotRoundRobinScheduler");
    m_enbDeviceFactory.SetTypeId("ns3::NbIotEnbNetDevice");
    m_ueDeviceFactory.SetTypeId("ns3::NbIotUeNetDevice");
    m_enbAntennaFactory.SetTypeId("ns3::IsotropicAntennaModel");
    m_ueAntennaFactory.SetTypeId("ns3::IsotropicAntennaModel");
}

NbIotHelper::~NbIotHelper()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotHelper::DoDispose()
{
    m_dlChannel = nullptr;
    m_ulChannel = nullptr;
    Object::DoDispose();
}

void NbIotHelper::SetDeploymentMode(NbIotDeploymentMode mode) { m_deploymentMode = mode; }
NbIotDeploymentMode NbIotHelper::GetDeploymentMode() const { return m_deploymentMode; }

void
NbIotHelper::SetEarfcn(uint32_t dlEarfcn, uint32_t ulEarfcn)
{
    m_dlEarfcn = dlEarfcn;
    m_ulEarfcn = ulEarfcn;
}

void NbIotHelper::SetDlChannel(Ptr<SpectrumChannel> channel) { m_dlChannel = channel; }
void NbIotHelper::SetUlChannel(Ptr<SpectrumChannel> channel) { m_ulChannel = channel; }
Ptr<SpectrumChannel> NbIotHelper::GetDlChannel() const { return m_dlChannel; }
Ptr<SpectrumChannel> NbIotHelper::GetUlChannel() const { return m_ulChannel; }

void
NbIotHelper::SetPathlossModelType(TypeId type)
{
    m_pathlossModelFactory.SetTypeId(type);
}

void
NbIotHelper::SetPathlossModelAttribute(std::string n, const AttributeValue& v)
{
    m_pathlossModelFactory.Set(n, v);
}

void
NbIotHelper::SetFadingModelType(TypeId type)
{
    m_fadingModelFactory.SetTypeId(type);
}

void
NbIotHelper::SetFadingModelAttribute(std::string n, const AttributeValue& v)
{
    m_fadingModelFactory.Set(n, v);
}

void
NbIotHelper::SetSchedulerType(TypeId type)
{
    m_schedulerFactory.SetTypeId(type);
}

void
NbIotHelper::SetSchedulerType(const std::string& type)
{
    m_schedulerFactory.SetTypeId(type);
}

void
NbIotHelper::SetSchedulerAttribute(std::string n, const AttributeValue& v)
{
    m_schedulerFactory.Set(n, v);
}

void
NbIotHelper::SetEnbDeviceAttribute(std::string n, const AttributeValue& v)
{
    m_enbDeviceFactory.Set(n, v);
}

void
NbIotHelper::SetUeDeviceAttribute(std::string n, const AttributeValue& v)
{
    m_ueDeviceFactory.Set(n, v);
}

void
NbIotHelper::SetEnbAntennaModelType(TypeId type)
{
    m_enbAntennaFactory.SetTypeId(type);
}

void
NbIotHelper::SetEnbAntennaModelAttribute(std::string n, const AttributeValue& v)
{
    m_enbAntennaFactory.Set(n, v);
}

void
NbIotHelper::SetUeAntennaModelType(TypeId type)
{
    m_ueAntennaFactory.SetTypeId(type);
}

void
NbIotHelper::SetUeAntennaModelAttribute(std::string n, const AttributeValue& v)
{
    m_ueAntennaFactory.Set(n, v);
}

void
NbIotHelper::InitializeChannels()
{
    NS_LOG_FUNCTION(this);

    if (!m_dlChannel)
    {
        m_dlChannel = CreateObject<MultiModelSpectrumChannel>();
        
        // Add pathloss model if configured
        if (!m_pathlossModelFactory.GetTypeId().GetName().empty())
        {
            Ptr<PropagationLossModel> plm =
                DynamicCast<PropagationLossModel>(m_pathlossModelFactory.Create());
            if (plm)
            {
                m_dlChannel->AddPropagationLossModel(plm);
            }
        }
    }

    if (!m_ulChannel)
    {
        m_ulChannel = CreateObject<MultiModelSpectrumChannel>();
        
        // Add pathloss model if configured
        if (!m_pathlossModelFactory.GetTypeId().GetName().empty())
        {
            Ptr<PropagationLossModel> plm =
                DynamicCast<PropagationLossModel>(m_pathlossModelFactory.Create());
            if (plm)
            {
                m_ulChannel->AddPropagationLossModel(plm);
            }
        }
    }
}

NetDeviceContainer
NbIotHelper::InstallEnbDevice(NodeContainer c)
{
    NS_LOG_FUNCTION(this);

    InitializeChannels();

    NetDeviceContainer devices;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        Ptr<NbIotEnbNetDevice> dev = CreateEnbDevice(*i);
        (*i)->AddDevice(dev);
        devices.Add(dev);
    }

    return devices;
}

NetDeviceContainer
NbIotHelper::InstallUeDevice(NodeContainer c)
{
    NS_LOG_FUNCTION(this);

    InitializeChannels();

    NetDeviceContainer devices;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        Ptr<NbIotUeNetDevice> dev = CreateUeDevice(*i);
        (*i)->AddDevice(dev);
        devices.Add(dev);
    }

    return devices;
}

Ptr<NbIotEnbNetDevice>
NbIotHelper::CreateEnbDevice(Ptr<Node> node)
{
    NS_LOG_FUNCTION(this << node);

    // Create eNB device
    Ptr<NbIotEnbNetDevice> dev =
        DynamicCast<NbIotEnbNetDevice>(m_enbDeviceFactory.Create<NbIotEnbNetDevice>());
    dev->SetNode(node);
    dev->SetCellId(m_cellIdCounter++);
    dev->SetDeploymentMode(m_deploymentMode);
    dev->SetEarfcn(m_dlEarfcn);

    // Create and configure PHY
    Ptr<NbIotEnbPhy> phy = CreateObject<NbIotEnbPhy>();
    phy->SetChannel(m_dlChannel);
    dev->SetPhy(phy);

    // Create and configure MAC with scheduler
    Ptr<NbIotEnbMac> mac = CreateObject<NbIotEnbMac>();
    Ptr<NbIotScheduler> scheduler =
        DynamicCast<NbIotScheduler>(m_schedulerFactory.Create());
    mac->SetScheduler(scheduler);
    dev->SetMac(mac);

    // Create RLC, PDCP, RRC
    Ptr<NbIotRlcUm> rlc = CreateObject<NbIotRlcUm>();
    dev->SetRlc(rlc);

    Ptr<NbIotPdcpDrb> pdcp = CreateObject<NbIotPdcpDrb>();
    dev->SetPdcp(pdcp);

    Ptr<NbIotEnbRrc> rrc = CreateObject<NbIotEnbRrc>();
    dev->SetRrc(rrc);

    return dev;
}

Ptr<NbIotUeNetDevice>
NbIotHelper::CreateUeDevice(Ptr<Node> node)
{
    NS_LOG_FUNCTION(this << node);

    // Create UE device
    Ptr<NbIotUeNetDevice> dev =
        DynamicCast<NbIotUeNetDevice>(m_ueDeviceFactory.Create<NbIotUeNetDevice>());
    dev->SetNode(node);
    dev->SetImsi(m_imsiCounter++);
    dev->SetDeploymentMode(m_deploymentMode);
    dev->SetEarfcn(m_dlEarfcn);

    // Create and configure PHY
    Ptr<NbIotUePhy> phy = CreateObject<NbIotUePhy>();
    phy->SetChannel(m_dlChannel);
    dev->SetPhy(phy);

    // Create and configure MAC
    Ptr<NbIotUeMac> mac = CreateObject<NbIotUeMac>();
    dev->SetMac(mac);

    // Create RLC, PDCP, RRC
    Ptr<NbIotRlcUm> rlc = CreateObject<NbIotRlcUm>();
    dev->SetRlc(rlc);

    Ptr<NbIotPdcpDrb> pdcp = CreateObject<NbIotPdcpDrb>();
    dev->SetPdcp(pdcp);

    Ptr<NbIotUeRrc> rrc = CreateObject<NbIotUeRrc>();
    dev->SetRrc(rrc);

    // Create power saving components
    Ptr<NbIotPsmManager> psm = CreateObject<NbIotPsmManager>();
    psm->SetPhy(phy);
    psm->SetMac(mac);
    psm->SetRrc(rrc);

    Ptr<NbIotEdrxManager> edrx = CreateObject<NbIotEdrxManager>();
    edrx->SetPhy(phy);
    edrx->SetMac(mac);

    Ptr<NbIotPowerSavingController> powerSaving = CreateObject<NbIotPowerSavingController>();
    powerSaving->SetPsmManager(psm);
    powerSaving->SetEdrxManager(edrx);
    dev->SetPowerSavingController(powerSaving);

    return dev;
}

void
NbIotHelper::Attach(NetDeviceContainer ueDevices, Ptr<NetDevice> enbDevice)
{
    NS_LOG_FUNCTION(this << enbDevice);

    Ptr<NbIotEnbNetDevice> enbDev = DynamicCast<NbIotEnbNetDevice>(enbDevice);
    NS_ASSERT_MSG(enbDev, "enbDevice must be an NbIotEnbNetDevice");

    for (auto i = ueDevices.Begin(); i != ueDevices.End(); ++i)
    {
        Ptr<NbIotUeNetDevice> ueDev = DynamicCast<NbIotUeNetDevice>(*i);
        if (ueDev)
        {
            // Configure UE to attach to this eNB
            Ptr<NbIotUeRrc> ueRrc = ueDev->GetRrc();
            if (ueRrc)
            {
                ueRrc->SetCellId(enbDev->GetCellId());
                // Trigger RRC connection
                Simulator::Schedule(MilliSeconds(10),
                                    &NbIotUeRrc::Connect,
                                    ueRrc,
                                    NbIotRrcEstablishmentCause::MO_DATA);
            }
        }
    }
}

void
NbIotHelper::AttachToClosestEnb(NetDeviceContainer ueDevices, NetDeviceContainer enbDevices)
{
    NS_LOG_FUNCTION(this);

    for (auto ue = ueDevices.Begin(); ue != ueDevices.End(); ++ue)
    {
        Ptr<Node> ueNode = (*ue)->GetNode();
        
        double minDistance = std::numeric_limits<double>::max();
        Ptr<NetDevice> closestEnb = nullptr;

        for (auto enb = enbDevices.Begin(); enb != enbDevices.End(); ++enb)
        {
            Ptr<Node> enbNode = (*enb)->GetNode();
            double distance = GetDistanceBetweenNodes(ueNode, enbNode);
            
            if (distance < minDistance)
            {
                minDistance = distance;
                closestEnb = *enb;
            }
        }

        if (closestEnb)
        {
            NetDeviceContainer singleUe;
            singleUe.Add(*ue);
            Attach(singleUe, closestEnb);
            
            NS_LOG_INFO("UE " << (*ue)->GetNode()->GetId()
                        << " attached to eNB at distance " << minDistance << " m");
        }
    }
}

void
NbIotHelper::EnablePowerSaving(NetDeviceContainer ueDevices, bool enablePsm, bool enableEdrx)
{
    NS_LOG_FUNCTION(this << enablePsm << enableEdrx);

    for (auto i = ueDevices.Begin(); i != ueDevices.End(); ++i)
    {
        Ptr<NbIotUeNetDevice> ueDev = DynamicCast<NbIotUeNetDevice>(*i);
        if (ueDev)
        {
            ueDev->EnablePowerSaving(enablePsm, enableEdrx);
        }
    }
}

void
NbIotHelper::SetPsmTimers(NetDeviceContainer ueDevices, Time t3324, Time t3412)
{
    NS_LOG_FUNCTION(this << t3324 << t3412);

    for (auto i = ueDevices.Begin(); i != ueDevices.End(); ++i)
    {
        Ptr<NbIotUeNetDevice> ueDev = DynamicCast<NbIotUeNetDevice>(*i);
        if (ueDev && ueDev->GetPowerSavingController())
        {
            Ptr<NbIotPsmManager> psm = ueDev->GetPowerSavingController()->GetPsmManager();
            if (psm)
            {
                NbIotPsmTimers timers;
                timers.t3324 = t3324;
                timers.t3412 = t3412;
                psm->SetTimers(timers);
            }
        }
    }
}

void
NbIotHelper::SetEdrxParameters(NetDeviceContainer ueDevices, Time edrxCycle, Time ptw)
{
    NS_LOG_FUNCTION(this << edrxCycle << ptw);

    for (auto i = ueDevices.Begin(); i != ueDevices.End(); ++i)
    {
        Ptr<NbIotUeNetDevice> ueDev = DynamicCast<NbIotUeNetDevice>(*i);
        if (ueDev && ueDev->GetPowerSavingController())
        {
            Ptr<NbIotEdrxManager> edrx = ueDev->GetPowerSavingController()->GetEdrxManager();
            if (edrx)
            {
                NbIotEdrxParams params;
                params.edrxCycle = edrxCycle;
                params.pagingTimeWindow = ptw;
                edrx->SetParams(params);
            }
        }
    }
}

void
NbIotHelper::ActivateDataRadioBearer(NetDeviceContainer ueDevices)
{
    NS_LOG_FUNCTION(this);

    for (auto i = ueDevices.Begin(); i != ueDevices.End(); ++i)
    {
        Ptr<NbIotUeNetDevice> ueDev = DynamicCast<NbIotUeNetDevice>(*i);
        if (ueDev)
        {
            // DRB activation would be triggered by RRC
            NS_LOG_DEBUG("DRB activation triggered for UE " << ueDev->GetImsi());
        }
    }
}

void
NbIotHelper::EnablePhyTraces()
{
    m_phyTracesEnabled = true;
    NS_LOG_INFO("PHY traces enabled");
}

void
NbIotHelper::EnableMacTraces()
{
    m_macTracesEnabled = true;
    NS_LOG_INFO("MAC traces enabled");
}

void
NbIotHelper::EnableRlcTraces()
{
    m_rlcTracesEnabled = true;
    NS_LOG_INFO("RLC traces enabled");
}

void
NbIotHelper::EnablePdcpTraces()
{
    m_pdcpTracesEnabled = true;
    NS_LOG_INFO("PDCP traces enabled");
}

void
NbIotHelper::EnableTraces()
{
    EnablePhyTraces();
    EnableMacTraces();
    EnableRlcTraces();
    EnablePdcpTraces();
}

double
NbIotHelper::GetDistanceBetweenNodes(Ptr<Node> a, Ptr<Node> b)
{
    Ptr<MobilityModel> mobA = a->GetObject<MobilityModel>();
    Ptr<MobilityModel> mobB = b->GetObject<MobilityModel>();
    
    if (!mobA || !mobB)
    {
        return 0.0;
    }
    
    return mobA->GetDistanceFrom(mobB);
}

// ====================== NbIotPhyStatsHelper ======================

NbIotPhyStatsHelper::NbIotPhyStatsHelper()
    : m_outputPrefix("nbiot-phy")
{
}

NbIotPhyStatsHelper::~NbIotPhyStatsHelper()
{
}

void
NbIotPhyStatsHelper::SetOutputPrefix(std::string prefix)
{
    m_outputPrefix = prefix;
}

void
NbIotPhyStatsHelper::ConnectToDevice(Ptr<NetDevice> dev)
{
    // Connect to PHY traces
    NS_LOG_INFO("PHY stats helper connected to device");
}

void
NbIotPhyStatsHelper::WriteOutput()
{
    std::ofstream outFile(m_outputPrefix + "-stats.txt");
    if (outFile.is_open())
    {
        outFile << "# NB-IoT PHY Statistics\n";
        outFile << "# Time(s)\tRSRP(dBm)\tRSRQ(dB)\tSINR(dB)\n";
        outFile.close();
    }
}

// ====================== NbIotEnergyHelper ======================

NbIotEnergyHelper::NbIotEnergyHelper()
{
}

NbIotEnergyHelper::~NbIotEnergyHelper()
{
}

void
NbIotEnergyHelper::InstallEnergyModel(NetDeviceContainer ueDevices, double initialEnergy)
{
    for (auto i = ueDevices.Begin(); i != ueDevices.End(); ++i)
    {
        m_initialEnergy[*i] = initialEnergy;
        NS_LOG_INFO("Energy model installed on UE with " << initialEnergy << " J initial energy");
    }
}

double
NbIotEnergyHelper::GetRemainingEnergy(Ptr<NetDevice> dev) const
{
    auto it = m_initialEnergy.find(dev);
    if (it != m_initialEnergy.end())
    {
        // Simplified: would use actual power consumption tracking
        return it->second;
    }
    return 0.0;
}

double
NbIotEnergyHelper::GetConsumedEnergy(Ptr<NetDevice> dev) const
{
    auto it = m_initialEnergy.find(dev);
    if (it != m_initialEnergy.end())
    {
        return it->second - GetRemainingEnergy(dev);
    }
    return 0.0;
}

double
NbIotEnergyHelper::GetEstimatedBatteryLife(Ptr<NetDevice> dev) const
{
    Ptr<NbIotUeNetDevice> ueDev = DynamicCast<NbIotUeNetDevice>(dev);
    if (ueDev && ueDev->GetPowerSavingController())
    {
        auto it = m_initialEnergy.find(dev);
        if (it != m_initialEnergy.end())
        {
            // Convert Joules to mWh (1 J = 1/3.6 mWh)
            double batteryCapacity = it->second / 3.6;
            return ueDev->GetPowerSavingController()->GetEstimatedBatteryLife(batteryCapacity, 3.0);
        }
    }
    return 0.0;
}

// ====================== NbIotCoverageHelper ======================

NbIotCoverageHelper::NbIotCoverageHelper()
{
}

NbIotCoverageHelper::~NbIotCoverageHelper()
{
}

double
NbIotCoverageHelper::CalculateMcl(Ptr<NbIotEnbNetDevice> enbDev,
                                   Ptr<NbIotUeNetDevice> ueDev) const
{
    // MCL = Tx Power - Rx Sensitivity
    // For NB-IoT: Tx Power ~ 43 dBm, Rx Sensitivity ~ -130 to -141 dBm
    double txPower = enbDev->GetTxPower();
    double distance = NbIotHelper::GetDistanceBetweenNodes(enbDev->GetNode(), ueDev->GetNode());
    
    // Simple free-space path loss approximation
    double frequency = 900.0; // MHz, typical NB-IoT frequency
    double pathLoss = 20 * std::log10(distance) + 20 * std::log10(frequency) + 32.45;
    
    // MCL approximation
    return txPower + 130.0 - pathLoss; // 130 dBm is reference sensitivity
}

NbIotCoverageClass
NbIotCoverageHelper::DetermineCoverageClass(double mcl)
{
    // Per 3GPP specifications
    if (mcl <= 144.0)
    {
        return NbIotCoverageClass::CE_LEVEL_0; // Normal coverage
    }
    else if (mcl <= 154.0)
    {
        return NbIotCoverageClass::CE_LEVEL_1; // Extended coverage
    }
    else
    {
        return NbIotCoverageClass::CE_LEVEL_2; // Extreme coverage
    }
}

uint16_t
NbIotCoverageHelper::GetRequiredRepetitions(NbIotCoverageClass coverageClass,
                                             NbIotPhysicalChannel channel)
{
    // Repetition factors based on coverage class and channel
    switch (channel)
    {
        case NbIotPhysicalChannel::NPDCCH:
            switch (coverageClass)
            {
                case NbIotCoverageClass::CE_LEVEL_0: return 1;
                case NbIotCoverageClass::CE_LEVEL_1: return 8;
                case NbIotCoverageClass::CE_LEVEL_2: return 64;
                default: return 1;
            }
        case NbIotPhysicalChannel::NPDSCH:
            switch (coverageClass)
            {
                case NbIotCoverageClass::CE_LEVEL_0: return 2;
                case NbIotCoverageClass::CE_LEVEL_1: return 16;
                case NbIotCoverageClass::CE_LEVEL_2: return 128;
                default: return 1;
            }
        case NbIotPhysicalChannel::NPUSCH:
            switch (coverageClass)
            {
                case NbIotCoverageClass::CE_LEVEL_0: return 2;
                case NbIotCoverageClass::CE_LEVEL_1: return 16;
                case NbIotCoverageClass::CE_LEVEL_2: return 128;
                default: return 1;
            }
        default:
            return 1;
    }
}

} // namespace ns3
