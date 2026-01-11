/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Semi-Persistent Scheduling (SPS) Implementation
 */

#include "nbiot-sps-scheduler.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>
#include <ns3/boolean.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotSpsScheduler");

NS_OBJECT_ENSURE_REGISTERED(NbIotSpsScheduler);

TypeId
NbIotSpsScheduler::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotSpsScheduler")
        .SetParent<NbIotScheduler>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotSpsScheduler>()
        .AddAttribute("DefaultSpsInterval",
                      "Default SPS periodic interval",
                      TimeValue(MilliSeconds(20)),
                      MakeTimeAccessor(&NbIotSpsScheduler::m_defaultSpsInterval),
                      MakeTimeChecker())
        .AddAttribute("AutoActivateSps",
                      "Automatically activate SPS for new UEs",
                      BooleanValue(true),
                      MakeBooleanAccessor(&NbIotSpsScheduler::m_autoActivateSps),
                      MakeBooleanChecker());
    return tid;
}

NbIotSpsScheduler::NbIotSpsScheduler()
    : m_defaultSpsInterval(MilliSeconds(20))
    , m_autoActivateSps(true)
    , m_paddingTransmissions(0)
    , m_dataTransmissions(0)
    , m_lastDynamicRnti(0)
    , m_subframeCounter(0)
{
    NS_LOG_FUNCTION(this);
}

NbIotSpsScheduler::~NbIotSpsScheduler()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotSpsScheduler::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_spsConfigs.clear();
    NbIotScheduler::DoDispose();
}

void
NbIotSpsScheduler::ConfigureSps(uint16_t rnti, Time interval, uint8_t numSubcarriers)
{
    NS_LOG_FUNCTION(this << rnti << interval.GetMilliSeconds() << (uint32_t)numSubcarriers);
    
    NbIotSpsConfig config;
    config.rnti = rnti;
    config.spsInterval = interval;
    config.numSubcarriers = numSubcarriers;
    config.nextAllocation = Simulator::Now();
    config.active = false;
    
    // Select MCS and TBS based on default CQI
    SelectMcsAndTbs(7, numSubcarriers, config.mcs, config.tbSize);
    
    m_spsConfigs[rnti] = config;
    
    NS_LOG_INFO("SPS configured for UE " << rnti 
                << ", interval=" << interval.GetMilliSeconds() << "ms"
                << ", subcarriers=" << (uint32_t)numSubcarriers
                << ", TBS=" << config.tbSize);
}

void
NbIotSpsScheduler::ActivateSps(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    auto it = m_spsConfigs.find(rnti);
    if (it == m_spsConfigs.end())
    {
        // Auto-configure with defaults if not configured
        ConfigureSps(rnti, m_defaultSpsInterval, 12);
        it = m_spsConfigs.find(rnti);
    }
    
    it->second.active = true;
    it->second.nextAllocation = Simulator::Now();
    
    NS_LOG_INFO("SPS activated for UE " << rnti);
}

void
NbIotSpsScheduler::DeactivateSps(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    auto it = m_spsConfigs.find(rnti);
    if (it != m_spsConfigs.end())
    {
        it->second.active = false;
        NS_LOG_INFO("SPS deactivated for UE " << rnti);
    }
}

bool
NbIotSpsScheduler::IsSpsActive(uint16_t rnti) const
{
    auto it = m_spsConfigs.find(rnti);
    return (it != m_spsConfigs.end() && it->second.active);
}

const NbIotSpsConfig*
NbIotSpsScheduler::GetSpsConfig(uint16_t rnti) const
{
    auto it = m_spsConfigs.find(rnti);
    if (it != m_spsConfigs.end())
    {
        return &(it->second);
    }
    return nullptr;
}

void
NbIotSpsScheduler::SetDefaultSpsInterval(Time interval)
{
    m_defaultSpsInterval = interval;
}

Time
NbIotSpsScheduler::GetDefaultSpsInterval() const
{
    return m_defaultSpsInterval;
}

void
NbIotSpsScheduler::SetAutoActivateSps(bool enable)
{
    m_autoActivateSps = enable;
}

uint32_t
NbIotSpsScheduler::GetActiveSpsCount() const
{
    uint32_t count = 0;
    for (const auto& config : m_spsConfigs)
    {
        if (config.second.active)
        {
            count++;
        }
    }
    return count;
}

void
NbIotSpsScheduler::GetSpsStatistics(uint32_t& paddingTransmissions, 
                                     uint32_t& dataTransmissions) const
{
    paddingTransmissions = m_paddingTransmissions;
    dataTransmissions = m_dataTransmissions;
}

std::vector<NbIotDlAssignmentResult>
NbIotSpsScheduler::ScheduleDl(uint8_t availableRbs)
{
    NS_LOG_FUNCTION(this << (uint32_t)availableRbs);
    
    std::vector<NbIotDlAssignmentResult> assignments;
    
    // For DL, use simple round-robin (SPS primarily for UL)
    if (m_mac == nullptr)
    {
        return assignments;
    }
    
    auto& ueContexts = m_mac->GetUeContexts();
    uint8_t usedRbs = 0;
    
    for (auto& context : ueContexts)
    {
        if (usedRbs >= availableRbs)
        {
            break;
        }
        
        uint16_t rnti = context.first;
        auto& txQueue = m_mac->GetDlTxQueue(rnti);
        
        if (!txQueue.empty())
        {
            NbIotDlAssignmentResult result;
            result.rnti = rnti;
            result.mcs = 4;
            result.numSubframes = 1;
            result.repetitions = 1;
            result.rv = 0;
            result.newData = true;
            result.harqProcess = 0;
            result.tbSize = 88;
            
            // Add pending SDUs
            while (!txQueue.empty() && result.sdus.size() < 8)
            {
                result.sdus.push_back(txQueue.front().second);
                txQueue.pop();
            }
            
            assignments.push_back(result);
            usedRbs++;
        }
    }
    
    return assignments;
}

std::vector<NbIotUlGrantResult>
NbIotSpsScheduler::ScheduleUl(uint8_t availableRbs)
{
    NS_LOG_FUNCTION(this << (uint32_t)availableRbs);
    
    m_subframeCounter++;
    
    std::vector<NbIotUlGrantResult> grants;
    Time currentTime = Simulator::Now();
    
    // First, schedule SPS grants
    auto spsGrants = ScheduleSpsGrants(currentTime, availableRbs);
    grants.insert(grants.end(), spsGrants.begin(), spsGrants.end());
    
    // Calculate remaining RBs after SPS allocation
    uint8_t remainingRbs = availableRbs;
    for (const auto& grant : spsGrants)
    {
        // Each SPS grant uses 1 RB (in NB-IoT, all 12 subcarriers = 1 PRB)
        remainingRbs = (remainingRbs > 0) ? remainingRbs - 1 : 0;
    }
    
    // Then, schedule dynamic grants for non-SPS UEs
    auto dynamicGrants = ScheduleDynamicGrants(remainingRbs);
    grants.insert(grants.end(), dynamicGrants.begin(), dynamicGrants.end());
    
    return grants;
}

std::vector<NbIotUlGrantResult>
NbIotSpsScheduler::ScheduleSpsGrants(Time currentTime, uint8_t availableRbs)
{
    NS_LOG_FUNCTION(this << currentTime.GetMilliSeconds() << (uint32_t)availableRbs);
    
    std::vector<NbIotUlGrantResult> grants;
    uint8_t usedRbs = 0;
    
    for (auto& configPair : m_spsConfigs)
    {
        if (usedRbs >= availableRbs)
        {
            break;
        }
        
        NbIotSpsConfig& config = configPair.second;
        
        if (!config.active)
        {
            continue;
        }
        
        // Check if this is an SPS allocation time for this UE
        if (currentTime >= config.nextAllocation)
        {
            NS_LOG_DEBUG("SPS grant for UE " << config.rnti 
                        << " at time " << currentTime.GetMilliSeconds() << "ms");
            
            NbIotUlGrantResult grant;
            grant.rnti = config.rnti;
            grant.mcs = config.mcs;
            grant.tbSize = config.tbSize;
            grant.numSubcarriers = config.numSubcarriers;
            grant.numSlots = 4;  // Typical NB-IoT slot allocation
            grant.repetitions = config.repetitions;
            grant.rv = 0;
            grant.newData = true;
            grant.harqProcess = config.harqProcess;
            
            grants.push_back(grant);
            usedRbs++;
            
            // Check if UE has data (for statistics)
            auto ueContext = GetUeContext(config.rnti);
            if (ueContext && ueContext->totalBufferSize > 0)
            {
                m_dataTransmissions++;
            }
            else
            {
                // UE will send padding
                m_paddingTransmissions++;
            }
            
            // Update next allocation time
            UpdateNextAllocation(config.rnti);
        }
    }
    
    return grants;
}

std::vector<NbIotUlGrantResult>
NbIotSpsScheduler::ScheduleDynamicGrants(uint8_t availableRbs)
{
    NS_LOG_FUNCTION(this << (uint32_t)availableRbs);
    
    std::vector<NbIotUlGrantResult> grants;
    
    if (m_mac == nullptr || availableRbs == 0)
    {
        return grants;
    }
    
    // Get UEs with pending SR (Scheduling Request) that don't have active SPS
    auto& ueContexts = m_mac->GetUeContexts();
    uint8_t usedRbs = 0;
    
    std::vector<uint16_t> dynamicUes;
    for (const auto& context : ueContexts)
    {
        uint16_t rnti = context.first;
        
        // Skip SPS-active UEs
        if (IsSpsActive(rnti))
        {
            continue;
        }
        
        // Check if UE has pending SR or buffer data
        if (context.second->srReceived || context.second->totalBufferSize > 0)
        {
            dynamicUes.push_back(rnti);
        }
    }
    
    // Round-robin scheduling for dynamic UEs
    for (uint16_t rnti : dynamicUes)
    {
        if (usedRbs >= availableRbs)
        {
            break;
        }
        
        auto ueContext = GetUeContext(rnti);
        if (ueContext == nullptr)
        {
            continue;
        }
        
        uint8_t mcs;
        uint16_t tbSize;
        SelectMcsAndTbs(ueContext->cqi, 12, mcs, tbSize);
        
        NbIotUlGrantResult grant;
        grant.rnti = rnti;
        grant.mcs = mcs;
        grant.tbSize = tbSize;
        grant.numSubcarriers = 12;
        grant.numSlots = 4;
        grant.repetitions = 1;
        grant.rv = 0;
        grant.newData = true;
        grant.harqProcess = 0;
        
        grants.push_back(grant);
        usedRbs++;
        
        // Clear SR flag
        ueContext->srReceived = false;
        
        NS_LOG_DEBUG("Dynamic grant for UE " << rnti);
    }
    
    return grants;
}

void
NbIotSpsScheduler::SelectMcsAndTbs(uint8_t cqi, uint8_t numSubcarriers,
                                    uint8_t& mcs, uint16_t& tbSize)
{
    // Simplified MCS/TBS selection for NB-IoT
    // Based on 3GPP TS 36.213 Table 16.4.1.5.1-1
    
    // Map CQI to MCS (simplified)
    if (cqi <= 3)
    {
        mcs = 0;
        tbSize = 16;
    }
    else if (cqi <= 5)
    {
        mcs = 2;
        tbSize = 56;
    }
    else if (cqi <= 8)
    {
        mcs = 4;
        tbSize = 88;
    }
    else if (cqi <= 11)
    {
        mcs = 6;
        tbSize = 152;
    }
    else
    {
        mcs = 8;
        tbSize = 256;
    }
    
    // Adjust TBS based on subcarrier count
    if (numSubcarriers < 12)
    {
        tbSize = tbSize * numSubcarriers / 12;
        if (tbSize < 16)
        {
            tbSize = 16;
        }
    }
}

void
NbIotSpsScheduler::UpdateNextAllocation(uint16_t rnti)
{
    auto it = m_spsConfigs.find(rnti);
    if (it != m_spsConfigs.end())
    {
        it->second.nextAllocation = Simulator::Now() + it->second.spsInterval;
        
        NS_LOG_DEBUG("Next SPS allocation for UE " << rnti 
                    << " at " << it->second.nextAllocation.GetMilliSeconds() << "ms");
    }
}

} // namespace ns3
