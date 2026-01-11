/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT eNB MAC Layer implementation
 */

#include "nbiot-enb-mac.h"
#include "nbiot-enb-phy.h"
#include "nbiot-rrc.h"
#include "nbiot-control-messages.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>
#include <ns3/boolean.h>
#include <ns3/pointer.h>

#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotEnbMac");

// ====================== NbIotScheduler ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotScheduler);

TypeId
NbIotScheduler::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotScheduler")
        .SetParent<Object>()
        .SetGroupName("NbIot");
    return tid;
}

NbIotScheduler::NbIotScheduler()
{
    NS_LOG_FUNCTION(this);
}

NbIotScheduler::~NbIotScheduler()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotScheduler::DoDispose()
{
    m_mac = nullptr;
    m_ueContexts.clear();
    Object::DoDispose();
}

void
NbIotScheduler::SetMac(Ptr<NbIotEnbMac> mac)
{
    m_mac = mac;
}

Ptr<NbIotUeContext>
NbIotScheduler::GetUeContext(uint16_t rnti)
{
    if (m_mac)
    {
        return m_mac->GetUeContext(rnti);
    }
    return nullptr;
}

// ====================== NbIotRoundRobinScheduler ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotRoundRobinScheduler);

TypeId
NbIotRoundRobinScheduler::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotRoundRobinScheduler")
        .SetParent<NbIotScheduler>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotRoundRobinScheduler>();
    return tid;
}

NbIotRoundRobinScheduler::NbIotRoundRobinScheduler()
    : m_lastDlRnti(0)
    , m_lastUlRnti(0)
{
    NS_LOG_FUNCTION(this);
}

NbIotRoundRobinScheduler::~NbIotRoundRobinScheduler()
{
    NS_LOG_FUNCTION(this);
}

std::vector<NbIotDlAssignmentResult>
NbIotRoundRobinScheduler::ScheduleDl(uint8_t availableRbs)
{
    NS_LOG_FUNCTION(this << static_cast<int>(availableRbs));
    
    std::vector<NbIotDlAssignmentResult> assignments;
    
    if (!m_mac || availableRbs == 0)
    {
        return assignments;
    }
    
    auto& ueContexts = m_mac->GetUeContexts();
    if (ueContexts.empty())
    {
        return assignments;
    }
    
    // Find next UE with pending data
    uint16_t rnti = GetNextRnti(m_lastDlRnti, true);
    if (rnti == 0)
    {
        return assignments;
    }
    
    // Check if UE has pending DL data
    auto& dlQueue = m_mac->GetDlTxQueue(rnti);
    if (dlQueue.empty())
    {
        return assignments;
    }
    
    auto ueContext = GetUeContext(rnti);
    if (!ueContext)
    {
        return assignments;
    }
    
    // Determine MCS based on CQI
    uint8_t mcs = std::min(static_cast<uint8_t>(ueContext->cqi), static_cast<uint8_t>(10));
    
    // Get repetitions based on coverage class
    uint8_t reps = NPDSCH_REPETITIONS[std::min(static_cast<size_t>(ueContext->coverageClass) * 2,
                                                 NPDSCH_REPETITIONS.size() - 1)];
    
    // Get TBS
    uint16_t tbSize = GetTbsFromMcsAndRbs(mcs, 1);
    
    NbIotDlAssignmentResult assignment;
    assignment.rnti = rnti;
    assignment.mcs = mcs;
    assignment.tbSize = tbSize;
    assignment.numSubframes = 1;
    assignment.repetitions = reps;
    assignment.rv = 0;
    assignment.newData = true;
    assignment.harqProcess = 0;
    
    // Collect SDUs for this TB
    uint16_t remaining = tbSize;
    while (!dlQueue.empty() && remaining > 3)
    {
        auto& front = dlQueue.front();
        uint16_t sduSize = front.second->GetSize();
        if (sduSize + 2 <= remaining)
        {
            assignment.sdus.push_back(front.second);
            remaining -= (sduSize + 2);
            dlQueue.pop();
        }
        else
        {
            break;
        }
    }
    
    if (!assignment.sdus.empty())
    {
        assignments.push_back(assignment);
        m_lastDlRnti = rnti;
    }
    
    return assignments;
}

std::vector<NbIotUlGrantResult>
NbIotRoundRobinScheduler::ScheduleUl(uint8_t availableRbs)
{
    NS_LOG_FUNCTION(this << static_cast<int>(availableRbs));
    
    std::vector<NbIotUlGrantResult> grants;
    
    if (!m_mac || availableRbs == 0)
    {
        return grants;
    }
    
    auto& ueContexts = m_mac->GetUeContexts();
    if (ueContexts.empty())
    {
        return grants;
    }
    
    // Find next UE with pending UL data or SR
    for (auto& [rnti, context] : ueContexts)
    {
        if (!context->connected)
        {
            continue;
        }
        
        if (context->srReceived || context->totalBufferSize > 0)
        {
            // Determine MCS based on coverage class and CQI
            uint8_t mcs = std::min(static_cast<uint8_t>(context->cqi / 2), static_cast<uint8_t>(7));
            
            // Get repetitions
            uint8_t reps = NPUSCH_FORMAT1_REPETITIONS[std::min(static_cast<size_t>(context->coverageClass),
                                                               NPUSCH_FORMAT1_REPETITIONS.size() - 1)];
            
            uint16_t tbSize = GetTbsFromMcsAndRbs(mcs, 1);
            
            NbIotUlGrantResult grant;
            grant.rnti = rnti;
            grant.mcs = mcs;
            grant.tbSize = tbSize;
            grant.numSubcarriers = 12;
            grant.numSlots = 2;
            grant.repetitions = reps;
            grant.rv = 0;
            grant.newData = true;
            grant.harqProcess = 0;
            
            grants.push_back(grant);
            
            context->srReceived = false;
            m_lastUlRnti = rnti;
            
            // NB-IoT only schedules one UE per subframe
            break;
        }
    }
    
    return grants;
}

uint16_t
NbIotRoundRobinScheduler::GetNextRnti(uint16_t lastRnti, bool dl)
{
    if (!m_mac)
    {
        return 0;
    }
    
    auto& ueContexts = m_mac->GetUeContexts();
    if (ueContexts.empty())
    {
        return 0;
    }
    
    // Find starting point
    auto it = ueContexts.upper_bound(lastRnti);
    if (it == ueContexts.end())
    {
        it = ueContexts.begin();
    }
    
    // Find next UE with data
    auto startIt = it;
    do
    {
        if (it->second->connected)
        {
            if (dl && !m_mac->GetDlTxQueue(it->first).empty())
            {
                return it->first;
            }
            else if (!dl && (it->second->srReceived || it->second->totalBufferSize > 0))
            {
                return it->first;
            }
        }
        
        ++it;
        if (it == ueContexts.end())
        {
            it = ueContexts.begin();
        }
    } while (it != startIt);
    
    return 0;
}

// ====================== NbIotCoverageClassScheduler ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotCoverageClassScheduler);

TypeId
NbIotCoverageClassScheduler::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotCoverageClassScheduler")
        .SetParent<NbIotScheduler>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotCoverageClassScheduler>();
    return tid;
}

NbIotCoverageClassScheduler::NbIotCoverageClassScheduler()
{
    NS_LOG_FUNCTION(this);
}

NbIotCoverageClassScheduler::~NbIotCoverageClassScheduler()
{
    NS_LOG_FUNCTION(this);
}

std::vector<NbIotDlAssignmentResult>
NbIotCoverageClassScheduler::ScheduleDl(uint8_t availableRbs)
{
    NS_LOG_FUNCTION(this << static_cast<int>(availableRbs));
    
    std::vector<NbIotDlAssignmentResult> assignments;
    
    if (!m_mac || availableRbs == 0)
    {
        return assignments;
    }
    
    auto& ueContexts = m_mac->GetUeContexts();
    
    // Get UEs with pending DL data
    std::vector<uint16_t> candidateRntis;
    for (auto& [rnti, context] : ueContexts)
    {
        if (context->connected && !m_mac->GetDlTxQueue(rnti).empty())
        {
            candidateRntis.push_back(rnti);
        }
    }
    
    if (candidateRntis.empty())
    {
        return assignments;
    }
    
    // Sort by priority (coverage class based)
    auto sortedRntis = SortByPriority(candidateRntis);
    
    // Schedule highest priority UE
    uint16_t rnti = sortedRntis.front();
    auto ueContext = GetUeContext(rnti);
    auto& dlQueue = m_mac->GetDlTxQueue(rnti);
    
    uint8_t mcs = std::min(static_cast<uint8_t>(ueContext->cqi), static_cast<uint8_t>(10));
    uint8_t reps = NPDSCH_REPETITIONS[std::min(static_cast<size_t>(ueContext->coverageClass) * 3,
                                                 NPDSCH_REPETITIONS.size() - 1)];
    uint16_t tbSize = GetTbsFromMcsAndRbs(mcs, 1);
    
    NbIotDlAssignmentResult assignment;
    assignment.rnti = rnti;
    assignment.mcs = mcs;
    assignment.tbSize = tbSize;
    assignment.numSubframes = 1;
    assignment.repetitions = reps;
    assignment.rv = 0;
    assignment.newData = true;
    assignment.harqProcess = 0;
    
    uint16_t remaining = tbSize;
    while (!dlQueue.empty() && remaining > 3)
    {
        auto& front = dlQueue.front();
        uint16_t sduSize = front.second->GetSize();
        if (sduSize + 2 <= remaining)
        {
            assignment.sdus.push_back(front.second);
            remaining -= (sduSize + 2);
            dlQueue.pop();
        }
        else
        {
            break;
        }
    }
    
    if (!assignment.sdus.empty())
    {
        assignments.push_back(assignment);
    }
    
    return assignments;
}

std::vector<NbIotUlGrantResult>
NbIotCoverageClassScheduler::ScheduleUl(uint8_t availableRbs)
{
    NS_LOG_FUNCTION(this << static_cast<int>(availableRbs));
    
    std::vector<NbIotUlGrantResult> grants;
    
    if (!m_mac || availableRbs == 0)
    {
        return grants;
    }
    
    auto& ueContexts = m_mac->GetUeContexts();
    
    // Get UEs with pending UL data
    std::vector<uint16_t> candidateRntis;
    for (auto& [rnti, context] : ueContexts)
    {
        if (context->connected && (context->srReceived || context->totalBufferSize > 0))
        {
            candidateRntis.push_back(rnti);
        }
    }
    
    if (candidateRntis.empty())
    {
        return grants;
    }
    
    // Sort by priority
    auto sortedRntis = SortByPriority(candidateRntis);
    
    // Grant to highest priority UE
    uint16_t rnti = sortedRntis.front();
    auto ueContext = GetUeContext(rnti);
    
    uint8_t mcs = std::min(static_cast<uint8_t>(ueContext->cqi / 2), static_cast<uint8_t>(7));
    uint8_t reps = NPUSCH_FORMAT1_REPETITIONS[std::min(static_cast<size_t>(ueContext->coverageClass) * 2,
                                                        NPUSCH_FORMAT1_REPETITIONS.size() - 1)];
    uint16_t tbSize = GetTbsFromMcsAndRbs(mcs, 1);
    
    NbIotUlGrantResult grant;
    grant.rnti = rnti;
    grant.mcs = mcs;
    grant.tbSize = tbSize;
    grant.numSubcarriers = 12;
    grant.numSlots = 2;
    grant.repetitions = reps;
    grant.rv = 0;
    grant.newData = true;
    grant.harqProcess = 0;
    
    grants.push_back(grant);
    ueContext->srReceived = false;
    
    return grants;
}

uint8_t
NbIotCoverageClassScheduler::GetPriority(NbIotCoverageClass ceLevel)
{
    // Higher coverage class gets higher priority (more resources needed)
    // to avoid starvation, but also consider fairness
    switch (ceLevel)
    {
        case NbIotCoverageClass::CE_LEVEL_0:
            return 3;
        case NbIotCoverageClass::CE_LEVEL_1:
            return 2;
        case NbIotCoverageClass::CE_LEVEL_2:
            return 1;
        default:
            return 4;
    }
}

std::vector<uint16_t>
NbIotCoverageClassScheduler::SortByPriority(const std::vector<uint16_t>& ueList)
{
    std::vector<std::pair<uint16_t, uint8_t>> rntiPriority;
    
    for (uint16_t rnti : ueList)
    {
        auto context = GetUeContext(rnti);
        if (context)
        {
            rntiPriority.emplace_back(rnti, GetPriority(context->coverageClass));
        }
    }
    
    std::sort(rntiPriority.begin(), rntiPriority.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });
    
    std::vector<uint16_t> sorted;
    for (const auto& [rnti, pri] : rntiPriority)
    {
        sorted.push_back(rnti);
    }
    
    return sorted;
}

// ====================== NbIotEnbMac ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotEnbMac);

TypeId
NbIotEnbMac::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotEnbMac")
        .SetParent<NbIotMac>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotEnbMac>()
        .AddAttribute("RarDelay",
                      "Delay before sending RAR after preamble reception",
                      TimeValue(MilliSeconds(3)),
                      MakeTimeAccessor(&NbIotEnbMac::m_rarDelay),
                      MakeTimeChecker())
        .AddTraceSource("UlSchedule",
                        "UL scheduling decision",
                        MakeTraceSourceAccessor(&NbIotEnbMac::m_ulScheduleTrace),
                        "ns3::NbIotEnbMac::UlScheduleTracedCallback")
        .AddTraceSource("DlSchedule",
                        "DL scheduling decision",
                        MakeTraceSourceAccessor(&NbIotEnbMac::m_dlScheduleTrace),
                        "ns3::NbIotEnbMac::DlScheduleTracedCallback")
        .AddTraceSource("Rar",
                        "RAR generation",
                        MakeTraceSourceAccessor(&NbIotEnbMac::m_rarTrace),
                        "ns3::NbIotEnbMac::RarTracedCallback");
    return tid;
}

NbIotEnbMac::NbIotEnbMac()
    : m_nextRnti(1)
{
    NS_LOG_FUNCTION(this);
    
    // Create default round-robin scheduler
    m_scheduler = CreateObject<NbIotRoundRobinScheduler>();
    m_scheduler->SetMac(this);
}

NbIotEnbMac::~NbIotEnbMac()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotEnbMac::DoDispose()
{
    NS_LOG_FUNCTION(this);
    
    m_phy = nullptr;
    m_rrc = nullptr;
    m_scheduler = nullptr;
    
    m_ueContexts.clear();
    m_dlTxQueues.clear();
    m_dlHarqManagers.clear();
    m_ulHarqManagers.clear();
    
    NbIotMac::DoDispose();
}

void
NbIotEnbMac::SetPhy(Ptr<NbIotEnbPhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    m_phy = phy;
}

Ptr<NbIotEnbPhy>
NbIotEnbMac::GetPhy() const
{
    return m_phy;
}

void
NbIotEnbMac::SetRrc(Ptr<NbIotEnbRrc> rrc)
{
    NS_LOG_FUNCTION(this << rrc);
    m_rrc = rrc;
}

void
NbIotEnbMac::SetScheduler(Ptr<NbIotScheduler> scheduler)
{
    NS_LOG_FUNCTION(this << scheduler);
    m_scheduler = scheduler;
    m_scheduler->SetMac(this);
}

Ptr<NbIotScheduler>
NbIotEnbMac::GetScheduler() const
{
    return m_scheduler;
}

void
NbIotEnbMac::ReceivePdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    // eNB receives PDUs with RNTI context from PHY, so this generic version
    // logs a warning and does nothing. Use ReceivePdu(rnti, packet) instead.
    NS_LOG_WARN("ReceivePdu called without RNTI on eNB MAC - use ReceivePdu(rnti, packet)");
}

void
NbIotEnbMac::ReceivePdu(uint16_t rnti, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << rnti << packet);
    
    m_macRxTrace(rnti, packet->GetSize());
    
    auto context = GetUeContext(rnti);
    if (context)
    {
        context->lastActivity = Simulator::Now();
    }
    
    // Parse MAC PDU
    auto sdus = NbIotMacPdu::Deserialize(packet);
    
    for (const auto& [lcid, packets] : sdus)
    {
        if (lcid == NbIotMacSubheader::SHORT_BSR || lcid == NbIotMacSubheader::LONG_BSR)
        {
            // Process BSR
            if (!packets.empty())
            {
                // Extract BSR values (simplified)
                auto bsrType = (lcid == NbIotMacSubheader::SHORT_BSR) 
                               ? NbIotBsrMessage::SHORT_BSR 
                               : NbIotBsrMessage::LONG_BSR;
                auto bsrMsg = Create<NbIotBsrMessage>(bsrType);
                // Parse BSR data from packet
                ProcessBsr(rnti, bsrMsg);
            }
        }
        else if (lcid == NbIotMacSubheader::PHR)
        {
            // Process PHR
            NS_LOG_DEBUG("Received PHR from UE " << rnti);
        }
        else
        {
            // Regular SDU - forward to RLC
            for (const auto& sdu : packets)
            {
                NS_LOG_DEBUG("eNB received SDU on LCID " << static_cast<int>(lcid)
                             << " from UE " << rnti << ", size " << sdu->GetSize());
            }
        }
    }
}

void
NbIotEnbMac::TransmitSdu(uint16_t rnti, uint8_t lcid, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<int>(lcid) << packet);
    
    m_dlTxQueues[rnti].push(std::make_pair(lcid, packet));
    
    // Update UE buffer status
    auto context = GetUeContext(rnti);
    if (context)
    {
        context->totalBufferSize += packet->GetSize();
    }
}

void
NbIotEnbMac::ProcessReceivedPreamble(uint8_t preambleId, uint16_t timingAdvance,
                                      NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << static_cast<int>(preambleId) << timingAdvance
                    << CoverageClassToString(coverageClass));
    
    NS_LOG_INFO("eNB received preamble " << static_cast<int>(preambleId)
                << " with TA=" << timingAdvance
                << ", coverage class " << CoverageClassToString(coverageClass));
    
    // Schedule RAR generation
    Simulator::Schedule(m_rarDelay,
                        &NbIotEnbMac::GenerateRar,
                        this,
                        preambleId,
                        timingAdvance,
                        coverageClass);
}

void
NbIotEnbMac::ProcessHarqFeedback(uint16_t rnti, uint8_t harqId, bool ack)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<int>(harqId) << ack);
    
    auto it = m_dlHarqManagers.find(rnti);
    if (it != m_dlHarqManagers.end())
    {
        if (ack)
        {
            it->second->ProcessAck(harqId);
        }
        else
        {
            it->second->ProcessNack(harqId);
        }
    }
}

void
NbIotEnbMac::ProcessBsr(uint16_t rnti, Ptr<NbIotBsrMessage> bsr)
{
    NS_LOG_FUNCTION(this << rnti << bsr);
    
    auto context = GetUeContext(rnti);
    if (context)
    {
        // Update buffer status from BSR
        // In full implementation, would decode BSR indices to bytes
        context->totalBufferSize = 100; // Simplified
        context->srReceived = true;
        
        NS_LOG_INFO("eNB processed BSR from UE " << rnti);
    }
}

void
NbIotEnbMac::ProcessSchedulingRequest(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    auto context = GetUeContext(rnti);
    if (context)
    {
        context->srReceived = true;
        NS_LOG_INFO("eNB received SR from UE " << rnti);
    }
}

uint16_t
NbIotEnbMac::AllocateRnti()
{
    NS_LOG_FUNCTION(this);
    
    uint16_t rnti = m_nextRnti++;
    
    // Skip reserved values
    if (m_nextRnti >= 0xFFF0)
    {
        m_nextRnti = 1;
    }
    
    return rnti;
}

void
NbIotEnbMac::AddUe(uint16_t rnti, NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << rnti << CoverageClassToString(coverageClass));
    
    auto context = Create<NbIotUeContext>();
    context->rnti = rnti;
    context->coverageClass = coverageClass;
    context->connected = true;
    
    m_ueContexts[rnti] = context;
    
    // Create HARQ managers
    m_dlHarqManagers[rnti] = CreateObject<NbIotHarqManager>();
    m_ulHarqManagers[rnti] = CreateObject<NbIotHarqManager>();
    
    NS_LOG_INFO("eNB added UE " << rnti << " with coverage class "
                << CoverageClassToString(coverageClass));
}

void
NbIotEnbMac::RemoveUe(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    m_ueContexts.erase(rnti);
    m_dlTxQueues.erase(rnti);
    m_dlHarqManagers.erase(rnti);
    m_ulHarqManagers.erase(rnti);
    
    NS_LOG_INFO("eNB removed UE " << rnti);
}

Ptr<NbIotUeContext>
NbIotEnbMac::GetUeContext(uint16_t rnti)
{
    auto it = m_ueContexts.find(rnti);
    if (it != m_ueContexts.end())
    {
        return it->second;
    }
    return nullptr;
}

std::map<uint16_t, Ptr<NbIotUeContext>>&
NbIotEnbMac::GetUeContexts()
{
    return m_ueContexts;
}

void
NbIotEnbMac::SubframeIndication()
{
    NS_LOG_FUNCTION(this);
    
    DoSchedule();
}

std::queue<std::pair<uint8_t, Ptr<Packet>>>&
NbIotEnbMac::GetDlTxQueue(uint16_t rnti)
{
    return m_dlTxQueues[rnti];
}

void
NbIotEnbMac::GenerateRar(uint8_t preambleId, uint16_t timingAdvance,
                          NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << static_cast<int>(preambleId) << timingAdvance);
    
    // Allocate temp C-RNTI
    uint16_t tempCRnti = AllocateRnti();
    
    // Create RAR message
    auto rar = Create<NbIotRarMessage>();
    rar->SetPreambleId(preambleId);
    
    NbIotRarMessage::RarGrant grant;
    grant.timingAdvance = timingAdvance;
    grant.tempCRnti = tempCRnti;
    grant.resourceAssignment = 0;  // Simplified
    grant.subcarrierSpacing = 0;
    grant.mcs = 0;
    
    rar->SetRarGrant(grant);
    
    m_rarTrace(preambleId, tempCRnti);
    
    // Add UE context
    AddUe(tempCRnti, coverageClass);
    
    NS_LOG_INFO("eNB generated RAR for preamble " << static_cast<int>(preambleId)
                << ", allocated C-RNTI " << tempCRnti);
    
    // Transmit RAR via NPDSCH
    if (m_phy)
    {
        uint8_t reps = NPDSCH_REPETITIONS[std::min(static_cast<size_t>(coverageClass) * 2,
                                                    NPDSCH_REPETITIONS.size() - 1)];
        
        // Serialize RAR to packet (simplified - 10 bytes for RAR)
        Ptr<Packet> rarPacket = Create<Packet>(10);
        
        m_phy->TransmitNpdsch(rarPacket, 1, 16, reps); // RA-RNTI = 1, TBS = 16
    }
}

void
NbIotEnbMac::DoSchedule()
{
    NS_LOG_FUNCTION(this);
    
    if (!m_scheduler)
    {
        return;
    }
    
    // NB-IoT uses 1 PRB (180 kHz)
    const uint8_t availableRbs = 1;
    
    // Schedule DL
    auto dlAssignments = m_scheduler->ScheduleDl(availableRbs);
    if (!dlAssignments.empty())
    {
        TransmitDlPdus(dlAssignments);
    }
    
    // Schedule UL
    auto ulGrants = m_scheduler->ScheduleUl(availableRbs);
    if (!ulGrants.empty())
    {
        SendUlGrants(ulGrants);
    }
}

void
NbIotEnbMac::TransmitDlPdus(const std::vector<NbIotDlAssignmentResult>& assignments)
{
    NS_LOG_FUNCTION(this);
    
    for (const auto& assignment : assignments)
    {
        // Build MAC PDU
        auto pdu = CreateObject<NbIotMacPdu>();
        
        for (const auto& sdu : assignment.sdus)
        {
            // Assuming LCID 1 for simplicity - in real implementation track LCID
            pdu->AddSdu(1, sdu);
        }
        
        pdu->AddPadding(assignment.tbSize);
        
        Ptr<Packet> macPdu = pdu->Serialize();
        
        m_dlScheduleTrace(assignment.rnti, assignment.mcs, assignment.tbSize);
        m_macTxTrace(assignment.rnti, macPdu->GetSize());
        
        NS_LOG_INFO("eNB transmitting DL PDU to UE " << assignment.rnti
                    << ", MCS=" << static_cast<int>(assignment.mcs)
                    << ", TBS=" << assignment.tbSize);
        
        // Store in HARQ buffer
        auto it = m_dlHarqManagers.find(assignment.rnti);
        if (it != m_dlHarqManagers.end())
        {
            auto* process = it->second->GetProcess(assignment.harqProcess);
            if (process)
            {
                process->SetPacket(macPdu->Copy());
                process->SetTbs(assignment.tbSize);
                process->SetActive(true);
            }
        }
        
        // Transmit via PHY
        if (m_phy)
        {
            m_phy->TransmitNpdsch(macPdu, assignment.rnti, assignment.mcs,
                                   assignment.repetitions);
        }
        
        // Send DL assignment via NPDCCH
        NbIotDci dci;
        dci.format = NbIotDciFormat::DCI_N1;
        dci.rnti = assignment.rnti;
        dci.mcs = assignment.mcs;
        dci.repetitionNumber = assignment.repetitions;
        dci.ndi = assignment.newData ? 1 : 0;
        dci.harqProcessNum = assignment.harqProcess;
        dci.resourceAssignment = 0;
        
        if (m_phy)
        {
            m_phy->TransmitNpdcch(dci, 1, 1); // repetitions=1, aggregationLevel=1
        }
    }
}

void
NbIotEnbMac::SendUlGrants(const std::vector<NbIotUlGrantResult>& grants)
{
    NS_LOG_FUNCTION(this);
    
    for (const auto& grant : grants)
    {
        m_ulScheduleTrace(grant.rnti, grant.mcs, grant.tbSize);
        
        NS_LOG_INFO("eNB sending UL grant to UE " << grant.rnti
                    << ", MCS=" << static_cast<int>(grant.mcs)
                    << ", TBS=" << grant.tbSize);
        
        // Build DCI N0 for UL grant
        NbIotDci dci;
        dci.format = NbIotDciFormat::DCI_N0;
        dci.rnti = grant.rnti;
        dci.mcs = grant.mcs;
        dci.repetitionNumber = grant.repetitions;
        dci.ndi = grant.newData ? 1 : 0;
        dci.harqProcessNum = grant.harqProcess;
        dci.resourceAssignment = 0;
        dci.subcarrierIndication = 0;
        dci.schedulingDelay = 4;
        
        if (m_phy)
        {
            m_phy->TransmitNpdcch(dci, 1, 1); // repetitions=1, aggregationLevel=1
        }
    }
}

} // namespace ns3
