/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT UE MAC Layer implementation
 */

#include "nbiot-ue-mac.h"
#include "nbiot-ue-phy.h"
#include "nbiot-control-messages.h"
#include "nbiot-rrc.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>
#include <ns3/boolean.h>
#include <ns3/pointer.h>

#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotUeMac");

// ====================== NbIotUeRandomAccessManager ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotUeRandomAccessManager);

TypeId
NbIotUeRandomAccessManager::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotUeRandomAccessManager")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotUeRandomAccessManager>();
    return tid;
}

NbIotUeRandomAccessManager::NbIotUeRandomAccessManager()
    : m_state(NbIotRaState::IDLE)
    , m_preambleIndex(0)
    , m_preambleTxCount(0)
    , m_timingAdvance(0)
    , m_tempCRnti(0)
    , m_coverageClass(NbIotCoverageClass::CE_LEVEL_0)
{
    NS_LOG_FUNCTION(this);
    
    // Default RACH configuration
    m_config.preambleRepetitions = 1;
    m_config.preambleFormat = NbIotNprachFormat::FORMAT_0;
    m_config.rarWindow = 10;
    m_config.maxPreambleTx = 10;
    m_config.backoffIndicator = 0;
}

NbIotUeRandomAccessManager::~NbIotUeRandomAccessManager()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotUeRandomAccessManager::DoDispose()
{
    NS_LOG_FUNCTION(this);
    
    m_mac = nullptr;
    
    if (m_rarWindowEvent.IsPending())
    {
        Simulator::Cancel(m_rarWindowEvent);
    }
    if (m_backoffEvent.IsPending())
    {
        Simulator::Cancel(m_backoffEvent);
    }
    if (m_preambleTxEvent.IsPending())
    {
        Simulator::Cancel(m_preambleTxEvent);
    }
    
    Object::DoDispose();
}

void
NbIotUeRandomAccessManager::SetMac(Ptr<NbIotUeMac> mac)
{
    NS_LOG_FUNCTION(this << mac);
    m_mac = mac;
}

void
NbIotUeRandomAccessManager::StartRach(NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << static_cast<int>(coverageClass));
    
    m_state = NbIotRaState::PREAMBLE_TRANSMISSION;
    m_coverageClass = coverageClass;
    m_preambleTxCount = 0;
    
    // Select random preamble index (0-47 for NB-IoT)
    m_preambleIndex = std::rand() % 48;
    
    // Determine repetitions based on coverage class
    uint8_t idx = static_cast<uint8_t>(std::min(static_cast<size_t>(coverageClass) * 2,
                                                NPRACH_REPETITIONS.size() - 1));
    uint8_t reps = NPRACH_REPETITIONS[idx];
    m_config.preambleRepetitions = reps;
    
    NS_LOG_INFO("UE starting RACH with preamble " << static_cast<int>(m_preambleIndex)
                << ", coverage class " << CoverageClassToString(coverageClass)
                << ", repetitions " << static_cast<int>(reps));
    
    TransmitPreamble();
}

void
NbIotUeRandomAccessManager::CancelRach()
{
    NS_LOG_FUNCTION(this);
    
    m_state = NbIotRaState::IDLE;
    
    if (m_rarWindowEvent.IsPending())
    {
        Simulator::Cancel(m_rarWindowEvent);
    }
    if (m_backoffEvent.IsPending())
    {
        Simulator::Cancel(m_backoffEvent);
    }
    if (m_preambleTxEvent.IsPending())
    {
        Simulator::Cancel(m_preambleTxEvent);
    }
}

void
NbIotUeRandomAccessManager::TransmitPreamble()
{
    NS_LOG_FUNCTION(this);
    
    ++m_preambleTxCount;
    
    if (m_preambleTxCount > m_config.maxPreambleTx)
    {
        NS_LOG_WARN("RACH failed: max preamble transmissions reached");
        m_state = NbIotRaState::IDLE;
        
        if (!m_rachCompleteCallback.IsNull())
        {
            m_rachCompleteCallback(false, 0);
        }
        return;
    }
    
    // Transmit preamble via PHY
    if (m_mac && m_mac->GetPhy())
    {
        m_mac->GetPhy()->TransmitNprach(m_config.preambleFormat,
                                         m_config.preambleRepetitions,
                                         m_coverageClass);
    }
    
    // Start RAR window timer after preamble transmission
    // RAR window starts after preamble + some processing time
    Time preambleDuration = MilliSeconds(m_config.preambleRepetitions * 5.6);
    Time rarWindowStart = preambleDuration + MilliSeconds(4);
    Time rarWindowDuration = MilliSeconds(m_config.rarWindow);
    
    m_state = NbIotRaState::WAIT_RAR;
    
    m_rarWindowEvent = Simulator::Schedule(rarWindowStart + rarWindowDuration,
                                            &NbIotUeRandomAccessManager::RarWindowExpired,
                                            this);
    
    NS_LOG_INFO("UE transmitted NPRACH preamble " << static_cast<int>(m_preambleIndex)
                << " (attempt " << static_cast<int>(m_preambleTxCount) << ")");
}

bool
NbIotUeRandomAccessManager::ProcessRar(Ptr<NbIotRarMessage> rar)
{
    NS_LOG_FUNCTION(this << rar);
    
    if (m_state != NbIotRaState::WAIT_RAR)
    {
        return false;
    }
    
    if (rar->GetPreambleId() != m_preambleIndex)
    {
        NS_LOG_DEBUG("RAR preamble ID mismatch: expected " << static_cast<int>(m_preambleIndex)
                     << ", got " << static_cast<int>(rar->GetPreambleId()));
        return false;
    }
    
    // Cancel RAR window timer
    if (m_rarWindowEvent.IsPending())
    {
        Simulator::Cancel(m_rarWindowEvent);
    }
    
    // Extract RAR grant
    auto grant = rar->GetRarGrant();
    m_timingAdvance = grant.timingAdvance;
    m_tempCRnti = grant.tempCRnti;
    
    NS_LOG_INFO("UE received RAR: TA=" << m_timingAdvance
                << ", temp C-RNTI=" << m_tempCRnti);
    
    // Move to contention resolution
    m_state = NbIotRaState::CONTENTION_RESOLUTION;
    
    // Schedule Msg3 transmission (simplified)
    // In real implementation, would use the grant to schedule NPUSCH
    
    return true;
}

void
NbIotUeRandomAccessManager::ProcessContentionResolution(bool success)
{
    NS_LOG_FUNCTION(this << success);
    
    if (success)
    {
        m_state = NbIotRaState::CONNECTED;
        NS_LOG_INFO("UE RACH completed successfully, C-RNTI=" << m_tempCRnti);
        
        if (!m_rachCompleteCallback.IsNull())
        {
            m_rachCompleteCallback(true, m_tempCRnti);
        }
    }
    else
    {
        NS_LOG_WARN("Contention resolution failed, retrying");
        ApplyBackoff();
    }
}

void
NbIotUeRandomAccessManager::RarWindowExpired()
{
    NS_LOG_FUNCTION(this);
    
    NS_LOG_WARN("RAR window expired without receiving RAR");
    ApplyBackoff();
}

void
NbIotUeRandomAccessManager::ApplyBackoff()
{
    NS_LOG_FUNCTION(this);
    
    m_state = NbIotRaState::PREAMBLE_TRANSMISSION;
    
    // Apply random backoff
    uint32_t backoffMs = (std::rand() % (1 << m_config.backoffIndicator)) * 10;
    
    NS_LOG_INFO("Applying backoff of " << backoffMs << " ms before next preamble");
    
    m_backoffEvent = Simulator::Schedule(MilliSeconds(backoffMs),
                                          &NbIotUeRandomAccessManager::TransmitPreamble,
                                          this);
}

void
NbIotUeRandomAccessManager::SetRachConfig(const NbIotRachConfig& config)
{
    m_config = config;
}

void
NbIotUeRandomAccessManager::SetRachCompleteCallback(RachCompleteCallback callback)
{
    m_rachCompleteCallback = callback;
}

// ====================== NbIotBsrManager ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotBsrManager);

TypeId
NbIotBsrManager::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotBsrManager")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotBsrManager>();
    return tid;
}

NbIotBsrManager::NbIotBsrManager()
    : m_bsrTriggered(false)
    , m_regularBsrTriggered(false)
    , m_periodicBsrTriggered(false)
    , m_periodicBsrTimer(Seconds(0))
{
    m_bufferSizes.fill(0);
}

NbIotBsrManager::~NbIotBsrManager()
{
}

void
NbIotBsrManager::UpdateBufferStatus(uint8_t lcg, uint32_t bufferSize)
{
    NS_LOG_FUNCTION(this << static_cast<int>(lcg) << bufferSize);
    
    if (lcg >= 4)
    {
        return;
    }
    
    bool wasEmpty = (m_bufferSizes[lcg] == 0);
    m_bufferSizes[lcg] = bufferSize;
    
    // Trigger BSR if data arrived when buffer was empty
    if (wasEmpty && bufferSize > 0)
    {
        m_regularBsrTriggered = true;
        m_bsrTriggered = true;
        NS_LOG_INFO("BSR triggered: data arrival in LCG " << static_cast<int>(lcg));
    }
}

bool
NbIotBsrManager::IsBsrTriggered() const
{
    return m_bsrTriggered;
}

std::array<uint8_t, 4>
NbIotBsrManager::GetBsrData(bool& shortBsr) const
{
    std::array<uint8_t, 4> bsrIndices;
    uint8_t nonEmptyLcgs = 0;
    uint8_t lastNonEmptyLcg = 0;
    
    for (uint8_t lcg = 0; lcg < 4; ++lcg)
    {
        bsrIndices[lcg] = ConvertBytesToBsrIndex(m_bufferSizes[lcg]);
        if (m_bufferSizes[lcg] > 0)
        {
            ++nonEmptyLcgs;
            lastNonEmptyLcg = lcg;
        }
    }
    
    // Use short BSR if only one LCG has data
    shortBsr = (nonEmptyLcgs <= 1);
    
    return bsrIndices;
}

void
NbIotBsrManager::ClearBsrTrigger()
{
    m_bsrTriggered = false;
    m_regularBsrTriggered = false;
    m_periodicBsrTriggered = false;
}

void
NbIotBsrManager::SetPeriodicBsrTimer(Time period)
{
    m_periodicBsrTimer = period;
    
    if (period > Seconds(0))
    {
        m_periodicBsrEvent = Simulator::Schedule(period,
                                                  &NbIotBsrManager::PeriodicBsrTimeout,
                                                  this);
    }
}

void
NbIotBsrManager::PeriodicBsrTimeout()
{
    NS_LOG_FUNCTION(this);
    
    m_periodicBsrTriggered = true;
    m_bsrTriggered = true;
    
    // Reschedule
    if (m_periodicBsrTimer > Seconds(0))
    {
        m_periodicBsrEvent = Simulator::Schedule(m_periodicBsrTimer,
                                                  &NbIotBsrManager::PeriodicBsrTimeout,
                                                  this);
    }
}

uint8_t
NbIotBsrManager::ConvertBytesToBsrIndex(uint32_t bytes) const
{
    // BSR index table per 3GPP TS 36.321 Table 6.1.3.1-1
    // Simplified - actual table has 64 entries with specific ranges
    if (bytes == 0) return 0;
    if (bytes <= 10) return 1;
    if (bytes <= 14) return 2;
    if (bytes <= 20) return 3;
    if (bytes <= 28) return 4;
    if (bytes <= 38) return 5;
    if (bytes <= 53) return 6;
    if (bytes <= 74) return 7;
    if (bytes <= 102) return 8;
    if (bytes <= 142) return 9;
    if (bytes <= 198) return 10;
    // ... more entries
    if (bytes <= 150000) return 62;
    return 63; // >150000 bytes
}

// ====================== NbIotUeMac ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotUeMac);

TypeId
NbIotUeMac::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotUeMac")
        .SetParent<NbIotMac>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotUeMac>()
        .AddTraceSource("UlGrant",
                        "Uplink grant received",
                        MakeTraceSourceAccessor(&NbIotUeMac::m_ulGrantTrace),
                        "ns3::NbIotUeMac::UlGrantTracedCallback")
        .AddTraceSource("DlAssignment",
                        "Downlink assignment received",
                        MakeTraceSourceAccessor(&NbIotUeMac::m_dlAssignmentTrace),
                        "ns3::NbIotUeMac::DlAssignmentTracedCallback")
        .AddTraceSource("RaState",
                        "Random access state changes",
                        MakeTraceSourceAccessor(&NbIotUeMac::m_raStateTrace),
                        "ns3::NbIotUeMac::RaStateTracedCallback");
    return tid;
}

NbIotUeMac::NbIotUeMac()
    : m_connected(false)
    , m_hasUlGrant(false)
{
    NS_LOG_FUNCTION(this);
    
    m_raManager = CreateObject<NbIotUeRandomAccessManager>();
    m_raManager->SetMac(this);
    m_raManager->SetRachCompleteCallback(MakeCallback(&NbIotUeMac::RachComplete, this));
    
    m_bsrManager = CreateObject<NbIotBsrManager>();
}

NbIotUeMac::~NbIotUeMac()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotUeMac::DoDispose()
{
    NS_LOG_FUNCTION(this);
    
    m_phy = nullptr;
    m_rrc = nullptr;
    m_raManager = nullptr;
    m_bsrManager = nullptr;
    
    while (!m_txQueues.empty())
    {
        m_txQueues.clear();
    }
    
    NbIotMac::DoDispose();
}

void
NbIotUeMac::SetPhy(Ptr<NbIotUePhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    m_phy = phy;
}

Ptr<NbIotUePhy>
NbIotUeMac::GetPhy() const
{
    return m_phy;
}

void
NbIotUeMac::SetRrc(Ptr<NbIotUeRrc> rrc)
{
    NS_LOG_FUNCTION(this << rrc);
    m_rrc = rrc;
}

Ptr<NbIotUeRrc>
NbIotUeMac::GetRrc() const
{
    return m_rrc;
}

void
NbIotUeMac::ReceivePdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    m_macRxTrace(m_rnti, packet->GetSize());
    
    // Parse MAC PDU
    auto sdus = NbIotMacPdu::Deserialize(packet);
    
    for (const auto& [lcid, packets] : sdus)
    {
        for (const auto& sdu : packets)
        {
            NS_LOG_DEBUG("Received SDU on LCID " << static_cast<int>(lcid)
                         << ", size " << sdu->GetSize());
            
            // Deliver to RLC based on LCID
            // In full implementation, would call RLC->ReceivePdu()
        }
    }
}

void
NbIotUeMac::TransmitSdu(uint8_t lcid, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << static_cast<int>(lcid) << packet);
    
    m_txQueues[lcid].push(packet);
    
    // Update BSR
    uint32_t totalSize = 0;
    auto& queue = m_txQueues[lcid];
    std::queue<Ptr<Packet>> tempQueue = queue;
    while (!tempQueue.empty())
    {
        totalSize += tempQueue.front()->GetSize();
        tempQueue.pop();
    }
    
    uint8_t lcg = lcid / 2; // Simplified LCG mapping
    m_bsrManager->UpdateBufferStatus(lcg, totalSize);
    
    // If not connected, start random access
    if (!m_connected)
    {
        StartRandomAccess("data");
    }
    else if (m_hasUlGrant)
    {
        // Transmit immediately if we have a grant
        BuildAndTransmitPdu(m_currentUlGrant.mcs);
    }
}

void
NbIotUeMac::ProcessDci(const NbIotDci& dci)
{
    NS_LOG_FUNCTION(this << static_cast<int>(dci.format));
    
    switch (dci.format)
    {
        case NbIotDciFormat::DCI_N0:
            NotifyUlGrant(dci);
            break;
        case NbIotDciFormat::DCI_N1:
            NotifyDlAssignment(dci);
            break;
        case NbIotDciFormat::DCI_N2:
            // Paging - handled by RRC
            break;
    }
}

void
NbIotUeMac::StartRandomAccess(const std::string& reason)
{
    NS_LOG_FUNCTION(this << reason);
    
    if (m_raManager->GetState() != NbIotRaState::IDLE)
    {
        NS_LOG_DEBUG("Random access already in progress");
        return;
    }
    
    // Get coverage class from PHY
    NbIotCoverageClass ceLevel = NbIotCoverageClass::CE_LEVEL_0;
    if (m_phy)
    {
        ceLevel = m_phy->GetCoverageClass();
    }
    
    m_raManager->StartRach(ceLevel);
    m_raStateTrace(NbIotRaState::PREAMBLE_TRANSMISSION);
}

Ptr<NbIotUeRandomAccessManager>
NbIotUeMac::GetRaManager() const
{
    return m_raManager;
}

Ptr<NbIotBsrManager>
NbIotUeMac::GetBsrManager() const
{
    return m_bsrManager;
}

void
NbIotUeMac::NotifyUlGrant(const NbIotDci& dci)
{
    NS_LOG_FUNCTION(this);
    
    uint16_t tbs = GetTbsFromIndex(dci.mcs);
    
    m_ulGrantTrace(m_rnti, dci.mcs, tbs);
    
    NS_LOG_INFO("UE " << m_rnti << " received UL grant: MCS=" << static_cast<int>(dci.mcs)
                << ", TBS=" << tbs);
    
    m_hasUlGrant = true;
    m_currentUlGrant = dci;
    
    // Build and transmit MAC PDU
    BuildAndTransmitPdu(tbs);
}

void
NbIotUeMac::NotifyDlAssignment(const NbIotDci& dci)
{
    NS_LOG_FUNCTION(this);
    
    uint16_t tbs = GetTbsFromIndex(dci.mcs);
    
    m_dlAssignmentTrace(m_rnti, dci.mcs, tbs);
    
    NS_LOG_INFO("UE " << m_rnti << " received DL assignment: MCS=" << static_cast<int>(dci.mcs)
                << ", TBS=" << tbs);
    
    // PHY will handle NPDSCH reception
}

void
NbIotUeMac::UpdateBufferStatus(uint8_t lcid, uint32_t txQueueSize, uint32_t retxQueueSize)
{
    NS_LOG_FUNCTION(this << static_cast<int>(lcid) << txQueueSize << retxQueueSize);
    
    uint8_t lcg = lcid / 2;
    m_bsrManager->UpdateBufferStatus(lcg, txQueueSize + retxQueueSize);
}

void
NbIotUeMac::SetRrcConnectionCallback(RrcConnectionCallback callback)
{
    m_rrcConnectionCallback = callback;
}

void
NbIotUeMac::BuildAndTransmitPdu(uint16_t tbs)
{
    NS_LOG_FUNCTION(this << tbs);
    
    auto pdu = CreateObject<NbIotMacPdu>();
    uint16_t remainingSize = tbs;
    
    // Add BSR if triggered
    if (m_bsrManager->IsBsrTriggered() && remainingSize >= 2)
    {
        bool shortBsr;
        auto bsrData = m_bsrManager->GetBsrData(shortBsr);
        
        std::vector<uint8_t> bsrCe;
        if (shortBsr)
        {
            bsrCe.push_back((0 << 6) | bsrData[0]); // LCG ID + buffer size
        }
        else
        {
            bsrCe.push_back(bsrData[0]);
            bsrCe.push_back(bsrData[1]);
            bsrCe.push_back((bsrData[2] << 2) | bsrData[3]);
        }
        
        pdu->AddControlElement(shortBsr ? NbIotMacSubheader::SHORT_BSR : NbIotMacSubheader::LONG_BSR, bsrCe);
        remainingSize -= (shortBsr ? 2 : 4);
        m_bsrManager->ClearBsrTrigger();
    }
    
    // Add SDUs from transmit queues
    for (auto& [lcid, queue] : m_txQueues)
    {
        while (!queue.empty() && remainingSize > 3) // Minimum subheader + 1 byte
        {
            Ptr<Packet> sdu = queue.front();
            uint16_t sduSize = sdu->GetSize();
            
            if (sduSize + 2 <= remainingSize) // 2 bytes for subheader
            {
                pdu->AddSdu(lcid, sdu);
                remainingSize -= (sduSize + 2);
                queue.pop();
            }
            else
            {
                // Would need segmentation - simplified for now
                break;
            }
        }
    }
    
    // Add padding if needed
    if (pdu->GetSize() < tbs)
    {
        pdu->AddPadding(tbs);
    }
    
    // Serialize and transmit
    Ptr<Packet> macPdu = pdu->Serialize();
    
    m_macTxTrace(m_rnti, macPdu->GetSize());
    
    // Send to PHY for NPUSCH transmission
    if (m_phy)
    {
        uint8_t reps = NPUSCH_FORMAT1_REPETITIONS[m_currentUlGrant.repetitionNumber % NPUSCH_FORMAT1_REPETITIONS.size()];
        m_phy->TransmitNpuschFormat1(macPdu, tbs, 12, reps, NbIotSubcarrierSpacing::SPACING_15_KHZ);
    }
    
    m_hasUlGrant = false;
}

void
NbIotUeMac::RachComplete(bool success, uint16_t cRnti)
{
    NS_LOG_FUNCTION(this << success << cRnti);
    
    if (success)
    {
        m_connected = true;
        m_rnti = cRnti;
        m_raStateTrace(NbIotRaState::CONNECTED);
        
        NS_LOG_INFO("UE connected with C-RNTI " << cRnti);
        
        if (!m_rrcConnectionCallback.IsNull())
        {
            m_rrcConnectionCallback(cRnti);
        }
    }
    else
    {
        m_raStateTrace(NbIotRaState::IDLE);
        NS_LOG_WARN("UE random access failed");
    }
}

} // namespace ns3
