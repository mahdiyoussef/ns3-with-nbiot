/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT PDCP Layer implementation
 */

#include "nbiot-pdcp.h"
#include "nbiot-rlc.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>
#include <ns3/boolean.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotPdcp");

// ====================== NbIotPdcpHeader ======================

NbIotPdcpHeader::NbIotPdcpHeader()
    : m_dc(true)
    , m_sn(0)
    , m_snLength(12)
{
}

void NbIotPdcpHeader::SetDcBit(bool dc) { m_dc = dc; }
bool NbIotPdcpHeader::GetDcBit() const { return m_dc; }
void NbIotPdcpHeader::SetSequenceNumber(uint16_t sn) { m_sn = sn; }
uint16_t NbIotPdcpHeader::GetSequenceNumber() const { return m_sn; }
void NbIotPdcpHeader::SetSnLength(uint8_t length) { m_snLength = length; }
uint8_t NbIotPdcpHeader::GetSnLength() const { return m_snLength; }

uint32_t NbIotPdcpHeader::GetSerializedSize() const
{
    // Header size depends on SN length and bearer type
    if (m_snLength <= 5)
        return 1;
    else if (m_snLength <= 12)
        return 2;
    else
        return 3;
}

void NbIotPdcpHeader::Serialize(uint8_t* buffer) const
{
    if (m_snLength <= 5)
    {
        // 5-bit SN (SRB)
        buffer[0] = (m_sn & 0x1F);
    }
    else if (m_snLength <= 12)
    {
        // 12-bit SN (DRB)
        buffer[0] = ((m_dc ? 1 : 0) << 7) | ((m_sn >> 8) & 0x0F);
        buffer[1] = m_sn & 0xFF;
    }
    else
    {
        // 18-bit SN
        buffer[0] = ((m_dc ? 1 : 0) << 7) | ((m_sn >> 16) & 0x03);
        buffer[1] = (m_sn >> 8) & 0xFF;
        buffer[2] = m_sn & 0xFF;
    }
}

uint32_t NbIotPdcpHeader::Deserialize(const uint8_t* buffer, uint32_t size)
{
    if (m_snLength <= 5)
    {
        if (size < 1) return 0;
        m_sn = buffer[0] & 0x1F;
        return 1;
    }
    else if (m_snLength <= 12)
    {
        if (size < 2) return 0;
        m_dc = (buffer[0] >> 7) & 0x01;
        m_sn = ((buffer[0] & 0x0F) << 8) | buffer[1];
        return 2;
    }
    else
    {
        if (size < 3) return 0;
        m_dc = (buffer[0] >> 7) & 0x01;
        m_sn = ((buffer[0] & 0x03) << 16) | (buffer[1] << 8) | buffer[2];
        return 3;
    }
}

// ====================== NbIotPdcp ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotPdcp);

TypeId
NbIotPdcp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotPdcp")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotPdcp>()
        .AddAttribute("SnLength",
                      "PDCP SN length in bits",
                      UintegerValue(12),
                      MakeUintegerAccessor(&NbIotPdcp::m_snLength),
                      MakeUintegerChecker<uint8_t>(5, 18))
        .AddAttribute("DiscardTimer",
                      "Discard timer value",
                      TimeValue(MilliSeconds(100)),
                      MakeTimeAccessor(&NbIotPdcp::m_discardTimer),
                      MakeTimeChecker())
        .AddAttribute("TReordering",
                      "Reordering timer value",
                      TimeValue(MilliSeconds(100)),
                      MakeTimeAccessor(&NbIotPdcp::m_tReordering),
                      MakeTimeChecker())
        .AddAttribute("HeaderCompression",
                      "Enable header compression",
                      BooleanValue(false),
                      MakeBooleanAccessor(&NbIotPdcp::m_headerCompression),
                      MakeBooleanChecker())
        .AddTraceSource("TxPdu",
                        "PDCP PDU transmitted",
                        MakeTraceSourceAccessor(&NbIotPdcp::m_txPduTrace),
                        "ns3::NbIotPdcp::TxPduTracedCallback")
        .AddTraceSource("RxPdu",
                        "PDCP PDU received",
                        MakeTraceSourceAccessor(&NbIotPdcp::m_rxPduTrace),
                        "ns3::NbIotPdcp::RxPduTracedCallback")
        .AddTraceSource("TxDelay",
                        "Transmission delay",
                        MakeTraceSourceAccessor(&NbIotPdcp::m_txDelayTrace),
                        "ns3::NbIotPdcp::TxDelayTracedCallback");
    return tid;
}

NbIotPdcp::NbIotPdcp()
    : m_rnti(0)
    , m_lcid(0)
    , m_isDrb(false)
    , m_headerCompression(false)
    , m_snLength(12)
    , m_txNext(0)
    , m_rxNext(0)
    , m_rxDeliv(0)
    , m_rxReord(0)
{
    NS_LOG_FUNCTION(this);
    
    m_snModulus = (1 << m_snLength);
    m_windowSize = m_snModulus / 2;
}

NbIotPdcp::~NbIotPdcp()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotPdcp::DoDispose()
{
    m_rlc = nullptr;
    m_txBuffer.clear();
    m_rxBuffer.clear();
    
    for (auto& [sn, timer] : m_discardTimers)
    {
        if (timer.IsPending())
        {
            Simulator::Cancel(timer);
        }
    }
    m_discardTimers.clear();
    
    if (m_reorderingTimer.IsPending())
    {
        Simulator::Cancel(m_reorderingTimer);
    }
    
    Object::DoDispose();
}

void
NbIotPdcp::SetBearerConfig(uint16_t rnti, uint8_t lcid, bool isDataRadioBearer)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<int>(lcid) << isDataRadioBearer);
    
    m_rnti = rnti;
    m_lcid = lcid;
    m_isDrb = isDataRadioBearer;
    
    // SRB uses 5-bit SN, DRB uses 12-bit
    if (!m_isDrb)
    {
        m_snLength = 5;
    }
    
    m_snModulus = (1 << m_snLength);
    m_windowSize = m_snModulus / 2;
}

void
NbIotPdcp::SetRlc(Ptr<NbIotRlc> rlc)
{
    NS_LOG_FUNCTION(this << rlc);
    m_rlc = rlc;
}

Ptr<NbIotRlc>
NbIotPdcp::GetRlc() const
{
    return m_rlc;
}

void NbIotPdcp::SetHeaderCompression(bool enabled) { m_headerCompression = enabled; }
void NbIotPdcp::SetSnLength(uint8_t length)
{
    m_snLength = length;
    m_snModulus = (1 << m_snLength);
    m_windowSize = m_snModulus / 2;
}
void NbIotPdcp::SetDiscardTimer(Time timer) { m_discardTimer = timer; }

uint16_t NbIotPdcp::GetRnti() const { return m_rnti; }
uint8_t NbIotPdcp::GetLcId() const { return m_lcid; }

void
NbIotPdcp::TransmitSdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    // Apply header compression if enabled
    Ptr<Packet> processedPacket = packet;
    if (m_headerCompression)
    {
        processedPacket = CompressHeader(packet);
    }
    
    // Assign PDCP SN
    uint32_t pdcpSn = m_txNext;
    m_txNext = (m_txNext + 1) % m_snModulus;
    
    // Add PDCP header
    NbIotPdcpHeader header;
    header.SetDcBit(true);  // Data PDU
    header.SetSnLength(m_snLength);
    header.SetSequenceNumber(pdcpSn);
    
    uint8_t headerBuf[4];
    header.Serialize(headerBuf);
    
    Ptr<Packet> pdcpPdu = Create<Packet>(headerBuf, header.GetSerializedSize());
    pdcpPdu->AddAtEnd(processedPacket);
    
    // Store in buffer
    NbIotPdcpSduInfo sduInfo;
    sduInfo.packet = pdcpPdu;
    sduInfo.pdcpSn = pdcpSn;
    sduInfo.arrivalTime = Simulator::Now();
    
    m_txBuffer.push_back(sduInfo);
    
    // Start discard timer
    if (m_discardTimer > Seconds(0))
    {
        m_discardTimers[pdcpSn] = Simulator::Schedule(m_discardTimer,
                                                       &NbIotPdcp::DiscardTimerExpiry,
                                                       this,
                                                       pdcpSn);
    }
    
    m_txPduTrace(m_rnti, m_lcid, pdcpPdu->GetSize());
    
    NS_LOG_DEBUG("PDCP: TX SDU with SN=" << pdcpSn << ", size=" << pdcpPdu->GetSize());
    
    // Pass to RLC
    if (m_rlc)
    {
        m_rlc->TransmitSdu(pdcpPdu);
    }
}

void
NbIotPdcp::ReceivePdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    m_rxPduTrace(m_rnti, m_lcid, packet->GetSize());
    
    // Parse PDCP header
    uint8_t headerBuf[4];
    packet->CopyData(headerBuf, std::min(packet->GetSize(), static_cast<uint32_t>(4)));
    
    NbIotPdcpHeader header;
    header.SetSnLength(m_snLength);
    uint32_t headerSize = header.Deserialize(headerBuf, packet->GetSize());
    
    uint32_t rxSn = header.GetSequenceNumber();
    
    NS_LOG_DEBUG("PDCP: RX PDU with SN=" << rxSn);
    
    // Remove header
    Ptr<Packet> sdu = packet->Copy();
    sdu->RemoveAtStart(headerSize);
    
    // Apply header decompression if enabled
    if (m_headerCompression)
    {
        sdu = DecompressHeader(sdu);
    }
    
    // For SRB (RLC AM), deliver in order is guaranteed by RLC
    // For DRB with UM, need reordering
    if (!m_isDrb || (m_rlc && m_rlc->GetRlcMode() == NbIotRlcMode::AM))
    {
        // Direct delivery
        if (!m_receiveSduCallback.IsNull())
        {
            m_receiveSduCallback(sdu);
        }
        
        m_rxNext = (rxSn + 1) % m_snModulus;
        m_rxDeliv = m_rxNext;
    }
    else
    {
        // Store for reordering
        m_rxBuffer[rxSn] = sdu;
        
        // Update RX_NEXT
        if (((rxSn - m_rxNext) % m_snModulus) < m_windowSize)
        {
            m_rxNext = (rxSn + 1) % m_snModulus;
        }
        
        // Handle reordering timer
        if (!m_reorderingTimer.IsPending() && m_rxDeliv != m_rxNext)
        {
            m_rxReord = m_rxNext;
            m_reorderingTimer = Simulator::Schedule(m_tReordering,
                                                     &NbIotPdcp::ReorderingTimerExpiry,
                                                     this);
        }
        
        DeliverInOrder();
    }
}

uint32_t
NbIotPdcp::GetTxBufferSize() const
{
    uint32_t size = 0;
    for (const auto& sdu : m_txBuffer)
    {
        size += sdu.packet->GetSize();
    }
    return size;
}

void
NbIotPdcp::SetReceiveSduCallback(ReceiveSduCallback cb)
{
    m_receiveSduCallback = cb;
}

Ptr<Packet>
NbIotPdcp::CompressHeader(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    // Simplified ROHC - just copy packet
    // Real implementation would apply IP/UDP/RTP header compression
    return packet->Copy();
}

Ptr<Packet>
NbIotPdcp::DecompressHeader(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    // Simplified ROHC decompression
    return packet->Copy();
}

void
NbIotPdcp::DeliverInOrder()
{
    NS_LOG_FUNCTION(this);
    
    // Deliver all SDUs from RX_DELIV in order
    while (m_rxBuffer.find(m_rxDeliv) != m_rxBuffer.end())
    {
        Ptr<Packet> sdu = m_rxBuffer[m_rxDeliv];
        m_rxBuffer.erase(m_rxDeliv);
        
        if (!m_receiveSduCallback.IsNull())
        {
            m_receiveSduCallback(sdu);
        }
        
        NS_LOG_DEBUG("PDCP: Delivered SDU SN=" << m_rxDeliv);
        
        m_rxDeliv = (m_rxDeliv + 1) % m_snModulus;
    }
    
    // Stop reordering timer if all delivered
    if (m_rxDeliv == m_rxNext && m_reorderingTimer.IsPending())
    {
        Simulator::Cancel(m_reorderingTimer);
    }
}

void
NbIotPdcp::DiscardTimerExpiry(uint32_t pdcpSn)
{
    NS_LOG_FUNCTION(this << pdcpSn);
    
    // Remove from buffer and notify RLC
    for (auto it = m_txBuffer.begin(); it != m_txBuffer.end(); ++it)
    {
        if (it->pdcpSn == pdcpSn)
        {
            NS_LOG_DEBUG("PDCP: Discarding SDU SN=" << pdcpSn);
            m_txBuffer.erase(it);
            break;
        }
    }
    
    m_discardTimers.erase(pdcpSn);
}

void
NbIotPdcp::ReorderingTimerExpiry()
{
    NS_LOG_FUNCTION(this);
    
    // Deliver all SDUs up to RX_REORD - 1
    // Then discard any missing
    
    while (m_rxDeliv != m_rxReord)
    {
        auto it = m_rxBuffer.find(m_rxDeliv);
        if (it != m_rxBuffer.end())
        {
            if (!m_receiveSduCallback.IsNull())
            {
                m_receiveSduCallback(it->second);
            }
            m_rxBuffer.erase(it);
        }
        
        m_rxDeliv = (m_rxDeliv + 1) % m_snModulus;
    }
    
    DeliverInOrder();
    
    // Restart timer if needed
    if (m_rxDeliv != m_rxNext)
    {
        m_rxReord = m_rxNext;
        m_reorderingTimer = Simulator::Schedule(m_tReordering,
                                                 &NbIotPdcp::ReorderingTimerExpiry,
                                                 this);
    }
}

// ====================== NbIotPdcpSrb ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotPdcpSrb);

TypeId
NbIotPdcpSrb::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotPdcpSrb")
        .SetParent<NbIotPdcp>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotPdcpSrb>();
    return tid;
}

NbIotPdcpSrb::NbIotPdcpSrb()
    : m_integrityProtection(false)
    , m_ciphering(false)
{
    NS_LOG_FUNCTION(this);
    
    // SRB uses 5-bit SN
    SetSnLength(5);
}

NbIotPdcpSrb::~NbIotPdcpSrb()
{
    NS_LOG_FUNCTION(this);
}

void NbIotPdcpSrb::SetIntegrityProtection(bool enabled) { m_integrityProtection = enabled; }
void NbIotPdcpSrb::SetCiphering(bool enabled) { m_ciphering = enabled; }

// ====================== NbIotPdcpDrb ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotPdcpDrb);

TypeId
NbIotPdcpDrb::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotPdcpDrb")
        .SetParent<NbIotPdcp>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotPdcpDrb>()
        .AddAttribute("RohcProfile",
                      "ROHC profile ID",
                      UintegerValue(0),
                      MakeUintegerAccessor(&NbIotPdcpDrb::m_rohcProfile),
                      MakeUintegerChecker<uint16_t>());
    return tid;
}

NbIotPdcpDrb::NbIotPdcpDrb()
    : m_rohcProfile(0)
{
    NS_LOG_FUNCTION(this);
    
    // DRB uses 12-bit SN by default
    SetSnLength(12);
}

NbIotPdcpDrb::~NbIotPdcpDrb()
{
    NS_LOG_FUNCTION(this);
}

void NbIotPdcpDrb::SetRohcProfile(uint16_t profile) { m_rohcProfile = profile; }

} // namespace ns3
