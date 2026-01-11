/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT RLC Layer implementation
 */

#include "nbiot-rlc.h"
#include "nbiot-mac.h"
#include "nbiot-pdcp.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cstring>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotRlc");

// ====================== NbIotRlcUmHeader ======================

NbIotRlcUmHeader::NbIotRlcUmHeader()
    : m_fi(0)
    , m_e(false)
    , m_sn(0)
{
}

void NbIotRlcUmHeader::SetSequenceNumber(uint16_t sn) { m_sn = sn; }
uint16_t NbIotRlcUmHeader::GetSequenceNumber() const { return m_sn; }
void NbIotRlcUmHeader::SetFramingInfo(uint8_t fi) { m_fi = fi; }
uint8_t NbIotRlcUmHeader::GetFramingInfo() const { return m_fi; }
void NbIotRlcUmHeader::SetExtensionBit(bool e) { m_e = e; }
bool NbIotRlcUmHeader::GetExtensionBit() const { return m_e; }
void NbIotRlcUmHeader::AddLengthIndicator(uint16_t li) { m_lis.push_back(li); }
std::vector<uint16_t> NbIotRlcUmHeader::GetLengthIndicators() const { return m_lis; }

uint32_t NbIotRlcUmHeader::GetSerializedSize() const
{
    // 1 byte for FI/E/SN, plus 2 bytes per LI
    return 1 + 2 * m_lis.size();
}

void NbIotRlcUmHeader::Serialize(uint8_t* buffer) const
{
    buffer[0] = ((m_fi & 0x03) << 6) | ((m_e ? 1 : 0) << 5) | (m_sn & 0x1F);
    
    uint32_t offset = 1;
    for (size_t i = 0; i < m_lis.size(); ++i)
    {
        bool e = (i < m_lis.size() - 1);
        buffer[offset++] = ((e ? 1 : 0) << 7) | ((m_lis[i] >> 4) & 0x7F);
        buffer[offset++] = (m_lis[i] & 0x0F) << 4;
    }
}

uint32_t NbIotRlcUmHeader::Deserialize(const uint8_t* buffer, uint32_t size)
{
    if (size < 1) return 0;
    
    m_fi = (buffer[0] >> 6) & 0x03;
    m_e = (buffer[0] >> 5) & 0x01;
    m_sn = buffer[0] & 0x1F;
    
    uint32_t offset = 1;
    m_lis.clear();
    
    while (m_e && offset + 1 < size)
    {
        bool nextE = (buffer[offset] >> 7) & 0x01;
        uint16_t li = ((buffer[offset] & 0x7F) << 4) | ((buffer[offset + 1] >> 4) & 0x0F);
        m_lis.push_back(li);
        offset += 2;
        m_e = nextE;
    }
    
    return offset;
}

// ====================== NbIotRlcAmHeader ======================

NbIotRlcAmHeader::NbIotRlcAmHeader()
    : m_dc(true)
    , m_rf(false)
    , m_p(false)
    , m_fi(0)
    , m_e(false)
    , m_sn(0)
    , m_lsf(false)
    , m_so(0)
{
}

void NbIotRlcAmHeader::SetDataControlBit(bool dc) { m_dc = dc; }
bool NbIotRlcAmHeader::IsDataPdu() const { return m_dc; }
void NbIotRlcAmHeader::SetResegmentationFlag(bool rf) { m_rf = rf; }
bool NbIotRlcAmHeader::GetResegmentationFlag() const { return m_rf; }
void NbIotRlcAmHeader::SetPollingBit(bool p) { m_p = p; }
bool NbIotRlcAmHeader::GetPollingBit() const { return m_p; }
void NbIotRlcAmHeader::SetFramingInfo(uint8_t fi) { m_fi = fi; }
uint8_t NbIotRlcAmHeader::GetFramingInfo() const { return m_fi; }
void NbIotRlcAmHeader::SetSequenceNumber(uint16_t sn) { m_sn = sn; }
uint16_t NbIotRlcAmHeader::GetSequenceNumber() const { return m_sn; }
void NbIotRlcAmHeader::SetLastSegmentFlag(bool lsf) { m_lsf = lsf; }
bool NbIotRlcAmHeader::GetLastSegmentFlag() const { return m_lsf; }
void NbIotRlcAmHeader::SetSegmentOffset(uint16_t so) { m_so = so; }
uint16_t NbIotRlcAmHeader::GetSegmentOffset() const { return m_so; }
void NbIotRlcAmHeader::AddLengthIndicator(uint16_t li) { m_lis.push_back(li); }
std::vector<uint16_t> NbIotRlcAmHeader::GetLengthIndicators() const { return m_lis; }

uint32_t NbIotRlcAmHeader::GetSerializedSize() const
{
    uint32_t size = 2;  // D/C, RF, P, FI, E, SN (2 bytes)
    if (m_rf)
    {
        size += 2;      // LSF, SO (2 bytes)
    }
    size += 2 * m_lis.size();
    return size;
}

void NbIotRlcAmHeader::Serialize(uint8_t* buffer) const
{
    buffer[0] = ((m_dc ? 1 : 0) << 7) | ((m_rf ? 1 : 0) << 6) | ((m_p ? 1 : 0) << 5)
                | ((m_fi & 0x03) << 3) | ((m_e ? 1 : 0) << 2) | ((m_sn >> 8) & 0x03);
    buffer[1] = m_sn & 0xFF;
    
    uint32_t offset = 2;
    if (m_rf)
    {
        buffer[offset++] = ((m_lsf ? 1 : 0) << 7) | ((m_so >> 8) & 0x7F);
        buffer[offset++] = m_so & 0xFF;
    }
    
    for (size_t i = 0; i < m_lis.size(); ++i)
    {
        bool e = (i < m_lis.size() - 1);
        buffer[offset++] = ((e ? 1 : 0) << 7) | ((m_lis[i] >> 4) & 0x7F);
        buffer[offset++] = (m_lis[i] & 0x0F) << 4;
    }
}

uint32_t NbIotRlcAmHeader::Deserialize(const uint8_t* buffer, uint32_t size)
{
    if (size < 2) return 0;
    
    m_dc = (buffer[0] >> 7) & 0x01;
    m_rf = (buffer[0] >> 6) & 0x01;
    m_p = (buffer[0] >> 5) & 0x01;
    m_fi = (buffer[0] >> 3) & 0x03;
    m_e = (buffer[0] >> 2) & 0x01;
    m_sn = ((buffer[0] & 0x03) << 8) | buffer[1];
    
    uint32_t offset = 2;
    if (m_rf)
    {
        if (size < 4) return 0;
        m_lsf = (buffer[offset] >> 7) & 0x01;
        m_so = ((buffer[offset] & 0x7F) << 8) | buffer[offset + 1];
        offset += 2;
    }
    
    m_lis.clear();
    while (m_e && offset + 1 < size)
    {
        bool nextE = (buffer[offset] >> 7) & 0x01;
        uint16_t li = ((buffer[offset] & 0x7F) << 4) | ((buffer[offset + 1] >> 4) & 0x0F);
        m_lis.push_back(li);
        offset += 2;
        m_e = nextE;
    }
    
    return offset;
}

// ====================== NbIotRlc ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotRlc);

TypeId
NbIotRlc::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotRlc")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddTraceSource("TxPdu",
                        "PDU transmitted",
                        MakeTraceSourceAccessor(&NbIotRlc::m_txPduTrace),
                        "ns3::NbIotRlc::TxPduTracedCallback")
        .AddTraceSource("RxPdu",
                        "PDU received",
                        MakeTraceSourceAccessor(&NbIotRlc::m_rxPduTrace),
                        "ns3::NbIotRlc::RxPduTracedCallback")
        .AddTraceSource("TxSdu",
                        "SDU sent to MAC",
                        MakeTraceSourceAccessor(&NbIotRlc::m_txSduTrace),
                        "ns3::NbIotRlc::TxSduTracedCallback")
        .AddTraceSource("RxSdu",
                        "SDU delivered to PDCP",
                        MakeTraceSourceAccessor(&NbIotRlc::m_rxSduTrace),
                        "ns3::NbIotRlc::RxSduTracedCallback");
    return tid;
}

NbIotRlc::NbIotRlc()
    : m_lcid(0)
    , m_rnti(0)
{
}

NbIotRlc::~NbIotRlc()
{
}

void
NbIotRlc::DoDispose()
{
    m_mac = nullptr;
    m_pdcp = nullptr;
    Object::DoDispose();
}

void NbIotRlc::SetLcId(uint8_t lcid) { m_lcid = lcid; }
uint8_t NbIotRlc::GetLcId() const { return m_lcid; }
void NbIotRlc::SetRnti(uint16_t rnti) { m_rnti = rnti; }
uint16_t NbIotRlc::GetRnti() const { return m_rnti; }
void NbIotRlc::SetMac(Ptr<NbIotMac> mac) { m_mac = mac; }
void NbIotRlc::SetPdcp(Ptr<NbIotPdcp> pdcp) { m_pdcp = pdcp; }

// ====================== NbIotRlcTm ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotRlcTm);

TypeId
NbIotRlcTm::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotRlcTm")
        .SetParent<NbIotRlc>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotRlcTm>();
    return tid;
}

NbIotRlcTm::NbIotRlcTm()
{
    NS_LOG_FUNCTION(this);
}

NbIotRlcTm::~NbIotRlcTm()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotRlcTm::DoDispose()
{
    m_txBuffer.clear();
    NbIotRlc::DoDispose();
}

void
NbIotRlcTm::TransmitSdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    NbIotRlcSduInfo sdu;
    sdu.packet = packet;
    sdu.arrivalTime = Simulator::Now();
    
    m_txBuffer.push_back(sdu);
    
    m_txSduTrace(m_rnti, m_lcid, packet->GetSize());
    
    NS_LOG_DEBUG("TM: Queued SDU, buffer size=" << m_txBuffer.size());
}

void
NbIotRlcTm::ReceivePdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    m_rxPduTrace(m_rnti, m_lcid, packet->GetSize());
    
    // TM: Pass through directly to upper layer
    m_rxSduTrace(m_rnti, m_lcid, packet->GetSize());
    
    // Deliver to PDCP if configured
    // if (m_pdcp) m_pdcp->ReceiveSdu(packet);
    
    NS_LOG_DEBUG("TM: Delivered PDU/SDU, size=" << packet->GetSize());
}

Ptr<Packet>
NbIotRlcTm::NotifyTxOpportunity(uint32_t bytes)
{
    NS_LOG_FUNCTION(this << bytes);
    
    if (m_txBuffer.empty())
    {
        return nullptr;
    }
    
    auto& sdu = m_txBuffer.front();
    if (sdu.packet->GetSize() > bytes)
    {
        NS_LOG_WARN("TM: SDU too large for TX opportunity");
        return nullptr;
    }
    
    Ptr<Packet> pdu = sdu.packet;
    m_txBuffer.pop_front();
    
    m_txPduTrace(m_rnti, m_lcid, pdu->GetSize());
    
    NS_LOG_DEBUG("TM: Transmitted PDU, size=" << pdu->GetSize());
    
    return pdu;
}

uint32_t
NbIotRlcTm::GetTxBufferSize() const
{
    uint32_t size = 0;
    for (const auto& sdu : m_txBuffer)
    {
        size += sdu.packet->GetSize();
    }
    return size;
}

NbIotRlcMode
NbIotRlcTm::GetRlcMode() const
{
    return NbIotRlcMode::TM;
}

// ====================== NbIotRlcUm ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotRlcUm);

TypeId
NbIotRlcUm::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotRlcUm")
        .SetParent<NbIotRlc>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotRlcUm>()
        .AddAttribute("SnFieldLength",
                      "SN field length in bits",
                      UintegerValue(5),
                      MakeUintegerAccessor(&NbIotRlcUm::m_snLength),
                      MakeUintegerChecker<uint8_t>(5, 10))
        .AddAttribute("TReordering",
                      "t-Reordering timer",
                      TimeValue(MilliSeconds(100)),
                      MakeTimeAccessor(&NbIotRlcUm::m_tReordering),
                      MakeTimeChecker());
    return tid;
}

NbIotRlcUm::NbIotRlcUm()
    : m_txSn(0)
    , m_snLength(5)
    , m_vrUr(0)
    , m_vrUx(0)
    , m_vrUh(0)
{
    NS_LOG_FUNCTION(this);
    
    m_snModulus = (1 << m_snLength);
    m_windowSize = m_snModulus / 2;
}

NbIotRlcUm::~NbIotRlcUm()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotRlcUm::DoDispose()
{
    m_txBuffer.clear();
    m_rxBuffer.clear();
    
    if (m_reorderingTimer.IsPending())
    {
        Simulator::Cancel(m_reorderingTimer);
    }
    
    NbIotRlc::DoDispose();
}

void
NbIotRlcUm::SetSnFieldLength(uint8_t length)
{
    m_snLength = length;
    m_snModulus = (1 << m_snLength);
    m_windowSize = m_snModulus / 2;
}

void
NbIotRlcUm::SetTReordering(Time timer)
{
    m_tReordering = timer;
}

void
NbIotRlcUm::TransmitSdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    NbIotRlcSduInfo sdu;
    sdu.packet = packet;
    sdu.arrivalTime = Simulator::Now();
    
    m_txBuffer.push_back(sdu);
    
    m_txSduTrace(m_rnti, m_lcid, packet->GetSize());
    
    NS_LOG_DEBUG("UM: Queued SDU, buffer size=" << m_txBuffer.size());
}

void
NbIotRlcUm::ReceivePdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    m_rxPduTrace(m_rnti, m_lcid, packet->GetSize());
    
    // Parse header
    uint8_t headerBuf[256];
    packet->CopyData(headerBuf, std::min(packet->GetSize(), static_cast<uint32_t>(256)));
    
    NbIotRlcUmHeader header;
    uint32_t headerSize = header.Deserialize(headerBuf, packet->GetSize());
    
    uint16_t sn = header.GetSequenceNumber();
    
    NS_LOG_DEBUG("UM: Received PDU SN=" << sn);
    
    // Check if within window
    if (!IsInsideWindow(sn))
    {
        NS_LOG_WARN("UM: Discarding PDU outside window, SN=" << sn);
        return;
    }
    
    // Store in receive buffer
    NbIotRlcPduSegment segment;
    segment.sn = sn;
    segment.so = 0;
    segment.data = packet->Copy();
    segment.data->RemoveAtStart(headerSize);
    segment.isFirst = ((header.GetFramingInfo() & 0x02) == 0);
    segment.isLast = ((header.GetFramingInfo() & 0x01) == 0);
    
    m_rxBuffer[sn] = segment;
    
    // Update VR(UH)
    if (((m_vrUh - sn) % m_snModulus) > m_windowSize)
    {
        m_vrUh = (sn + 1) % m_snModulus;
    }
    
    // Handle reordering timer
    if (!m_reorderingTimer.IsPending())
    {
        if (m_vrUh != m_vrUr)
        {
            m_vrUx = m_vrUh;
            m_reorderingTimer = Simulator::Schedule(m_tReordering,
                                                     &NbIotRlcUm::TReorderingExpiry,
                                                     this);
        }
    }
    
    ReassembleSdu();
}

Ptr<Packet>
NbIotRlcUm::NotifyTxOpportunity(uint32_t bytes)
{
    NS_LOG_FUNCTION(this << bytes);
    
    if (m_txBuffer.empty())
    {
        return nullptr;
    }
    
    // Build PDU from buffer
    uint32_t headerSize = 1;  // Minimum header
    if (bytes < headerSize + 1)
    {
        return nullptr;
    }
    
    uint32_t dataBytes = bytes - headerSize;
    
    NbIotRlcUmHeader header;
    header.SetSequenceNumber(m_txSn);
    
    Ptr<Packet> pdu = Create<Packet>();
    
    // Simplified: take one SDU per PDU
    auto& sdu = m_txBuffer.front();
    
    if (sdu.packet->GetSize() <= dataBytes)
    {
        // Complete SDU fits
        header.SetFramingInfo(0x00);  // First and last byte of SDU
        pdu->AddAtEnd(sdu.packet);
        m_txBuffer.pop_front();
    }
    else
    {
        // Segmentation needed (simplified)
        header.SetFramingInfo(0x01);  // First byte but not last
        Ptr<Packet> segment = sdu.packet->CreateFragment(0, dataBytes);
        sdu.packet->RemoveAtStart(dataBytes);
        pdu->AddAtEnd(segment);
    }
    
    // Add header
    uint8_t headerBuf[16];
    header.Serialize(headerBuf);
    Ptr<Packet> headerPkt = Create<Packet>(headerBuf, header.GetSerializedSize());
    headerPkt->AddAtEnd(pdu);
    
    m_txSn = (m_txSn + 1) % m_snModulus;
    
    m_txPduTrace(m_rnti, m_lcid, headerPkt->GetSize());
    
    NS_LOG_DEBUG("UM: Transmitted PDU SN=" << static_cast<int>((m_txSn - 1 + m_snModulus) % m_snModulus));
    
    return headerPkt;
}

uint32_t
NbIotRlcUm::GetTxBufferSize() const
{
    uint32_t size = 0;
    for (const auto& sdu : m_txBuffer)
    {
        size += sdu.packet->GetSize() + 1;  // +1 for header overhead
    }
    return size;
}

NbIotRlcMode
NbIotRlcUm::GetRlcMode() const
{
    return NbIotRlcMode::UM;
}

bool
NbIotRlcUm::IsInsideWindow(uint16_t sn) const
{
    uint16_t diff = (m_vrUh - sn) % m_snModulus;
    return diff <= m_windowSize;
}

void
NbIotRlcUm::ReassembleSdu()
{
    NS_LOG_FUNCTION(this);
    
    // Try to deliver PDUs in order starting from VR(UR)
    while (m_rxBuffer.find(m_vrUr) != m_rxBuffer.end())
    {
        auto& segment = m_rxBuffer[m_vrUr];
        
        // Deliver to upper layer
        m_rxSduTrace(m_rnti, m_lcid, segment.data->GetSize());
        
        NS_LOG_DEBUG("UM: Delivered SDU SN=" << m_vrUr);
        
        m_rxBuffer.erase(m_vrUr);
        m_vrUr = (m_vrUr + 1) % m_snModulus;
    }
}

void
NbIotRlcUm::TReorderingExpiry()
{
    NS_LOG_FUNCTION(this);
    
    // Discard all SNs < VR(UX)
    for (auto it = m_rxBuffer.begin(); it != m_rxBuffer.end();)
    {
        uint16_t diff = (m_vrUx - it->first) % m_snModulus;
        if (diff <= m_windowSize && diff > 0)
        {
            it = m_rxBuffer.erase(it);
        }
        else
        {
            ++it;
        }
    }
    
    m_vrUr = m_vrUx;
    
    ReassembleSdu();
    
    // Restart timer if needed
    if (m_vrUh != m_vrUr)
    {
        m_vrUx = m_vrUh;
        m_reorderingTimer = Simulator::Schedule(m_tReordering,
                                                 &NbIotRlcUm::TReorderingExpiry,
                                                 this);
    }
}

// ====================== NbIotRlcAm ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotRlcAm);

TypeId
NbIotRlcAm::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotRlcAm")
        .SetParent<NbIotRlc>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotRlcAm>()
        .AddAttribute("TPollRetransmit",
                      "t-PollRetransmit timer",
                      TimeValue(MilliSeconds(45)),
                      MakeTimeAccessor(&NbIotRlcAm::m_tPollRetransmit),
                      MakeTimeChecker())
        .AddAttribute("TReordering",
                      "t-Reordering timer",
                      TimeValue(MilliSeconds(35)),
                      MakeTimeAccessor(&NbIotRlcAm::m_tReordering),
                      MakeTimeChecker())
        .AddAttribute("TStatusProhibit",
                      "t-StatusProhibit timer",
                      TimeValue(MilliSeconds(10)),
                      MakeTimeAccessor(&NbIotRlcAm::m_tStatusProhibit),
                      MakeTimeChecker())
        .AddAttribute("MaxRetxThreshold",
                      "Maximum retransmissions",
                      UintegerValue(4),
                      MakeUintegerAccessor(&NbIotRlcAm::m_maxRetxThreshold),
                      MakeUintegerChecker<uint8_t>())
        .AddAttribute("PollPdu",
                      "PDUs before poll",
                      UintegerValue(16),
                      MakeUintegerAccessor(&NbIotRlcAm::m_pollPdu),
                      MakeUintegerChecker<uint16_t>())
        .AddAttribute("PollByte",
                      "Bytes before poll",
                      UintegerValue(1000),
                      MakeUintegerAccessor(&NbIotRlcAm::m_pollByte),
                      MakeUintegerChecker<uint32_t>());
    return tid;
}

NbIotRlcAm::NbIotRlcAm()
    : m_vtA(0)
    , m_vtS(0)
    , m_pollSn(0)
    , m_vrR(0)
    , m_vrX(0)
    , m_vrMs(0)
    , m_vrH(0)
    , m_maxRetxThreshold(4)
    , m_pollPdu(16)
    , m_pollByte(1000)
    , m_pduWithoutPoll(0)
    , m_byteWithoutPoll(0)
    , m_statusTriggered(false)
{
    NS_LOG_FUNCTION(this);
    
    m_vtMs = m_vtA + AM_WINDOW_SIZE;
    m_vrMr = m_vrR + AM_WINDOW_SIZE;
}

NbIotRlcAm::~NbIotRlcAm()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotRlcAm::DoDispose()
{
    m_txBuffer.clear();
    m_txedBuffer.clear();
    m_retxQueue.clear();
    m_rxBuffer.clear();
    m_nackList.clear();
    
    if (m_pollRetransmitTimer.IsPending())
        Simulator::Cancel(m_pollRetransmitTimer);
    if (m_reorderingTimer.IsPending())
        Simulator::Cancel(m_reorderingTimer);
    if (m_statusProhibitTimer.IsPending())
        Simulator::Cancel(m_statusProhibitTimer);
    
    NbIotRlc::DoDispose();
}

void NbIotRlcAm::SetTPollRetransmit(Time timer) { m_tPollRetransmit = timer; }
void NbIotRlcAm::SetTReordering(Time timer) { m_tReordering = timer; }
void NbIotRlcAm::SetTStatusProhibit(Time timer) { m_tStatusProhibit = timer; }
void NbIotRlcAm::SetMaxRetxThreshold(uint8_t maxRetx) { m_maxRetxThreshold = maxRetx; }
void NbIotRlcAm::SetPollPdu(uint16_t pollPdu) { m_pollPdu = pollPdu; }
void NbIotRlcAm::SetPollByte(uint32_t pollByte) { m_pollByte = pollByte; }

void
NbIotRlcAm::TransmitSdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    NbIotRlcSduInfo sdu;
    sdu.packet = packet;
    sdu.arrivalTime = Simulator::Now();
    
    m_txBuffer.push_back(sdu);
    
    m_txSduTrace(m_rnti, m_lcid, packet->GetSize());
    
    NS_LOG_DEBUG("AM: Queued SDU, buffer size=" << m_txBuffer.size());
}

void
NbIotRlcAm::ReceivePdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    m_rxPduTrace(m_rnti, m_lcid, packet->GetSize());
    
    uint8_t headerBuf[256];
    packet->CopyData(headerBuf, std::min(packet->GetSize(), static_cast<uint32_t>(256)));
    
    NbIotRlcAmHeader header;
    uint32_t headerSize = header.Deserialize(headerBuf, packet->GetSize());
    
    if (header.IsDataPdu())
    {
        ProcessDataPdu(packet, header);
    }
    else
    {
        ProcessStatusPdu(packet);
    }
}

Ptr<Packet>
NbIotRlcAm::NotifyTxOpportunity(uint32_t bytes)
{
    NS_LOG_FUNCTION(this << bytes);
    
    // Priority: Status PDU > Retransmissions > New data
    
    // 1. Check for status PDU
    if (m_statusTriggered && !m_statusProhibitTimer.IsPending())
    {
        Ptr<Packet> statusPdu = BuildStatusPdu();
        if (statusPdu && statusPdu->GetSize() <= bytes)
        {
            m_statusTriggered = false;
            m_statusProhibitTimer = Simulator::Schedule(m_tStatusProhibit,
                                                         &NbIotRlcAm::TStatusProhibitExpiry,
                                                         this);
            return statusPdu;
        }
    }
    
    // 2. Check retransmission queue
    if (!m_retxQueue.empty())
    {
        uint16_t sn = m_retxQueue.front();
        auto it = m_txedBuffer.find(sn);
        if (it != m_txedBuffer.end())
        {
            if (it->second->GetSize() <= bytes)
            {
                m_retxQueue.pop_front();
                m_retxCount[sn]++;
                
                if (m_retxCount[sn] >= m_maxRetxThreshold)
                {
                    NS_LOG_WARN("AM: Max retransmissions reached for SN=" << sn);
                    // Would trigger RLF here
                }
                
                NS_LOG_DEBUG("AM: Retransmitting SN=" << sn);
                return it->second->Copy();
            }
        }
        else
        {
            m_retxQueue.pop_front();
        }
    }
    
    // 3. New data transmission
    if (m_txBuffer.empty())
    {
        return nullptr;
    }
    
    uint32_t headerSize = 2;  // Minimum AM header
    if (bytes < headerSize + 1)
    {
        return nullptr;
    }
    
    uint32_t dataBytes = bytes - headerSize;
    
    NbIotRlcAmHeader header;
    header.SetDataControlBit(true);
    header.SetSequenceNumber(m_vtS);
    
    Ptr<Packet> pdu = Create<Packet>();
    
    auto& sdu = m_txBuffer.front();
    
    if (sdu.packet->GetSize() <= dataBytes)
    {
        header.SetFramingInfo(0x00);
        pdu->AddAtEnd(sdu.packet);
        m_txBuffer.pop_front();
    }
    else
    {
        header.SetFramingInfo(0x01);
        Ptr<Packet> segment = sdu.packet->CreateFragment(0, dataBytes);
        sdu.packet->RemoveAtStart(dataBytes);
        pdu->AddAtEnd(segment);
    }
    
    // Check if poll needed
    m_pduWithoutPoll++;
    m_byteWithoutPoll += pdu->GetSize();
    
    if (ShouldPoll())
    {
        header.SetPollingBit(true);
        m_pollSn = m_vtS;
        m_pduWithoutPoll = 0;
        m_byteWithoutPoll = 0;
        
        // Start/restart poll timer
        if (m_pollRetransmitTimer.IsPending())
        {
            Simulator::Cancel(m_pollRetransmitTimer);
        }
        m_pollRetransmitTimer = Simulator::Schedule(m_tPollRetransmit,
                                                     &NbIotRlcAm::TPollRetransmitExpiry,
                                                     this);
    }
    
    // Add header
    uint8_t headerBuf[16];
    header.Serialize(headerBuf);
    Ptr<Packet> headerPkt = Create<Packet>(headerBuf, header.GetSerializedSize());
    headerPkt->AddAtEnd(pdu);
    
    // Store for retransmission
    m_txedBuffer[m_vtS] = headerPkt->Copy();
    m_retxCount[m_vtS] = 0;
    
    m_vtS = (m_vtS + 1) % AM_SN_MODULUS;
    
    m_txPduTrace(m_rnti, m_lcid, headerPkt->GetSize());
    
    return headerPkt;
}

uint32_t
NbIotRlcAm::GetTxBufferSize() const
{
    uint32_t size = 0;
    for (const auto& sdu : m_txBuffer)
    {
        size += sdu.packet->GetSize() + 2;
    }
    // Add retx buffer
    for (const auto& [sn, pkt] : m_txedBuffer)
    {
        size += pkt->GetSize();
    }
    return size;
}

NbIotRlcMode
NbIotRlcAm::GetRlcMode() const
{
    return NbIotRlcMode::AM;
}

bool
NbIotRlcAm::IsInsideTxWindow(uint16_t sn) const
{
    return ((sn - m_vtA) % AM_SN_MODULUS) < AM_WINDOW_SIZE;
}

bool
NbIotRlcAm::IsInsideRxWindow(uint16_t sn) const
{
    return ((sn - m_vrR) % AM_SN_MODULUS) < AM_WINDOW_SIZE;
}

void
NbIotRlcAm::ProcessDataPdu(Ptr<Packet> packet, const NbIotRlcAmHeader& header)
{
    NS_LOG_FUNCTION(this);
    
    uint16_t sn = header.GetSequenceNumber();
    
    NS_LOG_DEBUG("AM: Received data PDU SN=" << sn);
    
    if (!IsInsideRxWindow(sn))
    {
        NS_LOG_WARN("AM: Discarding PDU outside window");
        return;
    }
    
    // Store in receive buffer
    NbIotRlcPduSegment segment;
    segment.sn = sn;
    segment.so = header.GetSegmentOffset();
    segment.data = packet->Copy();
    segment.data->RemoveAtStart(header.GetSerializedSize());
    segment.isFirst = ((header.GetFramingInfo() & 0x02) == 0);
    segment.isLast = ((header.GetFramingInfo() & 0x01) == 0);
    
    m_rxBuffer[sn] = segment;
    
    // Update VR(H)
    if (((sn - m_vrH) % AM_SN_MODULUS) < AM_WINDOW_SIZE)
    {
        m_vrH = (sn + 1) % AM_SN_MODULUS;
    }
    
    // Handle polling
    if (header.GetPollingBit())
    {
        m_statusTriggered = true;
    }
    
    // Handle reordering timer
    if (!m_reorderingTimer.IsPending() && m_vrH != m_vrR)
    {
        m_vrX = m_vrH;
        m_reorderingTimer = Simulator::Schedule(m_tReordering,
                                                 &NbIotRlcAm::TReorderingExpiry,
                                                 this);
    }
    
    ReassembleSdu();
}

void
NbIotRlcAm::ProcessStatusPdu(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this);
    
    // Parse status PDU (simplified)
    uint8_t buf[256];
    packet->CopyData(buf, std::min(packet->GetSize(), static_cast<uint32_t>(256)));
    
    // First 2 bytes: D/C=0, CPT=0, ACK_SN
    uint16_t ackSn = ((buf[0] & 0x0F) << 6) | ((buf[1] >> 2) & 0x3F);
    
    NS_LOG_DEBUG("AM: Received status PDU, ACK_SN=" << ackSn);
    
    // Acknowledge all SNs < ackSn
    for (uint16_t sn = m_vtA; sn != ackSn; sn = (sn + 1) % AM_SN_MODULUS)
    {
        m_txedBuffer.erase(sn);
        m_retxCount.erase(sn);
    }
    
    m_vtA = ackSn;
    m_vtMs = (m_vtA + AM_WINDOW_SIZE) % AM_SN_MODULUS;
    
    // Stop poll timer if all acknowledged
    if (m_vtA == m_vtS && m_pollRetransmitTimer.IsPending())
    {
        Simulator::Cancel(m_pollRetransmitTimer);
    }
}

Ptr<Packet>
NbIotRlcAm::BuildStatusPdu()
{
    NS_LOG_FUNCTION(this);
    
    // Build status PDU with ACK_SN = VR(R)
    uint8_t buf[3];
    
    // D/C = 0, CPT = 0
    buf[0] = ((m_vrR >> 6) & 0x0F);
    buf[1] = ((m_vrR & 0x3F) << 2);
    buf[2] = 0;  // No NACK for simplified implementation
    
    Ptr<Packet> statusPdu = Create<Packet>(buf, 3);
    
    NS_LOG_DEBUG("AM: Built status PDU, ACK_SN=" << m_vrR);
    
    return statusPdu;
}

void
NbIotRlcAm::ReassembleSdu()
{
    NS_LOG_FUNCTION(this);
    
    while (m_rxBuffer.find(m_vrR) != m_rxBuffer.end())
    {
        auto& segment = m_rxBuffer[m_vrR];
        
        m_rxSduTrace(m_rnti, m_lcid, segment.data->GetSize());
        
        NS_LOG_DEBUG("AM: Delivered SDU SN=" << m_vrR);
        
        m_rxBuffer.erase(m_vrR);
        m_vrR = (m_vrR + 1) % AM_SN_MODULUS;
        m_vrMr = (m_vrR + AM_WINDOW_SIZE) % AM_SN_MODULUS;
    }
}

void
NbIotRlcAm::TPollRetransmitExpiry()
{
    NS_LOG_FUNCTION(this);
    
    // Retransmit POLL_SN
    if (m_txedBuffer.find(m_pollSn) != m_txedBuffer.end())
    {
        m_retxQueue.push_back(m_pollSn);
    }
}

void
NbIotRlcAm::TReorderingExpiry()
{
    NS_LOG_FUNCTION(this);
    
    // Update VR(MS) and trigger status
    m_vrMs = m_vrX;
    m_statusTriggered = true;
    
    // Update VR(R)
    for (auto it = m_rxBuffer.begin(); it != m_rxBuffer.end();)
    {
        if (((m_vrX - it->first) % AM_SN_MODULUS) <= AM_WINDOW_SIZE &&
            ((m_vrX - it->first) % AM_SN_MODULUS) > 0)
        {
            // Add to NACK list
            m_nackList.insert(it->first);
            ++it;
        }
        else
        {
            ++it;
        }
    }
    
    m_vrR = m_vrX;
    m_vrMr = (m_vrR + AM_WINDOW_SIZE) % AM_SN_MODULUS;
    
    ReassembleSdu();
    
    // Restart timer if needed
    if (m_vrH != m_vrR)
    {
        m_vrX = m_vrH;
        m_reorderingTimer = Simulator::Schedule(m_tReordering,
                                                 &NbIotRlcAm::TReorderingExpiry,
                                                 this);
    }
}

void
NbIotRlcAm::TStatusProhibitExpiry()
{
    NS_LOG_FUNCTION(this);
    // Timer expired, status can now be sent
}

bool
NbIotRlcAm::ShouldPoll()
{
    // Poll conditions per 36.322
    if (m_txBuffer.empty() && m_retxQueue.empty())
    {
        return true;  // Last PDU
    }
    if (m_pduWithoutPoll >= m_pollPdu)
    {
        return true;
    }
    if (m_byteWithoutPoll >= m_pollByte)
    {
        return true;
    }
    return false;
}

void
NbIotRlcAm::HandleNack(uint16_t sn, uint16_t soStart, uint16_t soEnd)
{
    NS_LOG_FUNCTION(this << sn);
    
    if (m_txedBuffer.find(sn) != m_txedBuffer.end())
    {
        // Add to retransmission queue
        auto it = std::find(m_retxQueue.begin(), m_retxQueue.end(), sn);
        if (it == m_retxQueue.end())
        {
            m_retxQueue.push_back(sn);
        }
    }
}

} // namespace ns3
