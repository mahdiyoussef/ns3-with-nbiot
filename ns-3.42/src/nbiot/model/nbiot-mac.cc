/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT MAC Layer implementation
 */

#include "nbiot-mac.h"

#include <ns3/log.h>
#include <ns3/uinteger.h>
#include <ns3/pointer.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotMac");

// ====================== NbIotMacSubheader ======================

NbIotMacSubheader::NbIotMacSubheader()
    : m_lcid(0)
    , m_length(0)
    , m_extension(false)
    , m_format(false)
{
}

NbIotMacSubheader::NbIotMacSubheader(uint8_t lcid, uint16_t length, bool extension)
    : m_lcid(lcid)
    , m_length(length)
    , m_extension(extension)
    , m_format(length > 127)
{
}

bool
NbIotMacSubheader::IsFixedSize() const
{
    // Fixed-size LCIDs don't have length field
    return m_lcid >= 29 && m_lcid <= 31;
}

uint8_t
NbIotMacSubheader::GetSerializedSize() const
{
    if (IsFixedSize())
    {
        return 1; // R/R/E/LCID only
    }
    else if (m_format)
    {
        return 3; // R/R/E/LCID + F/L (2 bytes)
    }
    else
    {
        return 2; // R/R/E/LCID + F/L (1 byte)
    }
}

void
NbIotMacSubheader::Serialize(uint8_t* buffer) const
{
    // First byte: R/R/E/LCID
    buffer[0] = (m_extension ? 0x20 : 0x00) | (m_lcid & 0x1F);
    
    if (!IsFixedSize())
    {
        if (m_format)
        {
            // Long format: F=1, L=15 bits
            buffer[1] = 0x80 | ((m_length >> 8) & 0x7F);
            buffer[2] = m_length & 0xFF;
        }
        else
        {
            // Short format: F=0, L=7 bits
            buffer[1] = m_length & 0x7F;
        }
    }
}

uint32_t
NbIotMacSubheader::Deserialize(const uint8_t* buffer)
{
    m_extension = (buffer[0] >> 5) & 0x01;
    m_lcid = buffer[0] & 0x1F;
    
    if (IsFixedSize())
    {
        m_length = 0;
        return 1;
    }
    
    m_format = (buffer[1] >> 7) & 0x01;
    
    if (m_format)
    {
        m_length = ((buffer[1] & 0x7F) << 8) | buffer[2];
        return 3;
    }
    else
    {
        m_length = buffer[1] & 0x7F;
        return 2;
    }
}

// ====================== NbIotMacPdu ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotMacPdu);

TypeId
NbIotMacPdu::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotMacPdu")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotMacPdu>();
    return tid;
}

NbIotMacPdu::NbIotMacPdu()
    : m_size(0)
{
}

NbIotMacPdu::~NbIotMacPdu()
{
}

void
NbIotMacPdu::AddSdu(uint8_t lcid, Ptr<Packet> sdu)
{
    NS_LOG_FUNCTION(this << static_cast<int>(lcid) << sdu->GetSize());
    
    uint16_t sduSize = sdu->GetSize();
    bool isLast = true; // Will be updated if more SDUs added
    
    // Update previous subheader extension bit
    if (!m_subheaders.empty())
    {
        m_subheaders.back().SetExtension(true);
    }
    
    NbIotMacSubheader subheader(lcid, sduSize, false);
    m_subheaders.push_back(subheader);
    
    std::vector<uint8_t> payload(sduSize);
    sdu->CopyData(payload.data(), sduSize);
    m_payloads.push_back(payload);
    
    m_size += subheader.GetSerializedSize() + sduSize;
}

void
NbIotMacPdu::AddControlElement(uint8_t lcid, const std::vector<uint8_t>& data)
{
    NS_LOG_FUNCTION(this << static_cast<int>(lcid) << data.size());
    
    if (!m_subheaders.empty())
    {
        m_subheaders.back().SetExtension(true);
    }
    
    NbIotMacSubheader subheader(lcid, data.size(), false);
    m_subheaders.push_back(subheader);
    m_payloads.push_back(data);
    
    m_size += subheader.GetSerializedSize() + data.size();
}

void
NbIotMacPdu::AddPadding(uint16_t targetSize)
{
    NS_LOG_FUNCTION(this << targetSize);
    
    if (m_size >= targetSize)
    {
        return;
    }
    
    uint16_t paddingSize = targetSize - m_size - 1; // 1 byte for padding subheader
    
    if (!m_subheaders.empty())
    {
        m_subheaders.back().SetExtension(true);
    }
    
    NbIotMacSubheader subheader(NbIotMacSubheader::PADDING, 0, false);
    m_subheaders.push_back(subheader);
    
    std::vector<uint8_t> padding(paddingSize, 0);
    m_payloads.push_back(padding);
    
    m_size = targetSize;
}

Ptr<Packet>
NbIotMacPdu::Serialize() const
{
    NS_LOG_FUNCTION(this);
    
    std::vector<uint8_t> pduData;
    pduData.reserve(m_size);
    
    // Serialize subheaders
    for (const auto& subheader : m_subheaders)
    {
        uint8_t buffer[3];
        subheader.Serialize(buffer);
        for (uint8_t i = 0; i < subheader.GetSerializedSize(); ++i)
        {
            pduData.push_back(buffer[i]);
        }
    }
    
    // Serialize payloads
    for (const auto& payload : m_payloads)
    {
        pduData.insert(pduData.end(), payload.begin(), payload.end());
    }
    
    Ptr<Packet> pdu = Create<Packet>(pduData.data(), pduData.size());
    return pdu;
}

std::map<uint8_t, std::vector<Ptr<Packet>>>
NbIotMacPdu::Deserialize(Ptr<Packet> pdu)
{
    NS_LOG_FUNCTION(pdu);
    
    std::map<uint8_t, std::vector<Ptr<Packet>>> result;
    
    uint32_t pduSize = pdu->GetSize();
    std::vector<uint8_t> buffer(pduSize);
    pdu->CopyData(buffer.data(), pduSize);
    
    // Parse subheaders
    std::vector<NbIotMacSubheader> subheaders;
    uint32_t offset = 0;
    
    while (offset < pduSize)
    {
        NbIotMacSubheader subheader;
        offset += subheader.Deserialize(&buffer[offset]);
        subheaders.push_back(subheader);
        
        if (!subheader.HasExtension())
        {
            break;
        }
    }
    
    // Parse payloads
    for (const auto& subheader : subheaders)
    {
        if (subheader.GetLcid() == NbIotMacSubheader::PADDING)
        {
            break; // Rest is padding
        }
        
        uint16_t length = subheader.GetLength();
        if (subheader.IsFixedSize())
        {
            // Determine fixed size based on LCID
            switch (subheader.GetLcid())
            {
                case NbIotMacSubheader::SHORT_BSR:
                    length = 1;
                    break;
                case NbIotMacSubheader::LONG_BSR:
                    length = 3;
                    break;
                default:
                    length = 0;
            }
        }
        
        if (offset + length <= pduSize)
        {
            Ptr<Packet> sdu = Create<Packet>(&buffer[offset], length);
            result[subheader.GetLcid()].push_back(sdu);
            offset += length;
        }
    }
    
    return result;
}

uint16_t
NbIotMacPdu::GetSize() const
{
    return m_size;
}

// ====================== NbIotHarqEntity ======================

NbIotHarqEntity::NbIotHarqEntity()
    : m_active(false)
    , m_txCount(0)
    , m_ndi(false)
    , m_tbs(0)
    , m_repetitions(1)
{
}

void
NbIotHarqEntity::Reset()
{
    m_active = false;
    m_txCount = 0;
    m_ndi = false;
    m_tbs = 0;
    m_repetitions = 1;
    m_packet = nullptr;
}

// ====================== NbIotHarqManager ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotHarqManager);

TypeId
NbIotHarqManager::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotHarqManager")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotHarqManager>()
        .AddAttribute("NumProcesses",
                      "Number of HARQ processes",
                      UintegerValue(2),
                      MakeUintegerAccessor(&NbIotHarqManager::m_numProcesses),
                      MakeUintegerChecker<uint8_t>(1, 8))
        .AddAttribute("MaxTransmissions",
                      "Maximum number of HARQ transmissions",
                      UintegerValue(8),
                      MakeUintegerAccessor(&NbIotHarqManager::m_maxTransmissions),
                      MakeUintegerChecker<uint8_t>(1, 16));
    return tid;
}

NbIotHarqManager::NbIotHarqManager()
    : m_numProcesses(2)
    , m_maxTransmissions(8)
{
    m_processes.resize(m_numProcesses);
}

NbIotHarqManager::~NbIotHarqManager()
{
}

void
NbIotHarqManager::SetNumProcesses(uint8_t num)
{
    NS_LOG_FUNCTION(this << static_cast<int>(num));
    m_numProcesses = num;
    m_processes.resize(num);
}

int8_t
NbIotHarqManager::GetAvailableProcess()
{
    for (uint8_t i = 0; i < m_numProcesses; ++i)
    {
        if (!m_processes[i].IsActive())
        {
            return i;
        }
    }
    return -1;
}

NbIotHarqEntity*
NbIotHarqManager::GetProcess(uint8_t processId)
{
    if (processId < m_numProcesses)
    {
        return &m_processes[processId];
    }
    return nullptr;
}

void
NbIotHarqManager::ProcessAck(uint8_t processId)
{
    NS_LOG_FUNCTION(this << static_cast<int>(processId));
    
    if (processId < m_numProcesses)
    {
        m_processes[processId].Reset();
        NS_LOG_INFO("HARQ process " << static_cast<int>(processId) << " ACKed and reset");
    }
}

bool
NbIotHarqManager::ProcessNack(uint8_t processId)
{
    NS_LOG_FUNCTION(this << static_cast<int>(processId));
    
    if (processId >= m_numProcesses)
    {
        return false;
    }
    
    auto& process = m_processes[processId];
    process.IncrementTxCount();
    
    if (process.GetTxCount() >= m_maxTransmissions)
    {
        NS_LOG_INFO("HARQ process " << static_cast<int>(processId)
                    << " reached max transmissions, resetting");
        process.Reset();
        return false;
    }
    
    NS_LOG_INFO("HARQ process " << static_cast<int>(processId)
                << " NACKed, retransmission " << static_cast<int>(process.GetTxCount()));
    return true;
}

void
NbIotHarqManager::ResetProcess(uint8_t processId)
{
    NS_LOG_FUNCTION(this << static_cast<int>(processId));
    
    if (processId < m_numProcesses)
    {
        m_processes[processId].Reset();
    }
}

// ====================== NbIotMac ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotMac);

TypeId
NbIotMac::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotMac")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddTraceSource("MacTx",
                        "Trace fired when transmitting MAC PDU",
                        MakeTraceSourceAccessor(&NbIotMac::m_macTxTrace),
                        "ns3::NbIotMac::MacTxTracedCallback")
        .AddTraceSource("MacRx",
                        "Trace fired when receiving MAC PDU",
                        MakeTraceSourceAccessor(&NbIotMac::m_macRxTrace),
                        "ns3::NbIotMac::MacRxTracedCallback");
    return tid;
}

NbIotMac::NbIotMac()
    : m_rnti(0)
{
    NS_LOG_FUNCTION(this);
    m_harqManager = CreateObject<NbIotHarqManager>();
}

NbIotMac::~NbIotMac()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotMac::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_harqManager = nullptr;
    Object::DoDispose();
}

void
NbIotMac::SetRnti(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    m_rnti = rnti;
}

uint16_t
NbIotMac::GetRnti() const
{
    return m_rnti;
}

Ptr<NbIotHarqManager>
NbIotMac::GetHarqManager() const
{
    return m_harqManager;
}

} // namespace ns3
