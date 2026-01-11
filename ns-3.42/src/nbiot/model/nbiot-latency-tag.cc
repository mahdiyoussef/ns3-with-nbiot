/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Latency Measurement Tag Implementation
 */

#include "nbiot-latency-tag.h"

#include <ns3/log.h>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <numeric>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotLatencyTag");

NS_OBJECT_ENSURE_REGISTERED(NbIotLatencyTag);

TypeId
NbIotLatencyTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotLatencyTag")
        .SetParent<Tag>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotLatencyTag>();
    return tid;
}

NbIotLatencyTag::NbIotLatencyTag()
    : m_timestamp(Seconds(0))
    , m_rnti(0)
    , m_sequenceNum(0)
    , m_isSps(false)
{
}

NbIotLatencyTag::NbIotLatencyTag(Time timestamp)
    : m_timestamp(timestamp)
    , m_rnti(0)
    , m_sequenceNum(0)
    , m_isSps(false)
{
}

TypeId
NbIotLatencyTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
NbIotLatencyTag::GetSerializedSize() const
{
    // 8 bytes timestamp + 2 bytes RNTI + 4 bytes seqNum + 1 byte flags
    return 15;
}

void
NbIotLatencyTag::Serialize(TagBuffer i) const
{
    int64_t ns = m_timestamp.GetNanoSeconds();
    i.WriteU64(static_cast<uint64_t>(ns));
    i.WriteU16(m_rnti);
    i.WriteU32(m_sequenceNum);
    i.WriteU8(m_isSps ? 1 : 0);
}

void
NbIotLatencyTag::Deserialize(TagBuffer i)
{
    uint64_t ns = i.ReadU64();
    m_timestamp = NanoSeconds(static_cast<int64_t>(ns));
    m_rnti = i.ReadU16();
    m_sequenceNum = i.ReadU32();
    m_isSps = (i.ReadU8() == 1);
}

void
NbIotLatencyTag::Print(std::ostream& os) const
{
    os << "NbIotLatencyTag["
       << "rnti=" << m_rnti
       << ", seq=" << m_sequenceNum
       << ", ts=" << m_timestamp.GetMilliSeconds() << "ms"
       << ", sps=" << (m_isSps ? "yes" : "no")
       << "]";
}

void
NbIotLatencyTag::SetTimestamp(Time timestamp)
{
    m_timestamp = timestamp;
}

Time
NbIotLatencyTag::GetTimestamp() const
{
    return m_timestamp;
}

Time
NbIotLatencyTag::GetLatency() const
{
    return Simulator::Now() - m_timestamp;
}

void
NbIotLatencyTag::SetRnti(uint16_t rnti)
{
    m_rnti = rnti;
}

uint16_t
NbIotLatencyTag::GetRnti() const
{
    return m_rnti;
}

void
NbIotLatencyTag::SetSequenceNumber(uint32_t seqNum)
{
    m_sequenceNum = seqNum;
}

uint32_t
NbIotLatencyTag::GetSequenceNumber() const
{
    return m_sequenceNum;
}

void
NbIotLatencyTag::SetIsSps(bool isSps)
{
    m_isSps = isSps;
}

bool
NbIotLatencyTag::IsSps() const
{
    return m_isSps;
}

//=============================================================================
// NbIotLatencyStats Implementation
//=============================================================================

void
NbIotLatencyStats::RecordLatency(uint16_t rnti, Time latency, bool isSps, uint32_t seqNum)
{
    LatencyRecord record;
    record.rnti = rnti;
    record.latency = latency;
    record.timestamp = Simulator::Now();
    record.isSps = isSps;
    record.seqNum = seqNum;
    
    m_records.push_back(record);
    InvalidateCache();
    
    NS_LOG_DEBUG("Recorded latency: RNTI=" << rnti 
                 << ", latency=" << latency.GetMilliSeconds() << "ms"
                 << ", SPS=" << (isSps ? "yes" : "no"));
}

void
NbIotLatencyStats::InvalidateCache()
{
    m_cacheValid = false;
}

void
NbIotLatencyStats::ComputeCache() const
{
    if (m_cacheValid || m_records.empty())
    {
        return;
    }
    
    // Initialize
    m_cachedMin = Seconds(1000000);
    m_cachedMax = Seconds(0);
    
    double totalNs = 0;
    double spsTotalNs = 0;
    double dsTotalNs = 0;
    uint32_t spsCount = 0;
    uint32_t dsCount = 0;
    
    for (const auto& record : m_records)
    {
        double ns = record.latency.GetNanoSeconds();
        totalNs += ns;
        
        if (record.isSps)
        {
            spsTotalNs += ns;
            spsCount++;
        }
        else
        {
            dsTotalNs += ns;
            dsCount++;
        }
        
        if (record.latency < m_cachedMin)
        {
            m_cachedMin = record.latency;
        }
        if (record.latency > m_cachedMax)
        {
            m_cachedMax = record.latency;
        }
    }
    
    m_cachedAvg = NanoSeconds(static_cast<int64_t>(totalNs / m_records.size()));
    m_cachedSpsAvg = spsCount > 0 ? NanoSeconds(static_cast<int64_t>(spsTotalNs / spsCount)) : Seconds(0);
    m_cachedDsAvg = dsCount > 0 ? NanoSeconds(static_cast<int64_t>(dsTotalNs / dsCount)) : Seconds(0);
    
    m_cacheValid = true;
}

Time
NbIotLatencyStats::GetAverageLatency() const
{
    ComputeCache();
    return m_cachedAvg;
}

Time
NbIotLatencyStats::GetAverageSpsLatency() const
{
    ComputeCache();
    return m_cachedSpsAvg;
}

Time
NbIotLatencyStats::GetAverageDsLatency() const
{
    ComputeCache();
    return m_cachedDsAvg;
}

Time
NbIotLatencyStats::GetMinLatency() const
{
    ComputeCache();
    return m_cachedMin;
}

Time
NbIotLatencyStats::GetMaxLatency() const
{
    ComputeCache();
    return m_cachedMax;
}

Time
NbIotLatencyStats::Get95thPercentileLatency() const
{
    if (m_records.empty())
    {
        return Seconds(0);
    }
    
    std::vector<int64_t> latencies;
    latencies.reserve(m_records.size());
    
    for (const auto& record : m_records)
    {
        latencies.push_back(record.latency.GetNanoSeconds());
    }
    
    std::sort(latencies.begin(), latencies.end());
    
    size_t idx = static_cast<size_t>(0.95 * latencies.size());
    if (idx >= latencies.size())
    {
        idx = latencies.size() - 1;
    }
    
    return NanoSeconds(latencies[idx]);
}

uint32_t
NbIotLatencyStats::GetPacketCount() const
{
    return static_cast<uint32_t>(m_records.size());
}

uint32_t
NbIotLatencyStats::GetSpsPacketCount() const
{
    uint32_t count = 0;
    for (const auto& record : m_records)
    {
        if (record.isSps)
        {
            count++;
        }
    }
    return count;
}

uint32_t
NbIotLatencyStats::GetDsPacketCount() const
{
    uint32_t count = 0;
    for (const auto& record : m_records)
    {
        if (!record.isSps)
        {
            count++;
        }
    }
    return count;
}

Time
NbIotLatencyStats::GetStdDevLatency() const
{
    if (m_records.size() < 2)
    {
        return Seconds(0);
    }
    
    ComputeCache();
    double avgNs = m_cachedAvg.GetNanoSeconds();
    double sumSqDiff = 0;
    
    for (const auto& record : m_records)
    {
        double diff = record.latency.GetNanoSeconds() - avgNs;
        sumSqDiff += diff * diff;
    }
    
    double variance = sumSqDiff / (m_records.size() - 1);
    return NanoSeconds(static_cast<int64_t>(std::sqrt(variance)));
}

void
NbIotLatencyStats::Reset()
{
    m_records.clear();
    InvalidateCache();
}

void
NbIotLatencyStats::ExportToCsv(const std::string& filename) const
{
    std::ofstream file(filename);
    
    if (!file.is_open())
    {
        NS_LOG_ERROR("Failed to open file: " << filename);
        return;
    }
    
    // Header
    file << "timestamp_ms,rnti,sequence_number,latency_ms,scheduling_type\n";
    
    // Data rows
    for (const auto& record : m_records)
    {
        file << record.timestamp.GetMilliSeconds() << ","
             << record.rnti << ","
             << record.seqNum << ","
             << record.latency.GetMilliSeconds() << ","
             << (record.isSps ? "SPS" : "DS") << "\n";
    }
    
    file.close();
    
    NS_LOG_INFO("Exported " << m_records.size() << " latency records to " << filename);
}

bool
NbIotLatencyStats::GetUeStats(uint16_t rnti, Time& avgLatency, uint32_t& count) const
{
    count = 0;
    double totalNs = 0;
    
    for (const auto& record : m_records)
    {
        if (record.rnti == rnti)
        {
            totalNs += record.latency.GetNanoSeconds();
            count++;
        }
    }
    
    if (count == 0)
    {
        avgLatency = Seconds(0);
        return false;
    }
    
    avgLatency = NanoSeconds(static_cast<int64_t>(totalNs / count));
    return true;
}

} // namespace ns3
