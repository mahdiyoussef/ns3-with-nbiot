/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Latency Measurement Tag - Used for tracking packet latency
 * from MAC layer origination to MAC layer reception
 */

#ifndef NBIOT_LATENCY_TAG_H
#define NBIOT_LATENCY_TAG_H

#include <ns3/tag.h>
#include <ns3/nstime.h>
#include <ns3/simulator.h>

namespace ns3 {

/**
 * \ingroup nbiot
 * \brief Tag to track latency measurements for NB-IoT packets
 *
 * This tag is attached to packets at the UE MAC layer when data becomes
 * ready for transmission, and the timestamp is read at the eNB MAC layer
 * when data is successfully received to compute end-to-end MAC latency.
 */
class NbIotLatencyTag : public Tag
{
public:
    /**
     * \brief Get the type ID
     * \return The TypeId for this class
     */
    static TypeId GetTypeId();
    
    /**
     * \brief Default constructor
     */
    NbIotLatencyTag();
    
    /**
     * \brief Constructor with timestamp
     * \param timestamp The time when the packet was generated/queued
     */
    explicit NbIotLatencyTag(Time timestamp);
    
    // Inherited from Tag
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;
    
    /**
     * \brief Set the timestamp
     * \param timestamp The time to set
     */
    void SetTimestamp(Time timestamp);
    
    /**
     * \brief Get the timestamp
     * \return The stored timestamp
     */
    Time GetTimestamp() const;
    
    /**
     * \brief Calculate latency from stored timestamp to now
     * \return The latency (current time - stored timestamp)
     */
    Time GetLatency() const;
    
    /**
     * \brief Set the UE RNTI
     * \param rnti The RNTI of the UE
     */
    void SetRnti(uint16_t rnti);
    
    /**
     * \brief Get the UE RNTI
     * \return The RNTI
     */
    uint16_t GetRnti() const;
    
    /**
     * \brief Set the sequence number
     * \param seqNum Sequence number for tracking
     */
    void SetSequenceNumber(uint32_t seqNum);
    
    /**
     * \brief Get the sequence number
     * \return The sequence number
     */
    uint32_t GetSequenceNumber() const;
    
    /**
     * \brief Set whether this is an SPS transmission
     * \param isSps True if SPS, false if dynamic scheduling
     */
    void SetIsSps(bool isSps);
    
    /**
     * \brief Check if this is an SPS transmission
     * \return True if SPS, false otherwise
     */
    bool IsSps() const;

private:
    Time m_timestamp;        ///< Timestamp when packet was generated
    uint16_t m_rnti;         ///< UE RNTI
    uint32_t m_sequenceNum;  ///< Sequence number for tracking
    bool m_isSps;            ///< Whether this uses SPS
};


/**
 * \ingroup nbiot
 * \brief Latency statistics collector for NB-IoT simulations
 *
 * Collects and aggregates latency measurements for analysis
 */
class NbIotLatencyStats
{
public:
    /**
     * \brief Record a latency measurement
     * \param rnti The UE RNTI
     * \param latency The measured latency
     * \param isSps Whether this was an SPS transmission
     * \param seqNum The packet sequence number
     */
    void RecordLatency(uint16_t rnti, Time latency, bool isSps, uint32_t seqNum);
    
    /**
     * \brief Get average latency for all packets
     * \return Average latency
     */
    Time GetAverageLatency() const;
    
    /**
     * \brief Get average latency for SPS packets only
     * \return Average SPS latency
     */
    Time GetAverageSpsLatency() const;
    
    /**
     * \brief Get average latency for dynamic scheduling packets
     * \return Average DS latency
     */
    Time GetAverageDsLatency() const;
    
    /**
     * \brief Get minimum latency
     * \return Minimum latency observed
     */
    Time GetMinLatency() const;
    
    /**
     * \brief Get maximum latency
     * \return Maximum latency observed
     */
    Time GetMaxLatency() const;
    
    /**
     * \brief Get the 95th percentile latency
     * \return 95th percentile latency
     */
    Time Get95thPercentileLatency() const;
    
    /**
     * \brief Get total number of packets recorded
     * \return Packet count
     */
    uint32_t GetPacketCount() const;
    
    /**
     * \brief Get SPS packet count
     * \return SPS packet count
     */
    uint32_t GetSpsPacketCount() const;
    
    /**
     * \brief Get dynamic scheduling packet count
     * \return DS packet count
     */
    uint32_t GetDsPacketCount() const;
    
    /**
     * \brief Get standard deviation of latency
     * \return Standard deviation
     */
    Time GetStdDevLatency() const;
    
    /**
     * \brief Reset all statistics
     */
    void Reset();
    
    /**
     * \brief Export statistics to CSV format
     * \param filename Output filename
     */
    void ExportToCsv(const std::string& filename) const;
    
    /**
     * \brief Get per-UE latency statistics
     * \param rnti The UE RNTI
     * \param avgLatency Output average latency for UE
     * \param count Output packet count for UE
     * \return True if UE found, false otherwise
     */
    bool GetUeStats(uint16_t rnti, Time& avgLatency, uint32_t& count) const;

private:
    /**
     * \brief Individual latency record
     */
    struct LatencyRecord
    {
        uint16_t rnti;
        Time latency;
        Time timestamp;
        bool isSps;
        uint32_t seqNum;
    };
    
    std::vector<LatencyRecord> m_records;  ///< All recorded latencies
    
    // Cached aggregate values
    mutable bool m_cacheValid = false;
    mutable Time m_cachedAvg;
    mutable Time m_cachedSpsAvg;
    mutable Time m_cachedDsAvg;
    mutable Time m_cachedMin;
    mutable Time m_cachedMax;
    
    void InvalidateCache();
    void ComputeCache() const;
};

} // namespace ns3

#endif // NBIOT_LATENCY_TAG_H
