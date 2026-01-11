/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT eNB MAC Layer Header
 *
 * References:
 * - 3GPP TS 36.321 MAC protocol specification
 * - 3GPP TS 36.213 Physical layer procedures
 */

#ifndef NBIOT_ENB_MAC_H
#define NBIOT_ENB_MAC_H

#include "nbiot-mac.h"
#include "nbiot-common.h"
#include "nbiot-control-messages.h"

#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/simple-ref-count.h>
#include <ns3/callback.h>
#include <ns3/traced-callback.h>
#include <ns3/event-id.h>

#include <vector>
#include <map>
#include <queue>
#include <memory>
#include <functional>

namespace ns3 {

class NbIotEnbPhy;
class NbIotEnbRrc;
class NbIotScheduler;
class NbIotEnbMac;  // Forward declaration

/**
 * \ingroup nbiot
 *
 * \brief UE context maintained by eNB MAC
 */
struct NbIotUeContext : public SimpleRefCount<NbIotUeContext>
{
    uint16_t rnti;                                  ///< UE identifier
    NbIotCoverageClass coverageClass;               ///< Current coverage enhancement level
    uint8_t cqi;                                    ///< Channel quality indicator
    std::array<uint32_t, 4> bsrBuffer;              ///< Buffer status for each LCG
    uint32_t totalBufferSize;                       ///< Total pending bytes
    Time lastActivity;                              ///< Time of last activity
    bool srReceived;                                ///< Scheduling request pending
    bool connected;                                 ///< RRC connected state
    
    NbIotUeContext()
        : rnti(0)
        , coverageClass(NbIotCoverageClass::CE_LEVEL_0)
        , cqi(7)
        , totalBufferSize(0)
        , lastActivity(Seconds(0))
        , srReceived(false)
        , connected(false)
    {
        bsrBuffer.fill(0);
    }
};

/**
 * \ingroup nbiot
 *
 * \brief Scheduling request information
 */
struct NbIotSchedulingRequest
{
    uint16_t rnti;
    NbIotCoverageClass coverageClass;
    uint8_t priority;
    Time requestTime;
};

/**
 * \ingroup nbiot
 *
 * \brief Uplink grant result from scheduler
 */
struct NbIotUlGrantResult
{
    uint16_t rnti;
    uint8_t mcs;
    uint16_t tbSize;
    uint8_t numSubcarriers;
    uint8_t numSlots;
    uint8_t repetitions;
    uint8_t rv;                 ///< Redundancy version
    bool newData;               ///< New data indicator
    uint8_t harqProcess;
};

/**
 * \ingroup nbiot
 *
 * \brief Downlink assignment result from scheduler
 */
struct NbIotDlAssignmentResult
{
    uint16_t rnti;
    uint8_t mcs;
    uint16_t tbSize;
    uint8_t numSubframes;
    uint8_t repetitions;
    uint8_t rv;
    bool newData;
    uint8_t harqProcess;
    std::vector<Ptr<Packet>> sdus;  ///< SDUs to transmit
};

/**
 * \ingroup nbiot
 *
 * \brief Abstract base class for NB-IoT schedulers
 */
class NbIotScheduler : public Object
{
public:
    static TypeId GetTypeId();
    
    NbIotScheduler();
    virtual ~NbIotScheduler();
    
    /**
     * \brief Set the MAC layer
     * \param mac Pointer to eNB MAC
     */
    void SetMac(Ptr<NbIotEnbMac> mac);
    
    /**
     * \brief Schedule downlink data transmissions
     * \param availableRbs Number of available resource blocks
     * \return Vector of DL assignments
     */
    virtual std::vector<NbIotDlAssignmentResult> ScheduleDl(uint8_t availableRbs) = 0;
    
    /**
     * \brief Schedule uplink grants
     * \param availableRbs Number of available resource blocks
     * \return Vector of UL grants
     */
    virtual std::vector<NbIotUlGrantResult> ScheduleUl(uint8_t availableRbs) = 0;
    
    /**
     * \brief Get UE context for scheduling
     * \param rnti UE identifier
     * \return Pointer to UE context
     */
    Ptr<NbIotUeContext> GetUeContext(uint16_t rnti);
    
protected:
    void DoDispose() override;
    
    Ptr<NbIotEnbMac> m_mac;                             ///< Associated MAC layer
    std::map<uint16_t, Ptr<NbIotUeContext>> m_ueContexts;   ///< UE contexts
};

/**
 * \ingroup nbiot
 *
 * \brief Round-robin scheduler for NB-IoT
 */
class NbIotRoundRobinScheduler : public NbIotScheduler
{
public:
    static TypeId GetTypeId();
    
    NbIotRoundRobinScheduler();
    ~NbIotRoundRobinScheduler() override;
    
    std::vector<NbIotDlAssignmentResult> ScheduleDl(uint8_t availableRbs) override;
    std::vector<NbIotUlGrantResult> ScheduleUl(uint8_t availableRbs) override;
    
private:
    uint16_t m_lastDlRnti;      ///< Last scheduled DL RNTI
    uint16_t m_lastUlRnti;      ///< Last scheduled UL RNTI
    
    /**
     * \brief Get next UE in round-robin order
     * \param lastRnti Previous scheduled RNTI
     * \param dl True for DL, false for UL
     * \return Next RNTI or 0 if none available
     */
    uint16_t GetNextRnti(uint16_t lastRnti, bool dl);
};

/**
 * \ingroup nbiot
 *
 * \brief Coverage-class based scheduler for NB-IoT
 *
 * Prioritizes UEs based on coverage class and QoS requirements
 */
class NbIotCoverageClassScheduler : public NbIotScheduler
{
public:
    static TypeId GetTypeId();
    
    NbIotCoverageClassScheduler();
    ~NbIotCoverageClassScheduler() override;
    
    std::vector<NbIotDlAssignmentResult> ScheduleDl(uint8_t availableRbs) override;
    std::vector<NbIotUlGrantResult> ScheduleUl(uint8_t availableRbs) override;
    
private:
    /**
     * \brief Get priority based on coverage class
     * \param ceLevel Coverage enhancement level
     * \return Priority value (lower is higher priority)
     */
    uint8_t GetPriority(NbIotCoverageClass ceLevel);
    
    /**
     * \brief Sort UEs by priority
     * \param ueList List of UE RNTIs
     * \return Sorted list by priority
     */
    std::vector<uint16_t> SortByPriority(const std::vector<uint16_t>& ueList);
};

/**
 * \ingroup nbiot
 *
 * \brief NB-IoT eNB MAC layer
 *
 * Implements the MAC protocol for eNB side including:
 * - Scheduling decisions
 * - HARQ management
 * - Random access response generation
 * - UE context management
 */
class NbIotEnbMac : public NbIotMac
{
public:
    static TypeId GetTypeId();
    
    NbIotEnbMac();
    ~NbIotEnbMac() override;
    
    /**
     * \brief Set the PHY layer
     * \param phy Pointer to eNB PHY
     */
    void SetPhy(Ptr<NbIotEnbPhy> phy);
    
    /**
     * \brief Get the PHY layer
     * \return Pointer to eNB PHY
     */
    Ptr<NbIotEnbPhy> GetPhy() const;
    
    /**
     * \brief Set the RRC layer
     * \param rrc Pointer to eNB RRC
     */
    void SetRrc(Ptr<NbIotEnbRrc> rrc);
    
    /**
     * \brief Set the scheduler
     * \param scheduler Pointer to scheduler implementation
     */
    void SetScheduler(Ptr<NbIotScheduler> scheduler);
    
    /**
     * \brief Get the scheduler
     * \return Pointer to scheduler
     */
    Ptr<NbIotScheduler> GetScheduler() const;
    
    /**
     * \brief Receive MAC PDU (base class interface - forwards to ReceivePdu with unknown RNTI)
     * \param packet MAC PDU
     */
    void ReceivePdu(Ptr<Packet> packet) override;
    
    /**
     * \brief Process received MAC PDU from UE
     * \param rnti UE identifier
     * \param packet MAC PDU
     */
    void ReceivePdu(uint16_t rnti, Ptr<Packet> packet);
    
    /**
     * \brief Transmit data to a UE
     * \param rnti UE identifier
     * \param lcid Logical channel ID
     * \param packet Data packet
     */
    void TransmitSdu(uint16_t rnti, uint8_t lcid, Ptr<Packet> packet);
    
    /**
     * \brief Process preamble received from PHY
     * \param preambleId Preamble index
     * \param timingAdvance Measured timing advance
     * \param coverageClass Detected coverage class
     */
    void ProcessReceivedPreamble(uint8_t preambleId, uint16_t timingAdvance,
                                 NbIotCoverageClass coverageClass);
    
    /**
     * \brief Process HARQ feedback from UE
     * \param rnti UE identifier
     * \param harqId HARQ process ID
     * \param ack ACK/NACK indication
     */
    void ProcessHarqFeedback(uint16_t rnti, uint8_t harqId, bool ack);
    
    /**
     * \brief Process BSR received from UE
     * \param rnti UE identifier
     * \param bsr BSR message
     */
    void ProcessBsr(uint16_t rnti, Ptr<NbIotBsrMessage> bsr);
    
    /**
     * \brief Process scheduling request
     * \param rnti UE identifier
     */
    void ProcessSchedulingRequest(uint16_t rnti);
    
    /**
     * \brief Allocate a new RNTI
     * \return Allocated RNTI
     */
    uint16_t AllocateRnti();
    
    /**
     * \brief Add a new UE
     * \param rnti UE identifier
     * \param coverageClass Coverage enhancement level
     */
    void AddUe(uint16_t rnti, NbIotCoverageClass coverageClass);
    
    /**
     * \brief Remove a UE
     * \param rnti UE identifier
     */
    void RemoveUe(uint16_t rnti);
    
    /**
     * \brief Get UE context
     * \param rnti UE identifier
     * \return Pointer to UE context or nullptr
     */
    Ptr<NbIotUeContext> GetUeContext(uint16_t rnti);
    
    /**
     * \brief Get all active UEs
     * \return Map of UE contexts
     */
    std::map<uint16_t, Ptr<NbIotUeContext>>& GetUeContexts();
    
    /**
     * \brief Trigger subframe indication
     *
     * Called by PHY at each subframe to trigger scheduling
     */
    void SubframeIndication();
    
    /**
     * \brief Get DL transmit queue for a UE
     * \param rnti UE identifier
     * \return Reference to transmit queue
     */
    std::queue<std::pair<uint8_t, Ptr<Packet>>>& GetDlTxQueue(uint16_t rnti);
    
protected:
    void DoDispose() override;
    
private:
    /**
     * \brief Generate random access response
     * \param preambleId Received preamble
     * \param timingAdvance Timing advance value
     * \param coverageClass Coverage class
     */
    void GenerateRar(uint8_t preambleId, uint16_t timingAdvance,
                     NbIotCoverageClass coverageClass);
    
    /**
     * \brief Execute scheduling decisions
     */
    void DoSchedule();
    
    /**
     * \brief Build and transmit DL MAC PDUs
     * \param assignments DL assignment results
     */
    void TransmitDlPdus(const std::vector<NbIotDlAssignmentResult>& assignments);
    
    /**
     * \brief Send UL grants via NPDCCH
     * \param grants UL grant results
     */
    void SendUlGrants(const std::vector<NbIotUlGrantResult>& grants);
    
    Ptr<NbIotEnbPhy> m_phy;                             ///< PHY layer
    Ptr<NbIotEnbRrc> m_rrc;                             ///< RRC layer
    Ptr<NbIotScheduler> m_scheduler;                    ///< Scheduler
    
    std::map<uint16_t, Ptr<NbIotUeContext>> m_ueContexts;   ///< UE contexts
    std::map<uint16_t, std::queue<std::pair<uint8_t, Ptr<Packet>>>> m_dlTxQueues; ///< DL TX queues per UE
    std::map<uint16_t, Ptr<NbIotHarqManager>> m_dlHarqManagers;  ///< DL HARQ managers per UE
    std::map<uint16_t, Ptr<NbIotHarqManager>> m_ulHarqManagers;  ///< UL HARQ managers per UE
    
    std::queue<NbIotSchedulingRequest> m_pendingRaRequests;     ///< Pending RA requests
    
    uint16_t m_nextRnti;                                ///< Next RNTI to allocate
    Time m_rarDelay;                                    ///< RAR scheduling delay
    
    /// Traces
    TracedCallback<uint16_t, uint8_t, uint16_t> m_ulScheduleTrace;  ///< UL schedule trace
    TracedCallback<uint16_t, uint8_t, uint16_t> m_dlScheduleTrace;  ///< DL schedule trace
    TracedCallback<uint8_t, uint16_t> m_rarTrace;                   ///< RAR generation trace
};

} // namespace ns3

#endif /* NBIOT_ENB_MAC_H */
