/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT UE MAC Layer as per 3GPP TS 36.321
 */

#ifndef NBIOT_UE_MAC_H
#define NBIOT_UE_MAC_H

#include "nbiot-mac.h"
#include "nbiot-common.h"

#include <ns3/callback.h>
#include <ns3/event-id.h>

#include <queue>

namespace ns3 {

class NbIotUePhy;
class NbIotUeRrc;

/**
 * \ingroup nbiot
 * \brief Random Access procedure states
 */
enum class NbIotRaState : uint8_t
{
    IDLE,                  ///< Not performing RA
    PREAMBLE_TRANSMISSION, ///< Transmitting NPRACH preamble
    WAIT_RAR,              ///< Waiting for RAR
    CONTENTION_RESOLUTION, ///< Waiting for contention resolution
    CONNECTED              ///< RA completed successfully
};

/**
 * \ingroup nbiot
 * \brief NB-IoT Random Access Manager for UE
 */
class NbIotUeRandomAccessManager : public Object
{
public:
    static TypeId GetTypeId();

    NbIotUeRandomAccessManager();
    ~NbIotUeRandomAccessManager() override;

    void DoDispose() override;

    /**
     * \brief Set the UE MAC
     * \param mac UE MAC layer
     */
    void SetMac(Ptr<class NbIotUeMac> mac);

    /**
     * \brief Start random access procedure
     * \param coverageClass Initial coverage class estimate
     */
    void StartRach(NbIotCoverageClass coverageClass);

    /**
     * \brief Cancel random access procedure
     */
    void CancelRach();

    /**
     * \brief Get current RA state
     * \return RA state
     */
    NbIotRaState GetState() const { return m_state; }

    /**
     * \brief Process received RAR
     * \param rar RAR message
     * \return True if RAR is for this UE
     */
    bool ProcessRar(Ptr<class NbIotRarMessage> rar);

    /**
     * \brief Process contention resolution
     * \param success True if contention resolved successfully
     */
    void ProcessContentionResolution(bool success);

    /**
     * \brief Set RACH configuration
     * \param config RACH configuration
     */
    void SetRachConfig(const NbIotRachConfig& config);

    /**
     * \brief Get current timing advance
     * \return Timing advance value
     */
    uint16_t GetTimingAdvance() const { return m_timingAdvance; }

    /**
     * \brief Set callback for RA completion
     */
    typedef Callback<void, bool, uint16_t> RachCompleteCallback;
    void SetRachCompleteCallback(RachCompleteCallback callback);

private:
    void TransmitPreamble();
    void RarWindowExpired();
    void ApplyBackoff();

    Ptr<NbIotUeMac> m_mac;
    NbIotRaState m_state;
    NbIotRachConfig m_config;
    
    uint8_t m_preambleIndex;
    uint8_t m_preambleTxCount;
    uint16_t m_timingAdvance;
    uint16_t m_tempCRnti;
    NbIotCoverageClass m_coverageClass;
    
    EventId m_rarWindowEvent;
    EventId m_backoffEvent;
    EventId m_preambleTxEvent;
    
    RachCompleteCallback m_rachCompleteCallback;
};

/**
 * \ingroup nbiot
 * \brief NB-IoT BSR Manager for UE
 */
class NbIotBsrManager : public Object
{
public:
    static TypeId GetTypeId();

    NbIotBsrManager();
    ~NbIotBsrManager() override;

    /**
     * \brief Update buffer status for a logical channel group
     * \param lcg Logical channel group
     * \param bufferSize Buffer size in bytes
     */
    void UpdateBufferStatus(uint8_t lcg, uint32_t bufferSize);

    /**
     * \brief Check if BSR is triggered
     * \return True if BSR should be sent
     */
    bool IsBsrTriggered() const;

    /**
     * \brief Get BSR data
     * \param shortBsr Output: true if short BSR, false if long BSR
     * \return Buffer size indices for each LCG
     */
    std::array<uint8_t, 4> GetBsrData(bool& shortBsr) const;

    /**
     * \brief Clear BSR trigger
     */
    void ClearBsrTrigger();

    /**
     * \brief Set periodic BSR timer
     * \param period Timer period
     */
    void SetPeriodicBsrTimer(Time period);

private:
    void PeriodicBsrTimeout();
    uint8_t ConvertBytesToBsrIndex(uint32_t bytes) const;

    std::array<uint32_t, 4> m_bufferSizes;
    bool m_bsrTriggered;
    bool m_regularBsrTriggered;
    bool m_periodicBsrTriggered;
    
    Time m_periodicBsrTimer;
    EventId m_periodicBsrEvent;
};

/**
 * \ingroup nbiot
 * \brief NB-IoT UE MAC layer
 *
 * Implements the UE-side MAC layer for NB-IoT including:
 * - Random access procedure (RACH)
 * - Buffer status reporting (BSR)
 * - HARQ operation
 * - DRX handling
 * - MAC PDU construction and parsing
 */
class NbIotUeMac : public NbIotMac
{
public:
    static TypeId GetTypeId();

    NbIotUeMac();
    ~NbIotUeMac() override;

    void DoDispose() override;

    /**
     * \brief Set the PHY layer
     * \param phy The UE PHY layer
     */
    void SetPhy(Ptr<NbIotUePhy> phy);

    /**
     * \brief Get the PHY layer
     * \return UE PHY layer
     */
    Ptr<NbIotUePhy> GetPhy() const;

    /**
     * \brief Set the RRC layer
     * \param rrc The UE RRC layer
     */
    void SetRrc(Ptr<NbIotUeRrc> rrc);

    /**
     * \brief Get the RRC layer
     * \return UE RRC layer
     */
    Ptr<NbIotUeRrc> GetRrc() const;

    /**
     * \brief Receive MAC PDU from PHY
     * \param packet The MAC PDU
     */
    void ReceivePdu(Ptr<Packet> packet) override;

    /**
     * \brief Transmit MAC SDU from RLC
     * \param lcid Logical channel ID
     * \param packet The SDU
     */
    void TransmitSdu(uint8_t lcid, Ptr<Packet> packet);

    /**
     * \brief Process received DCI (uplink grant or downlink assignment)
     * \param dci DCI message
     */
    void ProcessDci(const NbIotDci& dci);

    /**
     * \brief Start random access
     * \param reason Reason for RA (data, RRC)
     */
    void StartRandomAccess(const std::string& reason = "data");

    /**
     * \brief Get the Random Access Manager
     * \return RA manager
     */
    Ptr<NbIotUeRandomAccessManager> GetRaManager() const;

    /**
     * \brief Get the BSR Manager
     * \return BSR manager
     */
    Ptr<NbIotBsrManager> GetBsrManager() const;

    /**
     * \brief Notify uplink grant received
     * \param dci Uplink grant DCI
     */
    void NotifyUlGrant(const NbIotDci& dci);

    /**
     * \brief Notify downlink assignment received
     * \param dci Downlink assignment DCI
     */
    void NotifyDlAssignment(const NbIotDci& dci);

    /**
     * \brief Update buffer status (called by RLC)
     * \param lcid Logical channel ID
     * \param txQueueSize Transmission queue size
     * \param retxQueueSize Retransmission queue size
     */
    void UpdateBufferStatus(uint8_t lcid, uint32_t txQueueSize, uint32_t retxQueueSize);

    // Callbacks
    typedef Callback<void, uint16_t> RrcConnectionCallback;
    void SetRrcConnectionCallback(RrcConnectionCallback callback);

private:
    /**
     * \brief Build and transmit MAC PDU based on grant
     * \param tbs Transport block size
     */
    void BuildAndTransmitPdu(uint16_t tbs);

    /**
     * \brief RA procedure completed
     * \param success True if successful
     * \param cRnti Assigned C-RNTI
     */
    void RachComplete(bool success, uint16_t cRnti);

    Ptr<NbIotUePhy> m_phy;
    Ptr<NbIotUeRrc> m_rrc;
    Ptr<NbIotUeRandomAccessManager> m_raManager;
    Ptr<NbIotBsrManager> m_bsrManager;

    // Transmit queues per logical channel
    std::map<uint8_t, std::queue<Ptr<Packet>>> m_txQueues;

    // State
    bool m_connected;
    bool m_hasUlGrant;
    NbIotDci m_currentUlGrant;

    // Callbacks
    RrcConnectionCallback m_rrcConnectionCallback;

    // Traced callbacks
    TracedCallback<uint16_t, uint8_t, uint16_t> m_ulGrantTrace; // RNTI, MCS, TBS
    TracedCallback<uint16_t, uint8_t, uint16_t> m_dlAssignmentTrace;
    TracedCallback<NbIotRaState> m_raStateTrace;
};

} // namespace ns3

#endif /* NBIOT_UE_MAC_H */
