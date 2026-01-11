/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT RLC Layer Header
 *
 * References:
 * - 3GPP TS 36.322 RLC protocol specification
 */

#ifndef NBIOT_RLC_H
#define NBIOT_RLC_H

#include "nbiot-common.h"

#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/packet.h>
#include <ns3/traced-callback.h>
#include <ns3/event-id.h>
#include <ns3/nstime.h>

#include <deque>
#include <map>
#include <set>

namespace ns3 {

class NbIotMac;
class NbIotPdcp;

// NbIotRlcMode is defined in nbiot-common.h

/**
 * \ingroup nbiot
 *
 * \brief RLC PDU segment information
 */
struct NbIotRlcPduSegment
{
    uint16_t sn;            ///< Sequence number
    uint16_t so;            ///< Segment offset
    uint16_t size;          ///< Segment size
    Ptr<Packet> data;       ///< Segment data
    bool isFirst;           ///< First segment flag
    bool isLast;            ///< Last segment flag
};

/**
 * \ingroup nbiot
 *
 * \brief RLC SDU information
 */
struct NbIotRlcSduInfo
{
    Ptr<Packet> packet;     ///< SDU data
    Time arrivalTime;       ///< Arrival time for delay calculation
};

/**
 * \ingroup nbiot
 *
 * \brief RLC AM PDU status report
 */
struct NbIotRlcStatusPdu
{
    uint16_t ackSn;                         ///< ACK SN - all SNs < ackSn are acknowledged
    std::vector<std::pair<uint16_t, uint16_t>> nackList;  ///< NACK list (SN, SO pairs)
};

/**
 * \ingroup nbiot
 *
 * \brief RLC PDU header for UM mode
 */
class NbIotRlcUmHeader
{
public:
    NbIotRlcUmHeader();
    
    void SetSequenceNumber(uint16_t sn);
    uint16_t GetSequenceNumber() const;
    
    void SetFramingInfo(uint8_t fi);
    uint8_t GetFramingInfo() const;
    
    void SetExtensionBit(bool e);
    bool GetExtensionBit() const;
    
    void AddLengthIndicator(uint16_t li);
    std::vector<uint16_t> GetLengthIndicators() const;
    
    uint32_t GetSerializedSize() const;
    void Serialize(uint8_t* buffer) const;
    uint32_t Deserialize(const uint8_t* buffer, uint32_t size);
    
private:
    uint8_t m_fi;                           ///< Framing info
    bool m_e;                               ///< Extension bit
    uint16_t m_sn;                          ///< Sequence number (5 or 10 bits)
    std::vector<uint16_t> m_lis;            ///< Length indicators
};

/**
 * \ingroup nbiot
 *
 * \brief RLC PDU header for AM mode
 */
class NbIotRlcAmHeader
{
public:
    NbIotRlcAmHeader();
    
    void SetDataControlBit(bool dc);        ///< True for data PDU
    bool IsDataPdu() const;
    
    void SetResegmentationFlag(bool rf);
    bool GetResegmentationFlag() const;
    
    void SetPollingBit(bool p);
    bool GetPollingBit() const;
    
    void SetFramingInfo(uint8_t fi);
    uint8_t GetFramingInfo() const;
    
    void SetSequenceNumber(uint16_t sn);
    uint16_t GetSequenceNumber() const;
    
    void SetLastSegmentFlag(bool lsf);
    bool GetLastSegmentFlag() const;
    
    void SetSegmentOffset(uint16_t so);
    uint16_t GetSegmentOffset() const;
    
    void AddLengthIndicator(uint16_t li);
    std::vector<uint16_t> GetLengthIndicators() const;
    
    uint32_t GetSerializedSize() const;
    void Serialize(uint8_t* buffer) const;
    uint32_t Deserialize(const uint8_t* buffer, uint32_t size);
    
private:
    bool m_dc;                              ///< Data/Control
    bool m_rf;                              ///< Resegmentation flag
    bool m_p;                               ///< Polling bit
    uint8_t m_fi;                           ///< Framing info
    bool m_e;                               ///< Extension bit
    uint16_t m_sn;                          ///< Sequence number (10 bits for AM)
    bool m_lsf;                             ///< Last segment flag
    uint16_t m_so;                          ///< Segment offset
    std::vector<uint16_t> m_lis;            ///< Length indicators
};

/**
 * \ingroup nbiot
 *
 * \brief Base class for NB-IoT RLC entities
 */
class NbIotRlc : public Object
{
public:
    static TypeId GetTypeId();
    
    NbIotRlc();
    ~NbIotRlc() override;
    
    /**
     * \brief Set the LCID
     * \param lcid Logical channel ID
     */
    void SetLcId(uint8_t lcid);
    
    /**
     * \brief Get the LCID
     * \return Logical channel ID
     */
    uint8_t GetLcId() const;
    
    /**
     * \brief Set the RNTI
     * \param rnti UE identifier
     */
    void SetRnti(uint16_t rnti);
    
    /**
     * \brief Get the RNTI
     * \return UE identifier
     */
    uint16_t GetRnti() const;
    
    /**
     * \brief Set the MAC layer
     * \param mac Pointer to MAC layer
     */
    virtual void SetMac(Ptr<NbIotMac> mac);
    
    /**
     * \brief Set the PDCP layer
     * \param pdcp Pointer to PDCP layer
     */
    virtual void SetPdcp(Ptr<NbIotPdcp> pdcp);
    
    /**
     * \brief Receive SDU from upper layer (PDCP)
     * \param packet SDU to transmit
     */
    virtual void TransmitSdu(Ptr<Packet> packet) = 0;
    
    /**
     * \brief Receive PDU from lower layer (MAC)
     * \param packet Received PDU
     */
    virtual void ReceivePdu(Ptr<Packet> packet) = 0;
    
    /**
     * \brief Notify opportunity to transmit
     * \param bytes Available bytes
     * \return PDU to transmit (or nullptr)
     */
    virtual Ptr<Packet> NotifyTxOpportunity(uint32_t bytes) = 0;
    
    /**
     * \brief Get transmit buffer size
     * \return Number of bytes pending
     */
    virtual uint32_t GetTxBufferSize() const = 0;
    
    /**
     * \brief Get RLC mode
     * \return RLC mode
     */
    virtual NbIotRlcMode GetRlcMode() const = 0;
    
protected:
    void DoDispose() override;
    
    uint8_t m_lcid;                         ///< Logical channel ID
    uint16_t m_rnti;                        ///< UE identifier
    
    Ptr<NbIotMac> m_mac;                    ///< MAC layer
    Ptr<NbIotPdcp> m_pdcp;                  ///< PDCP layer
    
    /// Traces
    TracedCallback<uint16_t, uint8_t, uint32_t> m_txPduTrace;   ///< TX PDU trace
    TracedCallback<uint16_t, uint8_t, uint32_t> m_rxPduTrace;   ///< RX PDU trace
    TracedCallback<uint16_t, uint8_t, uint32_t> m_txSduTrace;   ///< TX SDU trace
    TracedCallback<uint16_t, uint8_t, uint32_t> m_rxSduTrace;   ///< RX SDU trace
};

/**
 * \ingroup nbiot
 *
 * \brief RLC Transparent Mode entity
 *
 * Used for BCCH, CCCH, and PCCH. No segmentation, concatenation, or headers.
 */
class NbIotRlcTm : public NbIotRlc
{
public:
    static TypeId GetTypeId();
    
    NbIotRlcTm();
    ~NbIotRlcTm() override;
    
    void TransmitSdu(Ptr<Packet> packet) override;
    void ReceivePdu(Ptr<Packet> packet) override;
    Ptr<Packet> NotifyTxOpportunity(uint32_t bytes) override;
    uint32_t GetTxBufferSize() const override;
    NbIotRlcMode GetRlcMode() const override;
    
protected:
    void DoDispose() override;
    
private:
    std::deque<NbIotRlcSduInfo> m_txBuffer;  ///< Transmit buffer
};

/**
 * \ingroup nbiot
 *
 * \brief RLC Unacknowledged Mode entity
 *
 * Provides segmentation and reassembly without ARQ.
 * Used for delay-sensitive traffic and broadcast.
 */
class NbIotRlcUm : public NbIotRlc
{
public:
    static TypeId GetTypeId();
    
    NbIotRlcUm();
    ~NbIotRlcUm() override;
    
    void TransmitSdu(Ptr<Packet> packet) override;
    void ReceivePdu(Ptr<Packet> packet) override;
    Ptr<Packet> NotifyTxOpportunity(uint32_t bytes) override;
    uint32_t GetTxBufferSize() const override;
    NbIotRlcMode GetRlcMode() const override;
    
    /**
     * \brief Set SN field length
     * \param length 5 or 10 bits
     */
    void SetSnFieldLength(uint8_t length);
    
    /**
     * \brief Set t-Reordering timer value
     * \param timer Timer value
     */
    void SetTReordering(Time timer);
    
protected:
    void DoDispose() override;
    
private:
    /**
     * \brief Check if SN is within window
     * \param sn Sequence number to check
     * \return True if within window
     */
    bool IsInsideWindow(uint16_t sn) const;
    
    /**
     * \brief Reassemble SDU from received segments
     */
    void ReassembleSdu();
    
    /**
     * \brief t-Reordering timer expiry handler
     */
    void TReorderingExpiry();
    
    // Transmit side
    std::deque<NbIotRlcSduInfo> m_txBuffer;         ///< Transmit buffer
    uint16_t m_txSn;                                 ///< Next TX SN
    uint8_t m_snLength;                              ///< SN field length (5 or 10)
    uint16_t m_snModulus;                            ///< 2^snLength
    uint16_t m_windowSize;                           ///< Window size
    
    // Receive side
    std::map<uint16_t, NbIotRlcPduSegment> m_rxBuffer;  ///< Receive buffer
    uint16_t m_vrUr;                                 ///< VR(UR) - earliest SN not received
    uint16_t m_vrUx;                                 ///< VR(UX) - SN following reordering timer trigger
    uint16_t m_vrUh;                                 ///< VR(UH) - highest SN received + 1
    
    Time m_tReordering;                              ///< t-Reordering timer value
    EventId m_reorderingTimer;                       ///< Reordering timer event
};

/**
 * \ingroup nbiot
 *
 * \brief RLC Acknowledged Mode entity
 *
 * Provides reliable delivery with ARQ, segmentation, and reassembly.
 * Used for control plane and reliable user plane traffic.
 */
class NbIotRlcAm : public NbIotRlc
{
public:
    static TypeId GetTypeId();
    
    NbIotRlcAm();
    ~NbIotRlcAm() override;
    
    void TransmitSdu(Ptr<Packet> packet) override;
    void ReceivePdu(Ptr<Packet> packet) override;
    Ptr<Packet> NotifyTxOpportunity(uint32_t bytes) override;
    uint32_t GetTxBufferSize() const override;
    NbIotRlcMode GetRlcMode() const override;
    
    /**
     * \brief Set t-PollRetransmit timer value
     * \param timer Timer value
     */
    void SetTPollRetransmit(Time timer);
    
    /**
     * \brief Set t-Reordering timer value
     * \param timer Timer value
     */
    void SetTReordering(Time timer);
    
    /**
     * \brief Set t-StatusProhibit timer value
     * \param timer Timer value
     */
    void SetTStatusProhibit(Time timer);
    
    /**
     * \brief Set maximum retransmissions
     * \param maxRetx Maximum retransmissions
     */
    void SetMaxRetxThreshold(uint8_t maxRetx);
    
    /**
     * \brief Set poll PDU threshold
     * \param pollPdu PDU count before poll
     */
    void SetPollPdu(uint16_t pollPdu);
    
    /**
     * \brief Set poll byte threshold
     * \param pollByte Byte count before poll
     */
    void SetPollByte(uint32_t pollByte);
    
protected:
    void DoDispose() override;
    
private:
    /**
     * \brief Check if SN is inside transmit window
     */
    bool IsInsideTxWindow(uint16_t sn) const;
    
    /**
     * \brief Check if SN is inside receive window
     */
    bool IsInsideRxWindow(uint16_t sn) const;
    
    /**
     * \brief Process received data PDU
     */
    void ProcessDataPdu(Ptr<Packet> packet, const NbIotRlcAmHeader& header);
    
    /**
     * \brief Process received status PDU
     */
    void ProcessStatusPdu(Ptr<Packet> packet);
    
    /**
     * \brief Build and send status PDU
     */
    Ptr<Packet> BuildStatusPdu();
    
    /**
     * \brief Reassemble SDU from received PDUs
     */
    void ReassembleSdu();
    
    /**
     * \brief Timer expiry handlers
     */
    void TPollRetransmitExpiry();
    void TReorderingExpiry();
    void TStatusProhibitExpiry();
    
    /**
     * \brief Check if poll should be triggered
     */
    bool ShouldPoll();
    
    /**
     * \brief Handle retransmission request (NACK)
     */
    void HandleNack(uint16_t sn, uint16_t soStart, uint16_t soEnd);
    
    // State variables (3GPP TS 36.322 Section 7.1)
    // Transmit side
    uint16_t m_vtA;         ///< VT(A) - TX window base
    uint16_t m_vtMs;        ///< VT(MS) - max SN that can be TX'd
    uint16_t m_vtS;         ///< VT(S) - next SN to assign
    uint16_t m_pollSn;      ///< POLL_SN - SN of last PDU with poll
    
    // Receive side
    uint16_t m_vrR;         ///< VR(R) - RX window base
    uint16_t m_vrMr;        ///< VR(MR) - max acceptable SN
    uint16_t m_vrX;         ///< VR(X) - reordering timer SN
    uint16_t m_vrMs;        ///< VR(MS) - highest SN triggering status
    uint16_t m_vrH;         ///< VR(H) - highest SN received
    
    // Configuration
    static constexpr uint16_t AM_WINDOW_SIZE = 512;
    static constexpr uint16_t AM_SN_MODULUS = 1024;
    
    uint8_t m_maxRetxThreshold;     ///< Max retransmissions
    uint16_t m_pollPdu;             ///< Poll PDU counter threshold
    uint32_t m_pollByte;            ///< Poll byte counter threshold
    
    // Counters
    uint16_t m_pduWithoutPoll;      ///< PDUs since last poll
    uint32_t m_byteWithoutPoll;     ///< Bytes since last poll
    
    // Buffers
    std::deque<NbIotRlcSduInfo> m_txBuffer;                     ///< TX SDU buffer
    std::map<uint16_t, Ptr<Packet>> m_txedBuffer;               ///< TX'd PDU buffer (for retx)
    std::map<uint16_t, uint8_t> m_retxCount;                    ///< Retransmission count per SN
    std::deque<uint16_t> m_retxQueue;                           ///< Retransmission queue
    std::map<uint16_t, NbIotRlcPduSegment> m_rxBuffer;          ///< RX buffer
    
    // Status
    bool m_statusTriggered;         ///< Status report triggered
    std::set<uint16_t> m_nackList;  ///< NACKed SNs for status report
    
    // Timers
    Time m_tPollRetransmit;
    Time m_tReordering;
    Time m_tStatusProhibit;
    
    EventId m_pollRetransmitTimer;
    EventId m_reorderingTimer;
    EventId m_statusProhibitTimer;
};

/**
 * \ingroup nbiot
 *
 * \brief Convert RLC mode to string
 * \param mode RLC mode
 * \return String representation
 */
inline std::string RlcModeToString(NbIotRlcMode mode)
{
    switch (mode)
    {
        case NbIotRlcMode::TM: return "TM";
        case NbIotRlcMode::UM: return "UM";
        case NbIotRlcMode::AM: return "AM";
        default: return "Unknown";
    }
}

} // namespace ns3

#endif /* NBIOT_RLC_H */
