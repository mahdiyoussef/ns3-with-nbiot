/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT PDCP Layer Header
 *
 * References:
 * - 3GPP TS 36.323 PDCP protocol specification
 */

#ifndef NBIOT_PDCP_H
#define NBIOT_PDCP_H

#include "nbiot-common.h"

#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/packet.h>
#include <ns3/traced-callback.h>
#include <ns3/event-id.h>

#include <deque>
#include <map>

namespace ns3 {

class NbIotRlc;

/**
 * \ingroup nbiot
 *
 * \brief PDCP header for data plane
 */
class NbIotPdcpHeader
{
public:
    NbIotPdcpHeader();
    
    /**
     * \brief Set the D/C bit
     * \param dc True for data PDU
     */
    void SetDcBit(bool dc);
    bool GetDcBit() const;
    
    /**
     * \brief Set sequence number
     * \param sn Sequence number
     */
    void SetSequenceNumber(uint16_t sn);
    uint16_t GetSequenceNumber() const;
    
    /**
     * \brief Set SN length
     * \param length 5, 7, 12, 15, or 18 bits
     */
    void SetSnLength(uint8_t length);
    uint8_t GetSnLength() const;
    
    uint32_t GetSerializedSize() const;
    void Serialize(uint8_t* buffer) const;
    uint32_t Deserialize(const uint8_t* buffer, uint32_t size);
    
private:
    bool m_dc;              ///< Data/Control
    uint32_t m_sn;          ///< Sequence number
    uint8_t m_snLength;     ///< SN length in bits
};

/**
 * \ingroup nbiot
 *
 * \brief PDCP SDU information for buffering
 */
struct NbIotPdcpSduInfo
{
    Ptr<Packet> packet;
    uint32_t pdcpSn;
    Time arrivalTime;
};

/**
 * \ingroup nbiot
 *
 * \brief NB-IoT PDCP entity
 *
 * Implements PDCP functionality for NB-IoT including:
 * - Header compression (ROHC - simplified)
 * - Sequence numbering
 * - In-order delivery for AM bearers
 * - Reordering for UM bearers
 */
class NbIotPdcp : public Object
{
public:
    static TypeId GetTypeId();
    
    NbIotPdcp();
    ~NbIotPdcp() override;
    
    /**
     * \brief Set bearer parameters
     * \param rnti UE identifier
     * \param lcid Logical channel ID
     * \param isDataRadioBearer True for DRB, false for SRB
     */
    void SetBearerConfig(uint16_t rnti, uint8_t lcid, bool isDataRadioBearer);
    
    /**
     * \brief Set the RLC entity
     * \param rlc Pointer to RLC layer
     */
    void SetRlc(Ptr<NbIotRlc> rlc);
    
    /**
     * \brief Get the RLC entity
     * \return Pointer to RLC layer
     */
    Ptr<NbIotRlc> GetRlc() const;
    
    /**
     * \brief Set header compression enabled
     * \param enabled True to enable ROHC
     */
    void SetHeaderCompression(bool enabled);
    
    /**
     * \brief Set SN length
     * \param length 5, 7, 12, 15, or 18 bits
     */
    void SetSnLength(uint8_t length);
    
    /**
     * \brief Set discard timer
     * \param timer Discard timer value
     */
    void SetDiscardTimer(Time timer);
    
    /**
     * \brief Receive SDU from upper layer (IP/RRC)
     * \param packet SDU to transmit
     */
    void TransmitSdu(Ptr<Packet> packet);
    
    /**
     * \brief Receive PDU from lower layer (RLC)
     * \param packet Received PDU
     */
    void ReceivePdu(Ptr<Packet> packet);
    
    /**
     * \brief Get RNTI
     * \return UE identifier
     */
    uint16_t GetRnti() const;
    
    /**
     * \brief Get LCID
     * \return Logical channel ID
     */
    uint8_t GetLcId() const;
    
    /**
     * \brief Get transmit buffer size
     * \return Bytes pending
     */
    uint32_t GetTxBufferSize() const;
    
    /// Callback for SDU delivery to upper layer
    typedef Callback<void, Ptr<Packet>> ReceiveSduCallback;
    
    /**
     * \brief Set receive SDU callback
     * \param cb Callback function
     */
    void SetReceiveSduCallback(ReceiveSduCallback cb);
    
protected:
    void DoDispose() override;
    
private:
    /**
     * \brief Apply header compression
     * \param packet Packet to compress
     * \return Compressed packet
     */
    Ptr<Packet> CompressHeader(Ptr<Packet> packet);
    
    /**
     * \brief Apply header decompression
     * \param packet Packet to decompress
     * \return Decompressed packet
     */
    Ptr<Packet> DecompressHeader(Ptr<Packet> packet);
    
    /**
     * \brief Deliver PDUs in order
     */
    void DeliverInOrder();
    
    /**
     * \brief Discard timer expiry handler
     * \param pdcpSn SN of discarded SDU
     */
    void DiscardTimerExpiry(uint32_t pdcpSn);
    
    /**
     * \brief Reordering timer expiry handler
     */
    void ReorderingTimerExpiry();
    
    uint16_t m_rnti;                        ///< UE identifier
    uint8_t m_lcid;                         ///< Logical channel ID
    bool m_isDrb;                           ///< Data radio bearer flag
    
    Ptr<NbIotRlc> m_rlc;                    ///< RLC entity
    
    bool m_headerCompression;               ///< Header compression enabled
    uint8_t m_snLength;                     ///< SN length in bits
    uint32_t m_snModulus;                   ///< 2^snLength
    uint32_t m_windowSize;                  ///< Reordering window size
    
    // Transmit side
    uint32_t m_txNext;                      ///< Next TX PDCP SN
    std::deque<NbIotPdcpSduInfo> m_txBuffer;    ///< TX buffer
    std::map<uint32_t, EventId> m_discardTimers; ///< Discard timers per SN
    Time m_discardTimer;                    ///< Discard timer value
    
    // Receive side
    uint32_t m_rxNext;                      ///< Expected RX SN
    uint32_t m_rxDeliv;                     ///< First SN not delivered
    uint32_t m_rxReord;                     ///< Reordering SN
    std::map<uint32_t, Ptr<Packet>> m_rxBuffer;  ///< RX buffer for reordering
    EventId m_reorderingTimer;              ///< Reordering timer
    Time m_tReordering;                     ///< Reordering timer value
    
    ReceiveSduCallback m_receiveSduCallback; ///< Callback for SDU delivery
    
    /// Traces
    TracedCallback<uint16_t, uint8_t, uint32_t> m_txPduTrace;
    TracedCallback<uint16_t, uint8_t, uint32_t> m_rxPduTrace;
    TracedCallback<uint16_t, uint8_t, uint32_t, Time> m_txDelayTrace;
};

/**
 * \ingroup nbiot
 *
 * \brief PDCP entity for control plane (SRB)
 *
 * Simplified PDCP for SRBs with integrity protection
 */
class NbIotPdcpSrb : public NbIotPdcp
{
public:
    static TypeId GetTypeId();
    
    NbIotPdcpSrb();
    ~NbIotPdcpSrb() override;
    
    /**
     * \brief Enable integrity protection
     * \param enabled True to enable
     */
    void SetIntegrityProtection(bool enabled);
    
    /**
     * \brief Enable ciphering
     * \param enabled True to enable
     */
    void SetCiphering(bool enabled);
    
private:
    bool m_integrityProtection;     ///< Integrity protection enabled
    bool m_ciphering;               ///< Ciphering enabled
};

/**
 * \ingroup nbiot
 *
 * \brief PDCP entity for user plane (DRB)
 *
 * PDCP for DRBs with header compression (ROHC)
 */
class NbIotPdcpDrb : public NbIotPdcp
{
public:
    static TypeId GetTypeId();
    
    NbIotPdcpDrb();
    ~NbIotPdcpDrb() override;
    
    /**
     * \brief Set ROHC profile
     * \param profile ROHC profile ID
     */
    void SetRohcProfile(uint16_t profile);
    
private:
    uint16_t m_rohcProfile;         ///< ROHC profile ID
};

} // namespace ns3

#endif /* NBIOT_PDCP_H */
