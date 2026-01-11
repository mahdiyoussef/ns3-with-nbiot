/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT MAC Layer as per 3GPP TS 36.321
 */

#ifndef NBIOT_MAC_H
#define NBIOT_MAC_H

#include "nbiot-common.h"

#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/traced-callback.h>
#include <ns3/nstime.h>
#include <ns3/event-id.h>

#include <map>
#include <queue>
#include <vector>

namespace ns3 {

class NbIotPhy;
class NbIotRlc;

/**
 * \ingroup nbiot
 * \brief NB-IoT MAC PDU header
 *
 * Implements MAC PDU subheader format per 3GPP TS 36.321 Section 6.1.2
 */
class NbIotMacSubheader
{
public:
    /**
     * \brief LCID values for NB-IoT
     */
    enum Lcid : uint8_t
    {
        CCCH = 0,                    ///< Common Control Channel
        LCID_1 = 1,                  ///< Logical Channel ID 1 (DCCH)
        LCID_2 = 2,                  ///< Logical Channel ID 2 (DTCH)
        // ... LCID 3-10 reserved
        RESERVED_11 = 11,
        RESERVED_12 = 12,
        RECOMMENDED_BIT_RATE = 13,   ///< Recommended bit rate (Release 14)
        SC_MCCH_CHANGE = 14,         ///< SC-MCCH change notification
        TRUNCATED_SIDELINK_BSR = 15, ///< Truncated sidelink BSR
        PHR = 26,                    ///< Power Headroom Report
        SHORT_BSR = 29,              ///< Short BSR
        LONG_BSR = 30,               ///< Long BSR
        PADDING = 31                 ///< Padding
    };

    NbIotMacSubheader();
    NbIotMacSubheader(uint8_t lcid, uint16_t length, bool extension);

    uint8_t GetLcid() const { return m_lcid; }
    uint16_t GetLength() const { return m_length; }
    bool HasExtension() const { return m_extension; }
    bool IsFixedSize() const;

    void SetLcid(uint8_t lcid) { m_lcid = lcid; }
    void SetLength(uint16_t length) { m_length = length; }
    void SetExtension(bool ext) { m_extension = ext; }

    uint8_t GetSerializedSize() const;
    void Serialize(uint8_t* buffer) const;
    uint32_t Deserialize(const uint8_t* buffer);

private:
    uint8_t m_lcid;
    uint16_t m_length;
    bool m_extension;
    bool m_format;  // 0 = short, 1 = long length field
};

/**
 * \ingroup nbiot
 * \brief NB-IoT MAC PDU
 */
class NbIotMacPdu : public Object
{
public:
    static TypeId GetTypeId();

    NbIotMacPdu();
    ~NbIotMacPdu() override;

    /**
     * \brief Add a MAC SDU to the PDU
     * \param lcid Logical channel ID
     * \param sdu The SDU packet
     */
    void AddSdu(uint8_t lcid, Ptr<Packet> sdu);

    /**
     * \brief Add a MAC Control Element
     * \param lcid LCID indicating CE type
     * \param data CE data
     */
    void AddControlElement(uint8_t lcid, const std::vector<uint8_t>& data);

    /**
     * \brief Add padding to reach target size
     * \param targetSize Target PDU size in bytes
     */
    void AddPadding(uint16_t targetSize);

    /**
     * \brief Get the serialized PDU
     * \return PDU as a packet
     */
    Ptr<Packet> Serialize() const;

    /**
     * \brief Parse a received MAC PDU
     * \param pdu The received PDU
     * \return Map of LCID to SDU packets
     */
    static std::map<uint8_t, std::vector<Ptr<Packet>>> Deserialize(Ptr<Packet> pdu);

    /**
     * \brief Get current PDU size
     * \return Size in bytes
     */
    uint16_t GetSize() const;

private:
    std::vector<NbIotMacSubheader> m_subheaders;
    std::vector<std::vector<uint8_t>> m_payloads;
    uint16_t m_size;
};

/**
 * \ingroup nbiot
 * \brief HARQ entity for NB-IoT (manages individual HARQ process state)
 */
class NbIotHarqEntity
{
public:
    NbIotHarqEntity();
    
    void Reset();
    bool IsActive() const { return m_active; }
    void SetActive(bool active) { m_active = active; }
    
    uint8_t GetTxCount() const { return m_txCount; }
    void IncrementTxCount() { ++m_txCount; }
    
    void SetNdi(bool ndi) { m_ndi = ndi; }
    bool GetNdi() const { return m_ndi; }
    
    void SetTbs(uint16_t tbs) { m_tbs = tbs; }
    uint16_t GetTbs() const { return m_tbs; }
    
    void SetPacket(Ptr<Packet> packet) { m_packet = packet; }
    Ptr<Packet> GetPacket() const { return m_packet; }
    
    void SetRepetitions(uint8_t reps) { m_repetitions = reps; }
    uint8_t GetRepetitions() const { return m_repetitions; }

private:
    bool m_active;
    uint8_t m_txCount;
    bool m_ndi;
    uint16_t m_tbs;
    uint8_t m_repetitions;
    Ptr<Packet> m_packet;
};

/**
 * \ingroup nbiot
 * \brief NB-IoT HARQ Manager
 */
class NbIotHarqManager : public Object
{
public:
    static TypeId GetTypeId();

    NbIotHarqManager();
    ~NbIotHarqManager() override;

    /**
     * \brief Get the number of HARQ processes
     * \return Number of processes (typically 1-2 for NB-IoT)
     */
    uint8_t GetNumProcesses() const { return m_numProcesses; }

    /**
     * \brief Set number of HARQ processes
     * \param num Number of processes
     */
    void SetNumProcesses(uint8_t num);

    /**
     * \brief Get an available HARQ process
     * \return Process ID, or -1 if none available
     */
    int8_t GetAvailableProcess();

    /**
     * \brief Get a specific HARQ process
     * \param processId Process ID
     * \return Pointer to the process
     */
    NbIotHarqEntity* GetProcess(uint8_t processId);

    /**
     * \brief Process HARQ ACK feedback
     * \param processId Process ID
     */
    void ProcessAck(uint8_t processId);

    /**
     * \brief Process HARQ NACK feedback
     * \param processId Process ID
     * \return True if retransmission needed, false if max reached
     */
    bool ProcessNack(uint8_t processId);

    /**
     * \brief Reset a HARQ process
     * \param processId Process ID
     */
    void ResetProcess(uint8_t processId);

    /**
     * \brief Set max transmissions
     * \param maxTx Maximum transmissions
     */
    void SetMaxTransmissions(uint8_t maxTx) { m_maxTransmissions = maxTx; }

private:
    uint8_t m_numProcesses;
    uint8_t m_maxTransmissions;
    std::vector<NbIotHarqEntity> m_processes;
};

/**
 * \ingroup nbiot
 * \brief Abstract base class for NB-IoT MAC layer
 */
class NbIotMac : public Object
{
public:
    static TypeId GetTypeId();

    NbIotMac();
    ~NbIotMac() override;

    void DoDispose() override;

    /**
     * \brief Set the RNTI
     * \param rnti Radio Network Temporary Identifier
     */
    void SetRnti(uint16_t rnti);

    /**
     * \brief Get the RNTI
     * \return RNTI
     */
    uint16_t GetRnti() const;

    /**
     * \brief Receive a MAC PDU from PHY
     * \param packet The MAC PDU
     */
    virtual void ReceivePdu(Ptr<Packet> packet) = 0;

    /**
     * \brief Get the HARQ manager
     * \return HARQ manager
     */
    Ptr<NbIotHarqManager> GetHarqManager() const;

protected:
    uint16_t m_rnti;
    Ptr<NbIotHarqManager> m_harqManager;

    // Traced callbacks
    TracedCallback<uint16_t, uint16_t> m_macTxTrace;  // RNTI, size
    TracedCallback<uint16_t, uint16_t> m_macRxTrace;  // RNTI, size
};

} // namespace ns3

#endif /* NBIOT_MAC_H */
