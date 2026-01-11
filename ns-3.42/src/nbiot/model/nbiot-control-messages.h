/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Control Messages
 */

#ifndef NBIOT_CONTROL_MESSAGES_H
#define NBIOT_CONTROL_MESSAGES_H

#include "nbiot-common.h"

#include <ns3/simple-ref-count.h>
#include <ns3/ptr.h>

#include <vector>

namespace ns3 {

/**
 * \ingroup nbiot
 * \brief Base class for NB-IoT control messages
 */
class NbIotControlMessage : public SimpleRefCount<NbIotControlMessage>
{
public:
    /**
     * \brief Message types
     */
    enum MessageType
    {
        DCI,           ///< Downlink Control Information
        MIB,           ///< Master Information Block
        SIB,           ///< System Information Block
        RAR,           ///< Random Access Response
        UL_DCI,        ///< Uplink DCI (grant)
        DL_DCI,        ///< Downlink DCI (assignment)
        HARQ_FEEDBACK, ///< HARQ ACK/NACK
        BSR,           ///< Buffer Status Report
        PHR,           ///< Power Headroom Report
        TA_COMMAND,    ///< Timing Advance Command
        PAGING         ///< Paging message
    };

    /**
     * \brief Constructor
     * \param type Message type
     */
    NbIotControlMessage(MessageType type);

    /**
     * \brief Destructor
     */
    virtual ~NbIotControlMessage();

    /**
     * \brief Get message type
     * \return Message type
     */
    MessageType GetMessageType() const;

private:
    MessageType m_messageType;
};

/**
 * \ingroup nbiot
 * \brief DCI control message
 */
class NbIotDciMessage : public NbIotControlMessage
{
public:
    /**
     * \brief Constructor
     * \param dci DCI content
     */
    NbIotDciMessage(const NbIotDci& dci);

    /**
     * \brief Destructor
     */
    ~NbIotDciMessage() override;

    /**
     * \brief Get DCI content
     * \return DCI
     */
    NbIotDci GetDci() const;

private:
    NbIotDci m_dci;
};

/**
 * \ingroup nbiot
 * \brief MIB-NB control message
 */
class NbIotMibMessage : public NbIotControlMessage
{
public:
    /**
     * \brief Constructor
     * \param mib MIB content
     */
    NbIotMibMessage(const NbIotMib& mib);

    /**
     * \brief Destructor
     */
    ~NbIotMibMessage() override;

    /**
     * \brief Get MIB content
     * \return MIB
     */
    NbIotMib GetMib() const;

private:
    NbIotMib m_mib;
};

/**
 * \ingroup nbiot
 * \brief Random Access Response message
 */
class NbIotRarMessage : public NbIotControlMessage
{
public:
    /**
     * \brief RAR grant structure
     */
    struct RarGrant
    {
        uint16_t timingAdvance;     ///< Timing advance command
        uint16_t tempCRnti;         ///< Temporary C-RNTI
        uint8_t mcs;                ///< MCS for Msg3
        uint8_t resourceAssignment; ///< Resource assignment for Msg3
        uint8_t subcarrierSpacing;  ///< Subcarrier spacing indication
    };

    /**
     * \brief Constructor
     */
    NbIotRarMessage();

    /**
     * \brief Destructor
     */
    ~NbIotRarMessage() override;

    /**
     * \brief Set RAR window size
     * \param window Window size in subframes
     */
    void SetRarWindow(uint16_t window);

    /**
     * \brief Get RAR window size
     * \return Window size
     */
    uint16_t GetRarWindow() const;

    /**
     * \brief Set preamble ID being responded to
     * \param preambleId Preamble ID
     */
    void SetPreambleId(uint8_t preambleId);

    /**
     * \brief Get preamble ID
     * \return Preamble ID
     */
    uint8_t GetPreambleId() const;

    /**
     * \brief Set RAR grant
     * \param grant RAR grant
     */
    void SetRarGrant(const RarGrant& grant);

    /**
     * \brief Get RAR grant
     * \return RAR grant
     */
    RarGrant GetRarGrant() const;

private:
    uint16_t m_rarWindow;
    uint8_t m_preambleId;
    RarGrant m_grant;
};

/**
 * \ingroup nbiot
 * \brief HARQ feedback message
 */
class NbIotHarqFeedbackMessage : public NbIotControlMessage
{
public:
    /**
     * \brief Constructor
     * \param rnti UE RNTI
     * \param harqId HARQ process ID
     * \param ack ACK (true) or NACK (false)
     */
    NbIotHarqFeedbackMessage(uint16_t rnti, uint8_t harqId, bool ack);

    /**
     * \brief Destructor
     */
    ~NbIotHarqFeedbackMessage() override;

    /**
     * \brief Get RNTI
     * \return RNTI
     */
    uint16_t GetRnti() const;

    /**
     * \brief Get HARQ process ID
     * \return HARQ ID
     */
    uint8_t GetHarqId() const;

    /**
     * \brief Check if ACK
     * \return True if ACK
     */
    bool IsAck() const;

private:
    uint16_t m_rnti;
    uint8_t m_harqId;
    bool m_ack;
};

/**
 * \ingroup nbiot
 * \brief Buffer Status Report message
 */
class NbIotBsrMessage : public NbIotControlMessage
{
public:
    /**
     * \brief BSR types
     */
    enum BsrType
    {
        SHORT_BSR,
        LONG_BSR,
        TRUNCATED_BSR,
        PADDING_BSR
    };

    /**
     * \brief Constructor
     * \param type BSR type
     */
    NbIotBsrMessage(BsrType type);

    /**
     * \brief Destructor
     */
    ~NbIotBsrMessage() override;

    /**
     * \brief Get BSR type
     * \return BSR type
     */
    BsrType GetBsrType() const;

    /**
     * \brief Set buffer size for a logical channel group
     * \param lcg Logical channel group (0-3)
     * \param bufferSize Buffer size index (0-63)
     */
    void SetBufferSize(uint8_t lcg, uint8_t bufferSize);

    /**
     * \brief Get buffer size for a logical channel group
     * \param lcg Logical channel group
     * \return Buffer size index
     */
    uint8_t GetBufferSize(uint8_t lcg) const;

private:
    BsrType m_bsrType;
    std::array<uint8_t, 4> m_bufferSizes;
};

/**
 * \ingroup nbiot
 * \brief Power Headroom Report message
 */
class NbIotPhrMessage : public NbIotControlMessage
{
public:
    /**
     * \brief Constructor
     * \param powerHeadroom Power headroom value (0-63)
     */
    NbIotPhrMessage(uint8_t powerHeadroom);

    /**
     * \brief Destructor
     */
    ~NbIotPhrMessage() override;

    /**
     * \brief Get power headroom
     * \return Power headroom value
     */
    uint8_t GetPowerHeadroom() const;

    /**
     * \brief Get power headroom in dB
     * \return Power headroom in dB
     */
    double GetPowerHeadroomDb() const;

private:
    uint8_t m_powerHeadroom;
};

/**
 * \ingroup nbiot
 * \brief Timing Advance command message
 */
class NbIotTaCommandMessage : public NbIotControlMessage
{
public:
    /**
     * \brief Constructor
     * \param rnti Target UE RNTI
     * \param taValue Timing advance value
     */
    NbIotTaCommandMessage(uint16_t rnti, uint16_t taValue);

    /**
     * \brief Destructor
     */
    ~NbIotTaCommandMessage() override;

    /**
     * \brief Get RNTI
     * \return RNTI
     */
    uint16_t GetRnti() const;

    /**
     * \brief Get timing advance value
     * \return TA value
     */
    uint16_t GetTaValue() const;

private:
    uint16_t m_rnti;
    uint16_t m_taValue;
};

/**
 * \ingroup nbiot
 * \brief Paging message
 */
class NbIotPagingMessage : public NbIotControlMessage
{
public:
    /**
     * \brief Paging record structure
     */
    struct PagingRecord
    {
        uint64_t ueIdentity;  ///< UE identity (IMSI or S-TMSI)
        bool useSTmsi;        ///< True if using S-TMSI
        uint8_t cnDomain;     ///< CN domain (PS or CS)
    };

    /**
     * \brief Constructor
     */
    NbIotPagingMessage();

    /**
     * \brief Destructor
     */
    ~NbIotPagingMessage() override;

    /**
     * \brief Add a paging record
     * \param record Paging record
     */
    void AddPagingRecord(const PagingRecord& record);

    /**
     * \brief Get paging records
     * \return Vector of paging records
     */
    std::vector<PagingRecord> GetPagingRecords() const;

    /**
     * \brief Set system information modification flag
     * \param modified True if SI is modified
     */
    void SetSystemInfoModification(bool modified);

    /**
     * \brief Check if SI is modified
     * \return True if modified
     */
    bool IsSystemInfoModified() const;

private:
    std::vector<PagingRecord> m_pagingRecords;
    bool m_systemInfoModification;
};

} // namespace ns3

#endif /* NBIOT_CONTROL_MESSAGES_H */
