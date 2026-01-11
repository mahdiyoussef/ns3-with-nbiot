/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Control Messages implementation
 */

#include "nbiot-control-messages.h"

#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotControlMessages");

// ====================== NbIotControlMessage ======================

NbIotControlMessage::NbIotControlMessage(MessageType type)
    : m_messageType(type)
{
}

NbIotControlMessage::~NbIotControlMessage()
{
}

NbIotControlMessage::MessageType
NbIotControlMessage::GetMessageType() const
{
    return m_messageType;
}

// ====================== NbIotDciMessage ======================

NbIotDciMessage::NbIotDciMessage(const NbIotDci& dci)
    : NbIotControlMessage(DCI)
    , m_dci(dci)
{
}

NbIotDciMessage::~NbIotDciMessage()
{
}

NbIotDci
NbIotDciMessage::GetDci() const
{
    return m_dci;
}

// ====================== NbIotMibMessage ======================

NbIotMibMessage::NbIotMibMessage(const NbIotMib& mib)
    : NbIotControlMessage(MIB)
    , m_mib(mib)
{
}

NbIotMibMessage::~NbIotMibMessage()
{
}

NbIotMib
NbIotMibMessage::GetMib() const
{
    return m_mib;
}

// ====================== NbIotRarMessage ======================

NbIotRarMessage::NbIotRarMessage()
    : NbIotControlMessage(RAR)
    , m_rarWindow(0)
    , m_preambleId(0)
{
    m_grant.timingAdvance = 0;
    m_grant.tempCRnti = 0;
    m_grant.mcs = 0;
    m_grant.resourceAssignment = 0;
    m_grant.subcarrierSpacing = 0;
}

NbIotRarMessage::~NbIotRarMessage()
{
}

void
NbIotRarMessage::SetRarWindow(uint16_t window)
{
    m_rarWindow = window;
}

uint16_t
NbIotRarMessage::GetRarWindow() const
{
    return m_rarWindow;
}

void
NbIotRarMessage::SetPreambleId(uint8_t preambleId)
{
    m_preambleId = preambleId;
}

uint8_t
NbIotRarMessage::GetPreambleId() const
{
    return m_preambleId;
}

void
NbIotRarMessage::SetRarGrant(const RarGrant& grant)
{
    m_grant = grant;
}

NbIotRarMessage::RarGrant
NbIotRarMessage::GetRarGrant() const
{
    return m_grant;
}

// ====================== NbIotHarqFeedbackMessage ======================

NbIotHarqFeedbackMessage::NbIotHarqFeedbackMessage(uint16_t rnti, uint8_t harqId, bool ack)
    : NbIotControlMessage(HARQ_FEEDBACK)
    , m_rnti(rnti)
    , m_harqId(harqId)
    , m_ack(ack)
{
}

NbIotHarqFeedbackMessage::~NbIotHarqFeedbackMessage()
{
}

uint16_t
NbIotHarqFeedbackMessage::GetRnti() const
{
    return m_rnti;
}

uint8_t
NbIotHarqFeedbackMessage::GetHarqId() const
{
    return m_harqId;
}

bool
NbIotHarqFeedbackMessage::IsAck() const
{
    return m_ack;
}

// ====================== NbIotBsrMessage ======================

NbIotBsrMessage::NbIotBsrMessage(BsrType type)
    : NbIotControlMessage(BSR)
    , m_bsrType(type)
{
    m_bufferSizes.fill(0);
}

NbIotBsrMessage::~NbIotBsrMessage()
{
}

NbIotBsrMessage::BsrType
NbIotBsrMessage::GetBsrType() const
{
    return m_bsrType;
}

void
NbIotBsrMessage::SetBufferSize(uint8_t lcg, uint8_t bufferSize)
{
    if (lcg < 4)
    {
        m_bufferSizes[lcg] = bufferSize;
    }
}

uint8_t
NbIotBsrMessage::GetBufferSize(uint8_t lcg) const
{
    if (lcg < 4)
    {
        return m_bufferSizes[lcg];
    }
    return 0;
}

// ====================== NbIotPhrMessage ======================

NbIotPhrMessage::NbIotPhrMessage(uint8_t powerHeadroom)
    : NbIotControlMessage(PHR)
    , m_powerHeadroom(powerHeadroom)
{
}

NbIotPhrMessage::~NbIotPhrMessage()
{
}

uint8_t
NbIotPhrMessage::GetPowerHeadroom() const
{
    return m_powerHeadroom;
}

double
NbIotPhrMessage::GetPowerHeadroomDb() const
{
    // Convert PHR index to dB value per 3GPP TS 36.133
    // Range: -23 dB to +40 dB
    return static_cast<double>(m_powerHeadroom) - 23.0;
}

// ====================== NbIotTaCommandMessage ======================

NbIotTaCommandMessage::NbIotTaCommandMessage(uint16_t rnti, uint16_t taValue)
    : NbIotControlMessage(TA_COMMAND)
    , m_rnti(rnti)
    , m_taValue(taValue)
{
}

NbIotTaCommandMessage::~NbIotTaCommandMessage()
{
}

uint16_t
NbIotTaCommandMessage::GetRnti() const
{
    return m_rnti;
}

uint16_t
NbIotTaCommandMessage::GetTaValue() const
{
    return m_taValue;
}

// ====================== NbIotPagingMessage ======================

NbIotPagingMessage::NbIotPagingMessage()
    : NbIotControlMessage(PAGING)
    , m_systemInfoModification(false)
{
}

NbIotPagingMessage::~NbIotPagingMessage()
{
}

void
NbIotPagingMessage::AddPagingRecord(const PagingRecord& record)
{
    m_pagingRecords.push_back(record);
}

std::vector<NbIotPagingMessage::PagingRecord>
NbIotPagingMessage::GetPagingRecords() const
{
    return m_pagingRecords;
}

void
NbIotPagingMessage::SetSystemInfoModification(bool modified)
{
    m_systemInfoModification = modified;
}

bool
NbIotPagingMessage::IsSystemInfoModified() const
{
    return m_systemInfoModification;
}

} // namespace ns3
