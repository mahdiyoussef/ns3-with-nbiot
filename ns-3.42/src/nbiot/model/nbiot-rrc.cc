/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT RRC Layer implementation
 */

#include "nbiot-rrc.h"
#include "nbiot-ue-mac.h"
#include "nbiot-enb-mac.h"
#include "nbiot-ue-phy.h"
#include "nbiot-enb-phy.h"
#include "nbiot-pdcp.h"
#include "nbiot-rlc.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>
#include <ns3/boolean.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotRrc");

// ====================== NbIotRrc Base ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotRrc);

TypeId
NbIotRrc::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotRrc")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddTraceSource("StateTransition",
                        "RRC state transition",
                        MakeTraceSourceAccessor(&NbIotRrc::m_stateTransitionTrace),
                        "ns3::NbIotRrc::StateTransitionTracedCallback");
    return tid;
}

NbIotRrc::NbIotRrc()
    : m_cellId(0)
{
    NS_LOG_FUNCTION(this);
}

NbIotRrc::~NbIotRrc()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotRrc::DoDispose()
{
    Object::DoDispose();
}

void NbIotRrc::SetCellId(uint16_t cellId) { m_cellId = cellId; }
uint16_t NbIotRrc::GetCellId() const { return m_cellId; }

// ====================== NbIotUeRrc ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotUeRrc);

TypeId
NbIotUeRrc::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotUeRrc")
        .SetParent<NbIotRrc>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotUeRrc>()
        .AddAttribute("T300",
                      "T300 timer value",
                      TimeValue(Seconds(2)),
                      MakeTimeAccessor(&NbIotUeRrc::m_t300Value),
                      MakeTimeChecker())
        .AddAttribute("T301",
                      "T301 timer value",
                      TimeValue(Seconds(2)),
                      MakeTimeAccessor(&NbIotUeRrc::m_t301Value),
                      MakeTimeChecker())
        .AddAttribute("T311",
                      "T311 timer value",
                      TimeValue(Seconds(3)),
                      MakeTimeAccessor(&NbIotUeRrc::m_t311Value),
                      MakeTimeChecker())
        .AddAttribute("MaxConnectionAttempts",
                      "Maximum connection attempts",
                      UintegerValue(5),
                      MakeUintegerAccessor(&NbIotUeRrc::m_maxConnectionAttempts),
                      MakeUintegerChecker<uint8_t>())
        .AddTraceSource("Connection",
                        "UE connection trace",
                        MakeTraceSourceAccessor(&NbIotUeRrc::m_connectionTrace),
                        "ns3::NbIotUeRrc::ConnectionTracedCallback");
    return tid;
}

NbIotUeRrc::NbIotUeRrc()
    : m_imsi(0)
    , m_rnti(0)
    , m_state(NbIotRrcState::RRC_IDLE)
    , m_estCause(NbIotRrcEstablishmentCause::MO_DATA)
    , m_hasMib(false)
    , m_hasSib1(false)
    , m_hasSib2(false)
    , m_connectionAttempts(0)
    , m_maxConnectionAttempts(5)
{
    NS_LOG_FUNCTION(this);
}

NbIotUeRrc::~NbIotUeRrc()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotUeRrc::DoDispose()
{
    m_mac = nullptr;
    m_phy = nullptr;
    m_pdcpEntities.clear();
    
    if (m_t300.IsPending())
        Simulator::Cancel(m_t300);
    if (m_t301.IsPending())
        Simulator::Cancel(m_t301);
    if (m_t311.IsPending())
        Simulator::Cancel(m_t311);
    
    NbIotRrc::DoDispose();
}

void NbIotUeRrc::SetImsi(uint64_t imsi) { m_imsi = imsi; }
uint64_t NbIotUeRrc::GetImsi() const { return m_imsi; }
void NbIotUeRrc::SetRnti(uint16_t rnti) { m_rnti = rnti; }
uint16_t NbIotUeRrc::GetRnti() const { return m_rnti; }
void NbIotUeRrc::SetMac(Ptr<NbIotUeMac> mac) { m_mac = mac; }
void NbIotUeRrc::SetPhy(Ptr<NbIotUePhy> phy) { m_phy = phy; }
NbIotRrcState NbIotUeRrc::GetState() const { return m_state; }
bool NbIotUeRrc::IsConnected() const { return m_state == NbIotRrcState::RRC_CONNECTED; }

void
NbIotUeRrc::SetDataReceivedCallback(DataReceivedCallback cb)
{
    m_dataReceivedCallback = cb;
}

void
NbIotUeRrc::SetConnectionEstablishedCallback(ConnectionEstablishedCallback cb)
{
    m_connEstCallback = cb;
}

void
NbIotUeRrc::Connect(NbIotRrcEstablishmentCause cause)
{
    NS_LOG_FUNCTION(this << static_cast<int>(cause));
    
    if (m_state == NbIotRrcState::RRC_CONNECTED)
    {
        NS_LOG_WARN("UE already connected");
        return;
    }
    
    m_estCause = cause;
    m_connectionAttempts = 0;
    
    // Check if we have required system information
    if (!m_hasMib || !m_hasSib1)
    {
        NS_LOG_INFO("UE waiting for system information before connection");
        // In real implementation, would wait for SI acquisition
    }
    
    SendRrcConnectionRequest();
}

void
NbIotUeRrc::Disconnect()
{
    NS_LOG_FUNCTION(this);
    
    if (m_state != NbIotRrcState::RRC_CONNECTED)
    {
        return;
    }
    
    SwitchToState(NbIotRrcState::RRC_IDLE);
    
    // Clear bearers
    m_pdcpEntities.clear();
    
    NS_LOG_INFO("UE " << m_imsi << " disconnected");
}

void
NbIotUeRrc::SendRrcConnectionRequest()
{
    NS_LOG_FUNCTION(this);
    
    m_connectionAttempts++;
    
    if (m_connectionAttempts > m_maxConnectionAttempts)
    {
        NS_LOG_WARN("UE " << m_imsi << " max connection attempts reached");
        return;
    }
    
    // Build RRC Connection Request
    // Simplified: just encode message type and establishment cause
    uint8_t msgBuf[16];
    msgBuf[0] = static_cast<uint8_t>(NbIotRrcMessageType::RRC_CONNECTION_REQUEST);
    msgBuf[1] = static_cast<uint8_t>(m_estCause);
    // Add UE identity (S-TMSI or random value)
    msgBuf[2] = (m_imsi >> 24) & 0xFF;
    msgBuf[3] = (m_imsi >> 16) & 0xFF;
    msgBuf[4] = (m_imsi >> 8) & 0xFF;
    msgBuf[5] = m_imsi & 0xFF;
    
    Ptr<Packet> rrcMsg = Create<Packet>(msgBuf, 6);
    
    NS_LOG_INFO("UE " << m_imsi << " sending RRCConnectionRequest (attempt "
                << static_cast<int>(m_connectionAttempts) << ")");
    
    // Start T300 timer
    if (m_t300.IsPending())
    {
        Simulator::Cancel(m_t300);
    }
    m_t300 = Simulator::Schedule(m_t300Value, &NbIotUeRrc::T300Expiry, this);
    
    // Send via MAC (triggers RACH if not connected)
    if (m_mac)
    {
        // Use CCCH (LCID 0)
        m_mac->TransmitSdu(0, rrcMsg);
    }
}

void
NbIotUeRrc::ReceiveRrcMessage(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    uint8_t msgBuf[256];
    packet->CopyData(msgBuf, std::min(packet->GetSize(), static_cast<uint32_t>(256)));
    
    auto msgType = static_cast<NbIotRrcMessageType>(msgBuf[0]);
    
    NS_LOG_DEBUG("UE received RRC message: " << RrcMessageTypeToString(msgType));
    
    switch (msgType)
    {
        case NbIotRrcMessageType::RRC_CONNECTION_SETUP:
            ProcessRrcConnectionSetup(packet);
            break;
        case NbIotRrcMessageType::RRC_CONNECTION_RECONFIGURATION:
            ProcessRrcConnectionReconfiguration(packet);
            break;
        case NbIotRrcMessageType::RRC_CONNECTION_RELEASE:
            ProcessRrcConnectionRelease(packet);
            break;
        default:
            NS_LOG_WARN("Unhandled RRC message type");
            break;
    }
}

void
NbIotUeRrc::ProcessMib(const NbIotMib& mib)
{
    NS_LOG_FUNCTION(this);
    
    m_mib = mib;
    m_hasMib = true;
    
    NS_LOG_INFO("UE acquired MIB-NB: SFN=" << mib.systemFrameNumber
                << ", hyperSFN=" << static_cast<int>(mib.hyperFrameNumber));
}

void
NbIotUeRrc::ProcessSib1(const NbIotSib1& sib1)
{
    NS_LOG_FUNCTION(this);
    
    m_sib1 = sib1;
    m_hasSib1 = true;
    m_cellId = sib1.cellId;
    
    NS_LOG_INFO("UE acquired SIB1-NB: cellId=" << sib1.cellId
                << ", TAC=" << static_cast<int>(sib1.trackingAreaCode));
}

void
NbIotUeRrc::ProcessSib2(const NbIotSib2& sib2)
{
    NS_LOG_FUNCTION(this);
    
    m_sib2 = sib2;
    m_hasSib2 = true;
    
    NS_LOG_INFO("UE acquired SIB2-NB");
}

void
NbIotUeRrc::ProcessRrcConnectionSetup(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    // Stop T300
    if (m_t300.IsPending())
    {
        Simulator::Cancel(m_t300);
    }
    
    NS_LOG_INFO("UE " << m_imsi << " received RRCConnectionSetup");
    
    // Parse message (simplified)
    uint8_t msgBuf[256];
    packet->CopyData(msgBuf, std::min(packet->GetSize(), static_cast<uint32_t>(256)));
    
    // Extract C-RNTI (bytes 1-2)
    m_rnti = (msgBuf[1] << 8) | msgBuf[2];
    
    // Setup SRB1
    auto pdcpSrb1 = CreateObject<NbIotPdcpSrb>();
    pdcpSrb1->SetBearerConfig(m_rnti, 1, false);
    m_pdcpEntities[1] = pdcpSrb1;
    
    // Move to connected state
    SwitchToState(NbIotRrcState::RRC_CONNECTED);
    
    // Send RRC Connection Setup Complete
    SendRrcConnectionSetupComplete();
}

void
NbIotUeRrc::SendRrcConnectionSetupComplete()
{
    NS_LOG_FUNCTION(this);
    
    uint8_t msgBuf[32];
    msgBuf[0] = static_cast<uint8_t>(NbIotRrcMessageType::RRC_CONNECTION_SETUP_COMPLETE);
    // Add dedicated info NAS (simplified)
    msgBuf[1] = 0;  // Placeholder for NAS message
    
    Ptr<Packet> rrcMsg = Create<Packet>(msgBuf, 2);
    
    NS_LOG_INFO("UE " << m_imsi << " sending RRCConnectionSetupComplete");
    
    // Send via SRB1
    if (m_mac)
    {
        m_mac->TransmitSdu(1, rrcMsg);
    }
    
    m_connectionTrace(m_imsi, m_rnti, m_state);
    
    if (!m_connEstCallback.IsNull())
    {
        m_connEstCallback(m_rnti);
    }
}

void
NbIotUeRrc::ProcessRrcConnectionReconfiguration(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    NS_LOG_INFO("UE " << m_imsi << " received RRCConnectionReconfiguration");
    
    // Parse and apply configuration (simplified)
    uint8_t msgBuf[256];
    packet->CopyData(msgBuf, std::min(packet->GetSize(), static_cast<uint32_t>(256)));
    
    // Setup DRB (simplified: assume DRB1 setup)
    auto pdcpDrb1 = CreateObject<NbIotPdcpDrb>();
    pdcpDrb1->SetBearerConfig(m_rnti, 3, true);  // LCID 3 for DRB1
    m_pdcpEntities[3] = pdcpDrb1;
    
    // Send RRC Connection Reconfiguration Complete
    uint8_t respBuf[2];
    respBuf[0] = static_cast<uint8_t>(NbIotRrcMessageType::RRC_CONNECTION_RECONFIGURATION_COMPLETE);
    respBuf[1] = 0;
    
    Ptr<Packet> response = Create<Packet>(respBuf, 2);
    
    if (m_mac)
    {
        m_mac->TransmitSdu(1, response);  // SRB1
    }
    
    NS_LOG_INFO("UE " << m_imsi << " completed reconfiguration, DRB established");
}

void
NbIotUeRrc::ProcessRrcConnectionRelease(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);
    
    NS_LOG_INFO("UE " << m_imsi << " received RRCConnectionRelease");
    
    SwitchToState(NbIotRrcState::RRC_IDLE);
    m_pdcpEntities.clear();
}

void
NbIotUeRrc::NotifyConnectionEstablished(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    m_rnti = rnti;
    // This is called from MAC when RACH completes
}

void
NbIotUeRrc::SendData(Ptr<Packet> packet, uint8_t bearerId)
{
    NS_LOG_FUNCTION(this << packet << static_cast<int>(bearerId));
    
    if (m_state != NbIotRrcState::RRC_CONNECTED)
    {
        NS_LOG_WARN("Cannot send data: not connected");
        return;
    }
    
    uint8_t lcid = bearerId + 2;  // LCID = DRB ID + 2
    
    auto it = m_pdcpEntities.find(lcid);
    if (it != m_pdcpEntities.end())
    {
        it->second->TransmitSdu(packet);
    }
    else
    {
        // Send directly via MAC if no PDCP configured
        if (m_mac)
        {
            m_mac->TransmitSdu(lcid, packet);
        }
    }
}

void
NbIotUeRrc::T300Expiry()
{
    NS_LOG_FUNCTION(this);
    
    NS_LOG_WARN("UE " << m_imsi << " T300 expired");
    
    // Retry connection
    if (m_connectionAttempts < m_maxConnectionAttempts)
    {
        SendRrcConnectionRequest();
    }
    else
    {
        NS_LOG_WARN("UE " << m_imsi << " connection failed after max attempts");
    }
}

void
NbIotUeRrc::T301Expiry()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_WARN("UE " << m_imsi << " T301 expired");
}

void
NbIotUeRrc::T311Expiry()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_WARN("UE " << m_imsi << " T311 expired");
}

void
NbIotUeRrc::SwitchToState(NbIotRrcState newState)
{
    NS_LOG_FUNCTION(this << RrcStateToString(newState));
    
    NbIotRrcState oldState = m_state;
    m_state = newState;
    
    m_stateTransitionTrace(m_cellId, oldState, newState);
    
    NS_LOG_INFO("UE " << m_imsi << " RRC state: " << RrcStateToString(oldState)
                << " -> " << RrcStateToString(newState));
}

// ====================== NbIotEnbRrc ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotEnbRrc);

TypeId
NbIotEnbRrc::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotEnbRrc")
        .SetParent<NbIotRrc>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotEnbRrc>()
        .AddAttribute("InactivityTimer",
                      "Inactivity timer value",
                      TimeValue(Seconds(60)),
                      MakeTimeAccessor(&NbIotEnbRrc::m_inactivityTimer),
                      MakeTimeChecker())
        .AddTraceSource("UeConnection",
                        "UE connection trace",
                        MakeTraceSourceAccessor(&NbIotEnbRrc::m_ueConnectionTrace),
                        "ns3::NbIotEnbRrc::UeConnectionTracedCallback")
        .AddTraceSource("UeRelease",
                        "UE release trace",
                        MakeTraceSourceAccessor(&NbIotEnbRrc::m_ueReleaseTrace),
                        "ns3::NbIotEnbRrc::UeReleaseTracedCallback");
    return tid;
}

NbIotEnbRrc::NbIotEnbRrc()
{
    NS_LOG_FUNCTION(this);
    
    // Initialize MIB
    m_mib.systemFrameNumber = 0;
    m_mib.hyperFrameNumber = 0;
    m_mib.schedulingInfoSib1 = 4;
    m_mib.operationModeInfo = false;
    m_mib.spare = 0;
    m_mib.deploymentMode = NbIotDeploymentMode::STANDALONE;
}

NbIotEnbRrc::~NbIotEnbRrc()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotEnbRrc::DoDispose()
{
    m_mac = nullptr;
    m_phy = nullptr;
    m_ueContexts.clear();
    m_pdcpEntities.clear();
    
    NbIotRrc::DoDispose();
}

void NbIotEnbRrc::SetMac(Ptr<NbIotEnbMac> mac) { m_mac = mac; }
void NbIotEnbRrc::SetPhy(Ptr<NbIotEnbPhy> phy) { m_phy = phy; }
void NbIotEnbRrc::SetSib1(const NbIotSib1& sib1) { m_sib1 = sib1; m_cellId = sib1.cellId; }
void NbIotEnbRrc::SetSib2(const NbIotSib2& sib2) { m_sib2 = sib2; }
NbIotMib NbIotEnbRrc::GetMib() const { return m_mib; }

void
NbIotEnbRrc::SetDataReceivedCallback(DataReceivedCallback cb)
{
    m_dataReceivedCallback = cb;
}

void
NbIotEnbRrc::NotifyNewUe(uint16_t rnti, NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << rnti << CoverageClassToString(coverageClass));
    
    auto context = Create<NbIotUeRrcContext>();
    context->rnti = rnti;
    context->state = NbIotRrcState::RRC_IDLE;  // Will become CONNECTED after setup
    context->coverageClass = coverageClass;
    context->lastActivity = Simulator::Now();
    context->securityActivated = false;
    
    m_ueContexts[rnti] = context;
    
    NS_LOG_INFO("eNB " << m_cellId << " added UE " << rnti);
}

void
NbIotEnbRrc::RemoveUe(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    m_ueContexts.erase(rnti);
    m_pdcpEntities.erase(rnti);
    
    if (m_mac)
    {
        m_mac->RemoveUe(rnti);
    }
    
    m_ueReleaseTrace(rnti);
    
    NS_LOG_INFO("eNB " << m_cellId << " removed UE " << rnti);
}

Ptr<NbIotUeRrcContext>
NbIotEnbRrc::GetUeContext(uint16_t rnti)
{
    auto it = m_ueContexts.find(rnti);
    if (it != m_ueContexts.end())
    {
        return it->second;
    }
    return nullptr;
}

std::map<uint16_t, Ptr<NbIotUeRrcContext>>&
NbIotEnbRrc::GetUeContexts()
{
    return m_ueContexts;
}

uint32_t
NbIotEnbRrc::GetNumConnectedUes() const
{
    uint32_t count = 0;
    for (const auto& [rnti, context] : m_ueContexts)
    {
        if (context && context->state == NbIotRrcState::RRC_CONNECTED)
        {
            ++count;
        }
    }
    return count;
}

void
NbIotEnbRrc::ReceiveRrcMessage(uint16_t rnti, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << rnti << packet);
    
    uint8_t msgBuf[256];
    packet->CopyData(msgBuf, std::min(packet->GetSize(), static_cast<uint32_t>(256)));
    
    auto msgType = static_cast<NbIotRrcMessageType>(msgBuf[0]);
    
    NS_LOG_DEBUG("eNB received RRC message from UE " << rnti << ": "
                 << RrcMessageTypeToString(msgType));
    
    // Update activity
    auto context = GetUeContext(rnti);
    if (context)
    {
        context->lastActivity = Simulator::Now();
    }
    
    switch (msgType)
    {
        case NbIotRrcMessageType::RRC_CONNECTION_REQUEST:
            ProcessRrcConnectionRequest(rnti, packet);
            break;
        case NbIotRrcMessageType::RRC_CONNECTION_SETUP_COMPLETE:
            ProcessRrcConnectionSetupComplete(rnti, packet);
            break;
        default:
            NS_LOG_WARN("Unhandled RRC message type from UE " << rnti);
            break;
    }
}

void
NbIotEnbRrc::ProcessRrcConnectionRequest(uint16_t rnti, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << rnti << packet);
    
    uint8_t msgBuf[256];
    packet->CopyData(msgBuf, std::min(packet->GetSize(), static_cast<uint32_t>(256)));
    
    // Extract establishment cause and UE identity
    auto estCause = static_cast<NbIotRrcEstablishmentCause>(msgBuf[1]);
    uint64_t ueId = (static_cast<uint64_t>(msgBuf[2]) << 24) |
                    (static_cast<uint64_t>(msgBuf[3]) << 16) |
                    (static_cast<uint64_t>(msgBuf[4]) << 8) |
                    static_cast<uint64_t>(msgBuf[5]);
    
    NS_LOG_INFO("eNB " << m_cellId << " received RRCConnectionRequest from UE "
                << rnti << ", cause=" << static_cast<int>(estCause));
    
    // Update context
    auto context = GetUeContext(rnti);
    if (context)
    {
        context->imsi = ueId;
    }
    
    // Send RRC Connection Setup
    SendRrcConnectionSetup(rnti);
}

void
NbIotEnbRrc::SendRrcConnectionSetup(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    // Build RRC Connection Setup message
    uint8_t msgBuf[32];
    msgBuf[0] = static_cast<uint8_t>(NbIotRrcMessageType::RRC_CONNECTION_SETUP);
    msgBuf[1] = (rnti >> 8) & 0xFF;  // C-RNTI
    msgBuf[2] = rnti & 0xFF;
    // SRB1 configuration (simplified)
    msgBuf[3] = 0;  // RLC-Config (AM)
    msgBuf[4] = 0;  // logicalChannelConfig
    
    Ptr<Packet> rrcMsg = Create<Packet>(msgBuf, 5);
    
    NS_LOG_INFO("eNB " << m_cellId << " sending RRCConnectionSetup to UE " << rnti);
    
    // Send via MAC
    if (m_mac)
    {
        m_mac->TransmitSdu(rnti, 0, rrcMsg);  // CCCH
    }
}

void
NbIotEnbRrc::ProcessRrcConnectionSetupComplete(uint16_t rnti, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << rnti << packet);
    
    NS_LOG_INFO("eNB " << m_cellId << " received RRCConnectionSetupComplete from UE " << rnti);
    
    // Update UE state
    auto context = GetUeContext(rnti);
    if (context)
    {
        context->state = NbIotRrcState::RRC_CONNECTED;
        
        // Setup SRB1 PDCP
        auto pdcpSrb1 = CreateObject<NbIotPdcpSrb>();
        pdcpSrb1->SetBearerConfig(rnti, 1, false);
        m_pdcpEntities[rnti][1] = pdcpSrb1;
        
        m_ueConnectionTrace(rnti, NbIotRrcState::RRC_CONNECTED);
    }
    
    // In real implementation, would forward NAS message to MME
    // and wait for initial context setup
    
    // For simulation, immediately setup DRB
    SendRrcConnectionReconfiguration(rnti);
}

void
NbIotEnbRrc::SendRrcConnectionReconfiguration(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    // Build RRC Connection Reconfiguration message
    uint8_t msgBuf[32];
    msgBuf[0] = static_cast<uint8_t>(NbIotRrcMessageType::RRC_CONNECTION_RECONFIGURATION);
    // DRB configuration (simplified)
    msgBuf[1] = 1;  // DRB ID
    msgBuf[2] = 3;  // LCID
    msgBuf[3] = 0;  // RLC config (UM)
    msgBuf[4] = 12; // PDCP SN length
    
    Ptr<Packet> rrcMsg = Create<Packet>(msgBuf, 5);
    
    NS_LOG_INFO("eNB " << m_cellId << " sending RRCConnectionReconfiguration to UE " << rnti);
    
    if (m_mac)
    {
        m_mac->TransmitSdu(rnti, 1, rrcMsg);  // SRB1
    }
    
    // Setup DRB PDCP
    auto pdcpDrb1 = CreateObject<NbIotPdcpDrb>();
    pdcpDrb1->SetBearerConfig(rnti, 3, true);
    m_pdcpEntities[rnti][3] = pdcpDrb1;
}

void
NbIotEnbRrc::SetupDataRadioBearer(uint16_t rnti, const NbIotRadioBearerConfig& bearerConfig)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<int>(bearerConfig.drbId));
    
    auto context = GetUeContext(rnti);
    if (!context)
    {
        NS_LOG_WARN("UE " << rnti << " not found");
        return;
    }
    
    context->bearers.push_back(bearerConfig);
    
    // Setup PDCP entity
    auto pdcp = CreateObject<NbIotPdcpDrb>();
    pdcp->SetBearerConfig(rnti, bearerConfig.lcId, true);
    pdcp->SetSnLength(bearerConfig.pdcpSnLength);
    pdcp->SetHeaderCompression(bearerConfig.headerCompression);
    m_pdcpEntities[rnti][bearerConfig.lcId] = pdcp;
    
    NS_LOG_INFO("eNB " << m_cellId << " setup DRB " << static_cast<int>(bearerConfig.drbId)
                << " for UE " << rnti);
}

void
NbIotEnbRrc::ReleaseConnection(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    // Build RRC Connection Release message
    uint8_t msgBuf[4];
    msgBuf[0] = static_cast<uint8_t>(NbIotRrcMessageType::RRC_CONNECTION_RELEASE);
    msgBuf[1] = 0;  // releaseCause
    
    Ptr<Packet> rrcMsg = Create<Packet>(msgBuf, 2);
    
    NS_LOG_INFO("eNB " << m_cellId << " sending RRCConnectionRelease to UE " << rnti);
    
    if (m_mac)
    {
        m_mac->TransmitSdu(rnti, 1, rrcMsg);  // SRB1
    }
    
    // Schedule removal after some time
    Simulator::Schedule(MilliSeconds(100), &NbIotEnbRrc::RemoveUe, this, rnti);
}

void
NbIotEnbRrc::SendPaging(uint64_t imsi, NbIotRrcEstablishmentCause cause)
{
    NS_LOG_FUNCTION(this << imsi << static_cast<int>(cause));
    
    // Build paging message
    uint8_t msgBuf[16];
    msgBuf[0] = static_cast<uint8_t>(NbIotRrcMessageType::PAGING);
    msgBuf[1] = (imsi >> 24) & 0xFF;
    msgBuf[2] = (imsi >> 16) & 0xFF;
    msgBuf[3] = (imsi >> 8) & 0xFF;
    msgBuf[4] = imsi & 0xFF;
    msgBuf[5] = static_cast<uint8_t>(cause);
    
    Ptr<Packet> pagingMsg = Create<Packet>(msgBuf, 6);
    
    NS_LOG_INFO("eNB " << m_cellId << " sending paging for IMSI " << imsi);
    
    // Transmit via PHY on P-RNTI
    if (m_phy)
    {
        // Would transmit on NPDSCH with P-RNTI
    }
}

void
NbIotEnbRrc::SendData(uint16_t rnti, uint8_t bearerId, Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<int>(bearerId) << packet);
    
    auto context = GetUeContext(rnti);
    if (!context || context->state != NbIotRrcState::RRC_CONNECTED)
    {
        NS_LOG_WARN("Cannot send data to UE " << rnti << ": not connected");
        return;
    }
    
    uint8_t lcid = bearerId + 2;
    
    auto ueIt = m_pdcpEntities.find(rnti);
    if (ueIt != m_pdcpEntities.end())
    {
        auto bearerIt = ueIt->second.find(lcid);
        if (bearerIt != ueIt->second.end())
        {
            bearerIt->second->TransmitSdu(packet);
            return;
        }
    }
    
    // Send directly via MAC if no PDCP
    if (m_mac)
    {
        m_mac->TransmitSdu(rnti, lcid, packet);
    }
}

} // namespace ns3
