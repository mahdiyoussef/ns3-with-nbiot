/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Net Device implementation
 */

#include "nbiot-net-device.h"
#include "nbiot-ue-phy.h"
#include "nbiot-enb-phy.h"
#include "nbiot-ue-mac.h"
#include "nbiot-enb-mac.h"
#include "nbiot-rlc.h"
#include "nbiot-pdcp.h"
#include "nbiot-rrc.h"
#include "nbiot-power-saving.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/node.h>
#include <ns3/channel.h>
#include <ns3/pointer.h>
#include <ns3/uinteger.h>
#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/boolean.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotNetDevice");

// ====================== NbIotNetDevice ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotNetDevice);

TypeId
NbIotNetDevice::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotNetDevice")
        .SetParent<NetDevice>()
        .SetGroupName("NbIot")
        .AddAttribute("Mtu",
                      "The Maximum Transmission Unit",
                      UintegerValue(1500),
                      MakeUintegerAccessor(&NbIotNetDevice::SetMtu,
                                           &NbIotNetDevice::GetMtu),
                      MakeUintegerChecker<uint16_t>())
        .AddAttribute("Earfcn",
                      "The carrier EARFCN",
                      UintegerValue(0),
                      MakeUintegerAccessor(&NbIotNetDevice::m_earfcn),
                      MakeUintegerChecker<uint32_t>())
        .AddAttribute("DeploymentMode",
                      "NB-IoT deployment mode",
                      EnumValue(NbIotDeploymentMode::STANDALONE),
                      MakeEnumAccessor<NbIotDeploymentMode>(&NbIotNetDevice::m_deploymentMode),
                      MakeEnumChecker(NbIotDeploymentMode::STANDALONE, "Standalone",
                                      NbIotDeploymentMode::GUARD_BAND, "GuardBand",
                                      NbIotDeploymentMode::IN_BAND, "InBand"));
    return tid;
}

NbIotNetDevice::NbIotNetDevice()
    : m_node(nullptr)
    , m_ifIndex(0)
    , m_mtu(1500)
    , m_linkUp(false)
    , m_earfcn(0)
    , m_deploymentMode(NbIotDeploymentMode::STANDALONE)
{
    NS_LOG_FUNCTION(this);
}

NbIotNetDevice::~NbIotNetDevice()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotNetDevice::DoDispose()
{
    m_node = nullptr;
    m_rxCallback.Nullify();
    m_promiscRxCallback.Nullify();
    NetDevice::DoDispose();
}

void
NbIotNetDevice::SetIfIndex(const uint32_t index)
{
    m_ifIndex = index;
}

uint32_t
NbIotNetDevice::GetIfIndex() const
{
    return m_ifIndex;
}

Ptr<Channel>
NbIotNetDevice::GetChannel() const
{
    // Subclasses should override to return their specific channel
    return nullptr;
}

void
NbIotNetDevice::SetAddress(Address address)
{
    m_address = Mac48Address::ConvertFrom(address);
}

Address
NbIotNetDevice::GetAddress() const
{
    return m_address;
}

bool
NbIotNetDevice::SetMtu(const uint16_t mtu)
{
    m_mtu = mtu;
    return true;
}

uint16_t
NbIotNetDevice::GetMtu() const
{
    return m_mtu;
}

bool
NbIotNetDevice::IsLinkUp() const
{
    return m_linkUp;
}

void
NbIotNetDevice::AddLinkChangeCallback(Callback<void> callback)
{
    m_linkChangeCallbacks.ConnectWithoutContext(callback);
}

bool
NbIotNetDevice::IsBroadcast() const
{
    return false;
}

Address
NbIotNetDevice::GetBroadcast() const
{
    return Mac48Address::GetBroadcast();
}

bool
NbIotNetDevice::IsMulticast() const
{
    return false;
}

Address
NbIotNetDevice::GetMulticast(Ipv4Address multicastGroup) const
{
    return Mac48Address();
}

Address
NbIotNetDevice::GetMulticast(Ipv6Address addr) const
{
    return Mac48Address();
}

bool
NbIotNetDevice::IsBridge() const
{
    return false;
}

bool
NbIotNetDevice::IsPointToPoint() const
{
    return true;
}

bool
NbIotNetDevice::Send(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << packet << dest << protocolNumber);
    // To be implemented by subclasses
    return false;
}

bool
NbIotNetDevice::SendFrom(Ptr<Packet> packet, const Address& source, const Address& dest,
                          uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << packet << source << dest << protocolNumber);
    return false;
}

Ptr<Node>
NbIotNetDevice::GetNode() const
{
    return m_node;
}

void
NbIotNetDevice::SetNode(Ptr<Node> node)
{
    m_node = node;
}

bool
NbIotNetDevice::NeedsArp() const
{
    return false;
}

void
NbIotNetDevice::SetReceiveCallback(ReceiveCallback cb)
{
    m_rxCallback = cb;
}

void
NbIotNetDevice::SetPromiscReceiveCallback(PromiscReceiveCallback cb)
{
    m_promiscRxCallback = cb;
}

bool
NbIotNetDevice::SupportsSendFrom() const
{
    return false;
}

void
NbIotNetDevice::SetEarfcn(uint32_t earfcn)
{
    m_earfcn = earfcn;
}

uint32_t
NbIotNetDevice::GetEarfcn() const
{
    return m_earfcn;
}

void
NbIotNetDevice::SetDeploymentMode(NbIotDeploymentMode mode)
{
    m_deploymentMode = mode;
}

NbIotDeploymentMode
NbIotNetDevice::GetDeploymentMode() const
{
    return m_deploymentMode;
}

void
NbIotNetDevice::NotifyLinkChange(bool isUp)
{
    m_linkUp = isUp;
    m_linkChangeCallbacks();
}

// ====================== NbIotUeNetDevice ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotUeNetDevice);

TypeId
NbIotUeNetDevice::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotUeNetDevice")
        .SetParent<NbIotNetDevice>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotUeNetDevice>()
        .AddAttribute("Imsi",
                      "International Mobile Subscriber Identity",
                      UintegerValue(0),
                      MakeUintegerAccessor(&NbIotUeNetDevice::m_imsi),
                      MakeUintegerChecker<uint64_t>())
        .AddAttribute("NbIotUePhy",
                      "The PHY layer",
                      PointerValue(),
                      MakePointerAccessor(&NbIotUeNetDevice::m_phy),
                      MakePointerChecker<NbIotUePhy>())
        .AddAttribute("NbIotUeMac",
                      "The MAC layer",
                      PointerValue(),
                      MakePointerAccessor(&NbIotUeNetDevice::m_mac),
                      MakePointerChecker<NbIotUeMac>())
        .AddAttribute("NbIotUeRrc",
                      "The RRC layer",
                      PointerValue(),
                      MakePointerAccessor(&NbIotUeNetDevice::m_rrc),
                      MakePointerChecker<NbIotUeRrc>());
    return tid;
}

NbIotUeNetDevice::NbIotUeNetDevice()
    : m_imsi(0)
    , m_coverageClass(NbIotCoverageClass::CE_LEVEL_0)
{
    NS_LOG_FUNCTION(this);
}

NbIotUeNetDevice::~NbIotUeNetDevice()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotUeNetDevice::DoDispose()
{
    m_phy = nullptr;
    m_mac = nullptr;
    m_rlc = nullptr;
    m_pdcp = nullptr;
    m_rrc = nullptr;
    m_powerSaving = nullptr;
    NbIotNetDevice::DoDispose();
}

void
NbIotUeNetDevice::DoInitialize()
{
    NS_LOG_FUNCTION(this);

    // Initialize protocol stack layers
    if (m_phy)
    {
        m_phy->Initialize();
    }
    if (m_mac)
    {
        m_mac->Initialize();
    }
    if (m_rlc)
    {
        m_rlc->Initialize();
    }
    if (m_pdcp)
    {
        m_pdcp->Initialize();
    }
    if (m_rrc)
    {
        m_rrc->Initialize();
    }
    if (m_powerSaving)
    {
        m_powerSaving->Initialize();
    }

    NbIotNetDevice::DoInitialize();
}

void NbIotUeNetDevice::SetPhy(Ptr<NbIotUePhy> phy) { m_phy = phy; }
Ptr<NbIotUePhy> NbIotUeNetDevice::GetPhy() const { return m_phy; }
void NbIotUeNetDevice::SetMac(Ptr<NbIotUeMac> mac) { m_mac = mac; }
Ptr<NbIotUeMac> NbIotUeNetDevice::GetMac() const { return m_mac; }
void NbIotUeNetDevice::SetRlc(Ptr<NbIotRlc> rlc) { m_rlc = rlc; }
Ptr<NbIotRlc> NbIotUeNetDevice::GetRlc() const { return m_rlc; }
void NbIotUeNetDevice::SetPdcp(Ptr<NbIotPdcp> pdcp) { m_pdcp = pdcp; }
Ptr<NbIotPdcp> NbIotUeNetDevice::GetPdcp() const { return m_pdcp; }
void NbIotUeNetDevice::SetRrc(Ptr<NbIotUeRrc> rrc) { m_rrc = rrc; }
Ptr<NbIotUeRrc> NbIotUeNetDevice::GetRrc() const { return m_rrc; }
void NbIotUeNetDevice::SetImsi(uint64_t imsi) { m_imsi = imsi; }
uint64_t NbIotUeNetDevice::GetImsi() const { return m_imsi; }

void
NbIotUeNetDevice::SetPowerSavingController(Ptr<NbIotPowerSavingController> controller)
{
    m_powerSaving = controller;
}

Ptr<NbIotPowerSavingController>
NbIotUeNetDevice::GetPowerSavingController() const
{
    return m_powerSaving;
}

void
NbIotUeNetDevice::EnablePowerSaving(bool enablePsm, bool enableEdrx)
{
    NS_LOG_FUNCTION(this << enablePsm << enableEdrx);
    
    if (m_powerSaving)
    {
        m_powerSaving->Enable(enablePsm, enableEdrx);
    }
}

NbIotCoverageClass
NbIotUeNetDevice::GetCoverageClass() const
{
    // Get coverage class from PHY measurements
    if (m_phy)
    {
        // In real implementation, would use RSRP measurements
        // For now, return stored value
    }
    return m_coverageClass;
}

void
NbIotUeNetDevice::Receive(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);

    if (!m_rxCallback.IsNull())
    {
        m_rxCallback(this, packet, 0x0800, m_address);
    }
}

// ====================== NbIotEnbNetDevice ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotEnbNetDevice);

TypeId
NbIotEnbNetDevice::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotEnbNetDevice")
        .SetParent<NbIotNetDevice>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotEnbNetDevice>()
        .AddAttribute("CellId",
                      "Cell ID",
                      UintegerValue(0),
                      MakeUintegerAccessor(&NbIotEnbNetDevice::m_cellId),
                      MakeUintegerChecker<uint16_t>())
        .AddAttribute("TxPower",
                      "Transmission power in dBm",
                      DoubleValue(43.0),
                      MakeDoubleAccessor(&NbIotEnbNetDevice::m_txPower),
                      MakeDoubleChecker<double>())
        .AddAttribute("NbIotEnbPhy",
                      "The PHY layer",
                      PointerValue(),
                      MakePointerAccessor(&NbIotEnbNetDevice::m_phy),
                      MakePointerChecker<NbIotEnbPhy>())
        .AddAttribute("NbIotEnbMac",
                      "The MAC layer",
                      PointerValue(),
                      MakePointerAccessor(&NbIotEnbNetDevice::m_mac),
                      MakePointerChecker<NbIotEnbMac>())
        .AddAttribute("NbIotEnbRrc",
                      "The RRC layer",
                      PointerValue(),
                      MakePointerAccessor(&NbIotEnbNetDevice::m_rrc),
                      MakePointerChecker<NbIotEnbRrc>());
    return tid;
}

NbIotEnbNetDevice::NbIotEnbNetDevice()
    : m_cellId(0)
    , m_antennaConfig(1)
    , m_txPower(43.0)
{
    NS_LOG_FUNCTION(this);
}

NbIotEnbNetDevice::~NbIotEnbNetDevice()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotEnbNetDevice::DoDispose()
{
    m_phy = nullptr;
    m_mac = nullptr;
    m_rlc = nullptr;
    m_pdcp = nullptr;
    m_rrc = nullptr;
    NbIotNetDevice::DoDispose();
}

void
NbIotEnbNetDevice::DoInitialize()
{
    NS_LOG_FUNCTION(this);

    // Initialize protocol stack layers
    if (m_phy)
    {
        m_phy->Initialize();
    }
    if (m_mac)
    {
        m_mac->Initialize();
    }
    if (m_rlc)
    {
        m_rlc->Initialize();
    }
    if (m_pdcp)
    {
        m_pdcp->Initialize();
    }
    if (m_rrc)
    {
        m_rrc->Initialize();
    }

    // Set link as up when initialized
    NotifyLinkChange(true);

    NbIotNetDevice::DoInitialize();
}

void NbIotEnbNetDevice::SetPhy(Ptr<NbIotEnbPhy> phy) { m_phy = phy; }
Ptr<NbIotEnbPhy> NbIotEnbNetDevice::GetPhy() const { return m_phy; }
void NbIotEnbNetDevice::SetMac(Ptr<NbIotEnbMac> mac) { m_mac = mac; }
Ptr<NbIotEnbMac> NbIotEnbNetDevice::GetMac() const { return m_mac; }
void NbIotEnbNetDevice::SetRlc(Ptr<NbIotRlc> rlc) { m_rlc = rlc; }
Ptr<NbIotRlc> NbIotEnbNetDevice::GetRlc() const { return m_rlc; }
void NbIotEnbNetDevice::SetPdcp(Ptr<NbIotPdcp> pdcp) { m_pdcp = pdcp; }
Ptr<NbIotPdcp> NbIotEnbNetDevice::GetPdcp() const { return m_pdcp; }
void NbIotEnbNetDevice::SetRrc(Ptr<NbIotEnbRrc> rrc) { m_rrc = rrc; }
Ptr<NbIotEnbRrc> NbIotEnbNetDevice::GetRrc() const { return m_rrc; }
void NbIotEnbNetDevice::SetCellId(uint16_t cellId) { m_cellId = cellId; }
uint16_t NbIotEnbNetDevice::GetCellId() const { return m_cellId; }

uint32_t
NbIotEnbNetDevice::GetNumConnectedUes() const
{
    if (m_rrc)
    {
        return m_rrc->GetNumConnectedUes();
    }
    return 0;
}

void
NbIotEnbNetDevice::SetAntennaConfiguration(uint8_t numPorts)
{
    m_antennaConfig = numPorts;
}

uint8_t
NbIotEnbNetDevice::GetAntennaConfiguration() const
{
    return m_antennaConfig;
}

void
NbIotEnbNetDevice::SetTxPower(double power)
{
    m_txPower = power;
    if (m_phy)
    {
        m_phy->SetTxPower(power);
    }
}

double
NbIotEnbNetDevice::GetTxPower() const
{
    return m_txPower;
}

void
NbIotEnbNetDevice::Receive(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);

    // Forward to higher layers
    if (!m_rxCallback.IsNull())
    {
        m_rxCallback(this, packet, 0x0800, m_address);
    }
}

} // namespace ns3
