/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT RRC Layer Header
 *
 * References:
 * - 3GPP TS 36.331 RRC protocol specification
 * - 3GPP TS 36.300 E-UTRA overall description
 */

#ifndef NBIOT_RRC_H
#define NBIOT_RRC_H

#include "nbiot-common.h"

#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/simple-ref-count.h>
#include <ns3/packet.h>
#include <ns3/traced-callback.h>
#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/ipv4-address.h>

#include <map>
#include <vector>
#include <set>

namespace ns3 {

class NbIotUeMac;
class NbIotEnbMac;
class NbIotUePhy;
class NbIotEnbPhy;
class NbIotPdcp;

// NbIotRrcState is defined in nbiot-common.h

/**
 * \ingroup nbiot
 *
 * \brief RRC connection establishment cause
 */
enum class NbIotRrcEstablishmentCause : uint8_t
{
    MT_ACCESS,              ///< Mobile terminated access
    MO_SIGNALLING,          ///< Mobile originated signalling
    MO_DATA,                ///< Mobile originated data
    MO_EXCEPTION_DATA,      ///< Exception data (low priority)
    DELAY_TOLERANT_ACCESS,  ///< Delay tolerant
    HIGH_PRIORITY_ACCESS    ///< High priority access
};

/**
 * \ingroup nbiot
 *
 * \brief RRC message type
 */
enum class NbIotRrcMessageType : uint8_t
{
    // Downlink messages
    MASTER_INFORMATION_BLOCK,
    SYSTEM_INFORMATION_BLOCK_1,
    SYSTEM_INFORMATION_BLOCK_2,
    RRC_CONNECTION_SETUP,
    RRC_CONNECTION_RECONFIGURATION,
    RRC_CONNECTION_RELEASE,
    RRC_CONNECTION_REJECT,
    PAGING,
    DL_INFORMATION_TRANSFER,
    SECURITY_MODE_COMMAND,
    UE_CAPABILITY_ENQUIRY,
    
    // Uplink messages
    RRC_CONNECTION_REQUEST,
    RRC_CONNECTION_SETUP_COMPLETE,
    RRC_CONNECTION_RECONFIGURATION_COMPLETE,
    SECURITY_MODE_COMPLETE,
    SECURITY_MODE_FAILURE,
    UE_CAPABILITY_INFORMATION,
    UL_INFORMATION_TRANSFER,
    RRC_CONNECTION_RESUME_REQUEST,
    RRC_CONNECTION_RESUME_COMPLETE
};

/**
 * \ingroup nbiot
 *
 * \brief System Information Block Type 1 for NB-IoT
 */
struct NbIotSib1
{
    uint16_t cellId;
    uint8_t trackingAreaCode;
    uint8_t freqBandIndicator;
    NbIotDeploymentMode deploymentMode;
    uint8_t schedulingInfoSib2;
    bool cellBarred;
    bool intraFreqReselection;
    
    // NB-IoT specific
    uint8_t eutraControlRegionSize;
    bool attachWithoutPdnConnectivity;
    uint8_t nrsSequenceInfo;
};

/**
 * \ingroup nbiot
 *
 * \brief System Information Block Type 2 for NB-IoT
 */
struct NbIotSib2
{
    // RACH configuration
    uint8_t preambleInitialReceivedTargetPower;
    uint8_t powerRampingStep;
    uint8_t preambleTransMax;
    uint8_t raResponseWindowSize;
    uint8_t macContentionResolutionTimer;
    
    // UL power control
    int8_t p0NominalNpusch;
    uint8_t alphaNpusch;
    
    // Paging
    uint8_t defaultPagingCycle;
    uint8_t nB;
    
    // Coverage enhancement
    std::array<uint8_t, 3> nprach_ce_level_cfg;  ///< NPRACH config per CE level
};

/**
 * \ingroup nbiot
 *
 * \brief Radio bearer configuration
 */
struct NbIotRadioBearerConfig
{
    uint8_t drbId;
    uint8_t lcId;
    uint8_t lcgId;
    NbIotRlcMode rlcMode;
    uint8_t pdcpSnLength;
    bool headerCompression;
    int8_t priority;
    uint8_t prioritizedBitRate;
};

/**
 * \ingroup nbiot
 *
 * \brief UE context at eNB
 */
struct NbIotUeRrcContext : public SimpleRefCount<NbIotUeRrcContext>
{
    uint16_t rnti;
    uint64_t imsi;
    NbIotRrcState state;
    NbIotCoverageClass coverageClass;
    uint8_t cqi;
    std::vector<NbIotRadioBearerConfig> bearers;
    Time lastActivity;
    bool securityActivated;
    uint32_t nasSequenceNumber;
    Ipv4Address ueIpAddress;
};

/**
 * \ingroup nbiot
 *
 * \brief Base class for NB-IoT RRC
 */
class NbIotRrc : public Object
{
public:
    static TypeId GetTypeId();
    
    NbIotRrc();
    ~NbIotRrc() override;
    
    /**
     * \brief Set cell ID
     * \param cellId Cell identifier
     */
    void SetCellId(uint16_t cellId);
    
    /**
     * \brief Get cell ID
     * \return Cell identifier
     */
    uint16_t GetCellId() const;
    
protected:
    void DoDispose() override;
    
    uint16_t m_cellId;      ///< Cell identifier
    
    /// Traces
    TracedCallback<uint16_t, NbIotRrcState, NbIotRrcState> m_stateTransitionTrace;
};

/**
 * \ingroup nbiot
 *
 * \brief NB-IoT UE RRC layer
 */
class NbIotUeRrc : public NbIotRrc
{
public:
    static TypeId GetTypeId();
    
    NbIotUeRrc();
    ~NbIotUeRrc() override;
    
    /**
     * \brief Set IMSI
     * \param imsi International Mobile Subscriber Identity
     */
    void SetImsi(uint64_t imsi);
    
    /**
     * \brief Get IMSI
     * \return IMSI
     */
    uint64_t GetImsi() const;
    
    /**
     * \brief Set C-RNTI
     * \param rnti Cell RNTI
     */
    void SetRnti(uint16_t rnti);
    
    /**
     * \brief Get C-RNTI
     * \return RNTI
     */
    uint16_t GetRnti() const;
    
    /**
     * \brief Set MAC layer
     * \param mac Pointer to UE MAC
     */
    void SetMac(Ptr<NbIotUeMac> mac);
    
    /**
     * \brief Set PHY layer
     * \param phy Pointer to UE PHY
     */
    void SetPhy(Ptr<NbIotUePhy> phy);
    
    /**
     * \brief Get current RRC state
     * \return RRC state
     */
    NbIotRrcState GetState() const;
    
    /**
     * \brief Initiate RRC connection
     * \param cause Establishment cause
     */
    void Connect(NbIotRrcEstablishmentCause cause);
    
    /**
     * \brief Disconnect from network
     */
    void Disconnect();
    
    /**
     * \brief Process received RRC message
     * \param packet RRC PDU
     */
    void ReceiveRrcMessage(Ptr<Packet> packet);
    
    /**
     * \brief Process MIB-NB
     * \param mib MIB information
     */
    void ProcessMib(const NbIotMib& mib);
    
    /**
     * \brief Process SIB1-NB
     * \param sib1 SIB1 information
     */
    void ProcessSib1(const NbIotSib1& sib1);
    
    /**
     * \brief Process SIB2-NB
     * \param sib2 SIB2 information
     */
    void ProcessSib2(const NbIotSib2& sib2);
    
    /**
     * \brief Notify connection established
     * \param rnti Assigned C-RNTI
     */
    void NotifyConnectionEstablished(uint16_t rnti);
    
    /**
     * \brief Check if connected
     * \return True if RRC_CONNECTED
     */
    bool IsConnected() const;
    
    /**
     * \brief Send data via user plane
     * \param packet Data packet
     * \param bearerId DRB ID
     */
    void SendData(Ptr<Packet> packet, uint8_t bearerId);
    
    /**
     * \brief Receive data callback
     * \param packet Data packet
     * \param bearerId DRB ID
     */
    typedef Callback<void, Ptr<Packet>, uint8_t> DataReceivedCallback;
    
    /**
     * \brief Set data received callback
     * \param cb Callback function
     */
    void SetDataReceivedCallback(DataReceivedCallback cb);
    
    /**
     * \brief Connection established callback type
     */
    typedef Callback<void, uint16_t> ConnectionEstablishedCallback;
    
    /**
     * \brief Set connection established callback
     * \param cb Callback function
     */
    void SetConnectionEstablishedCallback(ConnectionEstablishedCallback cb);
    
protected:
    void DoDispose() override;
    
private:
    /**
     * \brief Send RRC connection request
     */
    void SendRrcConnectionRequest();
    
    /**
     * \brief Process RRC connection setup
     * \param packet Message PDU
     */
    void ProcessRrcConnectionSetup(Ptr<Packet> packet);
    
    /**
     * \brief Send RRC connection setup complete
     */
    void SendRrcConnectionSetupComplete();
    
    /**
     * \brief Process RRC connection reconfiguration
     * \param packet Message PDU
     */
    void ProcessRrcConnectionReconfiguration(Ptr<Packet> packet);
    
    /**
     * \brief Process RRC connection release
     * \param packet Message PDU
     */
    void ProcessRrcConnectionRelease(Ptr<Packet> packet);
    
    /**
     * \brief T300 timer (connection request) expiry
     */
    void T300Expiry();
    
    /**
     * \brief T301 timer (connection reestablishment) expiry
     */
    void T301Expiry();
    
    /**
     * \brief T311 timer (connection resume) expiry
     */
    void T311Expiry();
    
    /**
     * \brief Update state
     * \param newState New RRC state
     */
    void SwitchToState(NbIotRrcState newState);
    
    uint64_t m_imsi;                        ///< IMSI
    uint16_t m_rnti;                        ///< C-RNTI
    NbIotRrcState m_state;                  ///< RRC state
    NbIotRrcEstablishmentCause m_estCause;  ///< Establishment cause
    
    Ptr<NbIotUeMac> m_mac;                  ///< MAC layer
    Ptr<NbIotUePhy> m_phy;                  ///< PHY layer
    std::map<uint8_t, Ptr<NbIotPdcp>> m_pdcpEntities;  ///< PDCP per bearer
    
    // System information
    NbIotMib m_mib;
    NbIotSib1 m_sib1;
    NbIotSib2 m_sib2;
    bool m_hasMib;
    bool m_hasSib1;
    bool m_hasSib2;
    
    // Timers
    EventId m_t300;                         ///< T300 - RRC connection request
    EventId m_t301;                         ///< T301 - RRC connection reestablishment
    EventId m_t311;                         ///< T311 - RRC connection resume
    
    Time m_t300Value;
    Time m_t301Value;
    Time m_t311Value;
    
    uint8_t m_connectionAttempts;           ///< Number of connection attempts
    uint8_t m_maxConnectionAttempts;        ///< Max attempts before failure
    
    DataReceivedCallback m_dataReceivedCallback;
    ConnectionEstablishedCallback m_connEstCallback;
    
    /// Traces
    TracedCallback<uint64_t, uint16_t, NbIotRrcState> m_connectionTrace;
};

/**
 * \ingroup nbiot
 *
 * \brief NB-IoT eNB RRC layer
 */
class NbIotEnbRrc : public NbIotRrc
{
public:
    static TypeId GetTypeId();
    
    NbIotEnbRrc();
    ~NbIotEnbRrc() override;
    
    /**
     * \brief Set MAC layer
     * \param mac Pointer to eNB MAC
     */
    void SetMac(Ptr<NbIotEnbMac> mac);
    
    /**
     * \brief Set PHY layer
     * \param phy Pointer to eNB PHY
     */
    void SetPhy(Ptr<NbIotEnbPhy> phy);
    
    /**
     * \brief Configure SIB1
     * \param sib1 SIB1 configuration
     */
    void SetSib1(const NbIotSib1& sib1);
    
    /**
     * \brief Configure SIB2
     * \param sib2 SIB2 configuration
     */
    void SetSib2(const NbIotSib2& sib2);
    
    /**
     * \brief Get MIB
     * \return MIB structure
     */
    NbIotMib GetMib() const;
    
    /**
     * \brief Process received RRC message from UE
     * \param rnti UE identifier
     * \param packet RRC PDU
     */
    void ReceiveRrcMessage(uint16_t rnti, Ptr<Packet> packet);
    
    /**
     * \brief Notify new UE connected via RACH
     * \param rnti UE identifier
     * \param coverageClass Coverage class
     */
    void NotifyNewUe(uint16_t rnti, NbIotCoverageClass coverageClass);
    
    /**
     * \brief Remove UE
     * \param rnti UE identifier
     */
    void RemoveUe(uint16_t rnti);
    
    /**
     * \brief Get UE context
     * \param rnti UE identifier
     * \return Pointer to UE context or nullptr
     */
    Ptr<NbIotUeRrcContext> GetUeContext(uint16_t rnti);
    
    /**
     * \brief Get all UE contexts
     * \return Map of UE contexts
     */
    std::map<uint16_t, Ptr<NbIotUeRrcContext>>& GetUeContexts();
    
    /**
     * \brief Get number of connected UEs
     * \return Number of UEs in RRC_CONNECTED state
     */
    uint32_t GetNumConnectedUes() const;
    
    /**
     * \brief Setup data radio bearer for UE
     * \param rnti UE identifier
     * \param bearerConfig Bearer configuration
     */
    void SetupDataRadioBearer(uint16_t rnti, const NbIotRadioBearerConfig& bearerConfig);
    
    /**
     * \brief Send paging to UE
     * \param imsi UE IMSI
     * \param cause Paging cause
     */
    void SendPaging(uint64_t imsi, NbIotRrcEstablishmentCause cause);
    
    /**
     * \brief Send data to UE via user plane
     * \param rnti UE identifier
     * \param bearerId DRB ID
     * \param packet Data packet
     */
    void SendData(uint16_t rnti, uint8_t bearerId, Ptr<Packet> packet);
    
    /**
     * \brief Data received from UE callback
     */
    typedef Callback<void, uint16_t, uint8_t, Ptr<Packet>> DataReceivedCallback;
    
    /**
     * \brief Set data received callback
     * \param cb Callback function
     */
    void SetDataReceivedCallback(DataReceivedCallback cb);
    
protected:
    void DoDispose() override;
    
private:
    /**
     * \brief Process RRC connection request
     * \param rnti UE identifier
     * \param packet Message PDU
     */
    void ProcessRrcConnectionRequest(uint16_t rnti, Ptr<Packet> packet);
    
    /**
     * \brief Send RRC connection setup
     * \param rnti UE identifier
     */
    void SendRrcConnectionSetup(uint16_t rnti);
    
    /**
     * \brief Process RRC connection setup complete
     * \param rnti UE identifier
     * \param packet Message PDU
     */
    void ProcessRrcConnectionSetupComplete(uint16_t rnti, Ptr<Packet> packet);
    
    /**
     * \brief Send RRC connection reconfiguration
     * \param rnti UE identifier
     */
    void SendRrcConnectionReconfiguration(uint16_t rnti);
    
    /**
     * \brief Release RRC connection
     * \param rnti UE identifier
     */
    void ReleaseConnection(uint16_t rnti);
    
    Ptr<NbIotEnbMac> m_mac;                 ///< MAC layer
    Ptr<NbIotEnbPhy> m_phy;                 ///< PHY layer
    
    std::map<uint16_t, Ptr<NbIotUeRrcContext>> m_ueContexts;  ///< UE contexts
    std::map<uint16_t, std::map<uint8_t, Ptr<NbIotPdcp>>> m_pdcpEntities;  ///< PDCP per UE per bearer
    
    NbIotMib m_mib;
    NbIotSib1 m_sib1;
    NbIotSib2 m_sib2;
    
    Time m_inactivityTimer;                 ///< Inactivity timer value
    
    DataReceivedCallback m_dataReceivedCallback;
    
    /// Traces
    TracedCallback<uint16_t, NbIotRrcState> m_ueConnectionTrace;
    TracedCallback<uint16_t> m_ueReleaseTrace;
};

/**
 * \ingroup nbiot
 *
 * \brief Convert RRC state to string
 * \param state RRC state
 * \return String representation
 */
inline std::string RrcStateToString(NbIotRrcState state)
{
    switch (state)
    {
        case NbIotRrcState::RRC_IDLE: return "IDLE";
        case NbIotRrcState::RRC_CONNECTED: return "CONNECTED";
        case NbIotRrcState::RRC_INACTIVE: return "INACTIVE";
        default: return "Unknown";
    }
}

/**
 * \ingroup nbiot
 *
 * \brief Convert RRC message type to string
 * \param type Message type
 * \return String representation
 */
inline std::string RrcMessageTypeToString(NbIotRrcMessageType type)
{
    switch (type)
    {
        case NbIotRrcMessageType::MASTER_INFORMATION_BLOCK: return "MIB-NB";
        case NbIotRrcMessageType::SYSTEM_INFORMATION_BLOCK_1: return "SIB1-NB";
        case NbIotRrcMessageType::SYSTEM_INFORMATION_BLOCK_2: return "SIB2-NB";
        case NbIotRrcMessageType::RRC_CONNECTION_SETUP: return "RRCConnectionSetup";
        case NbIotRrcMessageType::RRC_CONNECTION_REQUEST: return "RRCConnectionRequest";
        case NbIotRrcMessageType::RRC_CONNECTION_SETUP_COMPLETE: return "RRCConnectionSetupComplete";
        case NbIotRrcMessageType::RRC_CONNECTION_RECONFIGURATION: return "RRCConnectionReconfiguration";
        case NbIotRrcMessageType::RRC_CONNECTION_RELEASE: return "RRCConnectionRelease";
        default: return "Unknown";
    }
}

} // namespace ns3

#endif /* NBIOT_RRC_H */
