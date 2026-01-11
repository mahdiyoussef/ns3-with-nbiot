/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Net Device classes
 */

#ifndef NBIOT_NET_DEVICE_H
#define NBIOT_NET_DEVICE_H

#include "nbiot-common.h"

#include <ns3/net-device.h>
#include <ns3/traced-callback.h>
#include <ns3/callback.h>
#include <ns3/mac48-address.h>
#include <ns3/ipv4-address.h>
#include <ns3/packet.h>

namespace ns3 {

class Node;
class Channel;
class NbIotPhy;
class NbIotMac;
class NbIotRlc;
class NbIotPdcp;
class NbIotRrc;
class NbIotUePhy;
class NbIotEnbPhy;
class NbIotUeMac;
class NbIotEnbMac;
class NbIotUeRrc;
class NbIotEnbRrc;
class NbIotPsmManager;
class NbIotEdrxManager;
class NbIotPowerSavingController;
class NbIotSpectrumChannel;

/**
 * \ingroup nbiot
 * \brief Base class for NB-IoT network devices
 *
 * Abstract base class providing common functionality for
 * NB-IoT UE and eNB network devices.
 */
class NbIotNetDevice : public NetDevice
{
public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    NbIotNetDevice();
    virtual ~NbIotNetDevice();

    // Inherited from NetDevice
    void SetIfIndex(const uint32_t index) override;
    uint32_t GetIfIndex() const override;
    Ptr<Channel> GetChannel() const override;
    void SetAddress(Address address) override;
    Address GetAddress() const override;
    bool SetMtu(const uint16_t mtu) override;
    uint16_t GetMtu() const override;
    bool IsLinkUp() const override;
    void AddLinkChangeCallback(Callback<void> callback) override;
    bool IsBroadcast() const override;
    Address GetBroadcast() const override;
    bool IsMulticast() const override;
    Address GetMulticast(Ipv4Address multicastGroup) const override;
    Address GetMulticast(Ipv6Address addr) const override;
    bool IsBridge() const override;
    bool IsPointToPoint() const override;
    bool Send(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber) override;
    bool SendFrom(Ptr<Packet> packet, const Address& source, const Address& dest,
                  uint16_t protocolNumber) override;
    Ptr<Node> GetNode() const override;
    void SetNode(Ptr<Node> node) override;
    bool NeedsArp() const override;
    void SetReceiveCallback(ReceiveCallback cb) override;
    void SetPromiscReceiveCallback(PromiscReceiveCallback cb) override;
    bool SupportsSendFrom() const override;

    /**
     * \brief Set the carrier frequency
     * \param earfcn the EARFCN
     */
    void SetEarfcn(uint32_t earfcn);

    /**
     * \brief Get the carrier frequency
     * \return the EARFCN
     */
    uint32_t GetEarfcn() const;

    /**
     * \brief Set the deployment mode
     * \param mode deployment mode (standalone, guard-band, in-band)
     */
    void SetDeploymentMode(NbIotDeploymentMode mode);

    /**
     * \brief Get the deployment mode
     * \return deployment mode
     */
    NbIotDeploymentMode GetDeploymentMode() const;

    /**
     * \brief Receive a packet from the lower layers
     * \param packet the packet
     */
    virtual void Receive(Ptr<Packet> packet) = 0;

protected:
    void DoDispose() override;

    /**
     * \brief Notify link state change
     * \param isUp true if link is up
     */
    void NotifyLinkChange(bool isUp);

    Ptr<Node> m_node;                           //!< The node
    uint32_t m_ifIndex;                         //!< Interface index
    uint16_t m_mtu;                             //!< MTU
    bool m_linkUp;                              //!< Link state
    Mac48Address m_address;                     //!< MAC address
    uint32_t m_earfcn;                          //!< Carrier EARFCN
    NbIotDeploymentMode m_deploymentMode;       //!< Deployment mode

    ReceiveCallback m_rxCallback;               //!< Receive callback
    PromiscReceiveCallback m_promiscRxCallback; //!< Promiscuous receive callback

    TracedCallback<> m_linkChangeCallbacks;     //!< Link change callbacks
};

/**
 * \ingroup nbiot
 * \brief NB-IoT UE network device
 *
 * Implements UE-side network device with protocol stack:
 * PHY -> MAC -> RLC -> PDCP -> RRC
 */
class NbIotUeNetDevice : public NbIotNetDevice
{
public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    NbIotUeNetDevice();
    virtual ~NbIotUeNetDevice();

    /**
     * \brief Set the UE PHY
     * \param phy the PHY layer
     */
    void SetPhy(Ptr<NbIotUePhy> phy);

    /**
     * \brief Get the UE PHY
     * \return the PHY layer
     */
    Ptr<NbIotUePhy> GetPhy() const;

    /**
     * \brief Set the UE MAC
     * \param mac the MAC layer
     */
    void SetMac(Ptr<NbIotUeMac> mac);

    /**
     * \brief Get the UE MAC
     * \return the MAC layer
     */
    Ptr<NbIotUeMac> GetMac() const;

    /**
     * \brief Set the RLC layer
     * \param rlc the RLC layer
     */
    void SetRlc(Ptr<NbIotRlc> rlc);

    /**
     * \brief Get the RLC layer
     * \return the RLC layer
     */
    Ptr<NbIotRlc> GetRlc() const;

    /**
     * \brief Set the PDCP layer
     * \param pdcp the PDCP layer
     */
    void SetPdcp(Ptr<NbIotPdcp> pdcp);

    /**
     * \brief Get the PDCP layer
     * \return the PDCP layer
     */
    Ptr<NbIotPdcp> GetPdcp() const;

    /**
     * \brief Set the UE RRC
     * \param rrc the RRC layer
     */
    void SetRrc(Ptr<NbIotUeRrc> rrc);

    /**
     * \brief Get the UE RRC
     * \return the RRC layer
     */
    Ptr<NbIotUeRrc> GetRrc() const;

    /**
     * \brief Set the IMSI
     * \param imsi the International Mobile Subscriber Identity
     */
    void SetImsi(uint64_t imsi);

    /**
     * \brief Get the IMSI
     * \return the IMSI
     */
    uint64_t GetImsi() const;

    /**
     * \brief Set power saving controller
     * \param controller the power saving controller
     */
    void SetPowerSavingController(Ptr<NbIotPowerSavingController> controller);

    /**
     * \brief Get power saving controller
     * \return the power saving controller
     */
    Ptr<NbIotPowerSavingController> GetPowerSavingController() const;

    /**
     * \brief Enable/disable power saving mode
     * \param enablePsm enable PSM
     * \param enableEdrx enable eDRX
     */
    void EnablePowerSaving(bool enablePsm, bool enableEdrx);

    /**
     * \brief Get the coverage class
     * \return the current coverage class
     */
    NbIotCoverageClass GetCoverageClass() const;

    // Inherited from NbIotNetDevice
    void Receive(Ptr<Packet> packet) override;

protected:
    void DoDispose() override;
    void DoInitialize() override;

private:
    Ptr<NbIotUePhy> m_phy;                      //!< PHY layer
    Ptr<NbIotUeMac> m_mac;                      //!< MAC layer
    Ptr<NbIotRlc> m_rlc;                        //!< RLC layer
    Ptr<NbIotPdcp> m_pdcp;                      //!< PDCP layer
    Ptr<NbIotUeRrc> m_rrc;                      //!< RRC layer
    Ptr<NbIotPowerSavingController> m_powerSaving; //!< Power saving controller

    uint64_t m_imsi;                            //!< IMSI
    NbIotCoverageClass m_coverageClass;         //!< Coverage class
};

/**
 * \ingroup nbiot
 * \brief NB-IoT eNB network device
 *
 * Implements eNB-side network device with protocol stack:
 * PHY -> MAC (with scheduler) -> RLC -> PDCP -> RRC
 */
class NbIotEnbNetDevice : public NbIotNetDevice
{
public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    NbIotEnbNetDevice();
    virtual ~NbIotEnbNetDevice();

    /**
     * \brief Set the eNB PHY
     * \param phy the PHY layer
     */
    void SetPhy(Ptr<NbIotEnbPhy> phy);

    /**
     * \brief Get the eNB PHY
     * \return the PHY layer
     */
    Ptr<NbIotEnbPhy> GetPhy() const;

    /**
     * \brief Set the eNB MAC
     * \param mac the MAC layer
     */
    void SetMac(Ptr<NbIotEnbMac> mac);

    /**
     * \brief Get the eNB MAC
     * \return the MAC layer
     */
    Ptr<NbIotEnbMac> GetMac() const;

    /**
     * \brief Set the RLC layer
     * \param rlc the RLC layer
     */
    void SetRlc(Ptr<NbIotRlc> rlc);

    /**
     * \brief Get the RLC layer
     * \return the RLC layer
     */
    Ptr<NbIotRlc> GetRlc() const;

    /**
     * \brief Set the PDCP layer
     * \param pdcp the PDCP layer
     */
    void SetPdcp(Ptr<NbIotPdcp> pdcp);

    /**
     * \brief Get the PDCP layer
     * \return the PDCP layer
     */
    Ptr<NbIotPdcp> GetPdcp() const;

    /**
     * \brief Set the eNB RRC
     * \param rrc the RRC layer
     */
    void SetRrc(Ptr<NbIotEnbRrc> rrc);

    /**
     * \brief Get the eNB RRC
     * \return the RRC layer
     */
    Ptr<NbIotEnbRrc> GetRrc() const;

    /**
     * \brief Set the cell ID
     * \param cellId the cell ID
     */
    void SetCellId(uint16_t cellId);

    /**
     * \brief Get the cell ID
     * \return the cell ID
     */
    uint16_t GetCellId() const;

    /**
     * \brief Get the number of connected UEs
     * \return number of connected UEs
     */
    uint32_t GetNumConnectedUes() const;

    /**
     * \brief Set antenna configuration
     * \param numPorts number of antenna ports (1 or 2)
     */
    void SetAntennaConfiguration(uint8_t numPorts);

    /**
     * \brief Get antenna configuration
     * \return number of antenna ports
     */
    uint8_t GetAntennaConfiguration() const;

    /**
     * \brief Set transmission power
     * \param power transmission power in dBm
     */
    void SetTxPower(double power);

    /**
     * \brief Get transmission power
     * \return transmission power in dBm
     */
    double GetTxPower() const;

    // Inherited from NbIotNetDevice
    void Receive(Ptr<Packet> packet) override;

protected:
    void DoDispose() override;
    void DoInitialize() override;

private:
    Ptr<NbIotEnbPhy> m_phy;                     //!< PHY layer
    Ptr<NbIotEnbMac> m_mac;                     //!< MAC layer
    Ptr<NbIotRlc> m_rlc;                        //!< RLC layer
    Ptr<NbIotPdcp> m_pdcp;                      //!< PDCP layer
    Ptr<NbIotEnbRrc> m_rrc;                     //!< RRC layer

    uint16_t m_cellId;                          //!< Cell ID
    uint8_t m_antennaConfig;                    //!< Antenna configuration
    double m_txPower;                           //!< TX power in dBm
};

} // namespace ns3

#endif /* NBIOT_NET_DEVICE_H */
