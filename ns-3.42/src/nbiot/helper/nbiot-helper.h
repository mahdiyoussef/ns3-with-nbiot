/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Helper classes for simulation setup
 */

#ifndef NBIOT_HELPER_H
#define NBIOT_HELPER_H

#include "../model/nbiot-common.h"

#include <ns3/node-container.h>
#include <ns3/net-device-container.h>
#include <ns3/object-factory.h>
#include <ns3/spectrum-channel.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/spectrum-propagation-loss-model.h>
#include <ns3/antenna-model.h>
#include <ns3/mobility-model.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/ptr.h>

#include <string>
#include <map>

namespace ns3 {

class NbIotUeNetDevice;
class NbIotEnbNetDevice;
class NbIotScheduler;
class NbIotSpectrumChannel;
class NbIotUePhy;
class NbIotEnbPhy;

/**
 * \ingroup nbiot
 * \brief Helper class for NB-IoT network configuration
 *
 * This helper simplifies the creation and configuration of
 * NB-IoT networks, including eNBs, UEs, and protocol stack setup.
 */
class NbIotHelper : public Object
{
public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    NbIotHelper();
    virtual ~NbIotHelper();

    /**
     * \brief Set the deployment mode for NB-IoT
     * \param mode deployment mode (standalone, guard-band, in-band)
     */
    void SetDeploymentMode(NbIotDeploymentMode mode);

    /**
     * \brief Get the deployment mode
     * \return deployment mode
     */
    NbIotDeploymentMode GetDeploymentMode() const;

    /**
     * \brief Set the EARFCN for DL and UL
     * \param dlEarfcn DL EARFCN
     * \param ulEarfcn UL EARFCN
     */
    void SetEarfcn(uint32_t dlEarfcn, uint32_t ulEarfcn);

    /**
     * \brief Set spectrum channel for downlink
     * \param channel the spectrum channel
     */
    void SetDlChannel(Ptr<SpectrumChannel> channel);

    /**
     * \brief Set spectrum channel for uplink
     * \param channel the spectrum channel
     */
    void SetUlChannel(Ptr<SpectrumChannel> channel);

    /**
     * \brief Get DL spectrum channel
     * \return the DL channel
     */
    Ptr<SpectrumChannel> GetDlChannel() const;

    /**
     * \brief Get UL spectrum channel
     * \return the UL channel
     */
    Ptr<SpectrumChannel> GetUlChannel() const;

    /**
     * \brief Set the pathloss model for the channel
     * \param type the pathloss model TypeId
     */
    void SetPathlossModelType(TypeId type);

    /**
     * \brief Set pathloss model attribute
     * \param n attribute name
     * \param v attribute value
     */
    void SetPathlossModelAttribute(std::string n, const AttributeValue& v);

    /**
     * \brief Set the fading model type
     * \param type the fading model TypeId
     */
    void SetFadingModelType(TypeId type);

    /**
     * \brief Set fading model attribute
     * \param n attribute name
     * \param v attribute value
     */
    void SetFadingModelAttribute(std::string n, const AttributeValue& v);

    /**
     * \brief Set the scheduler type for eNB MAC
     * \param type scheduler TypeId (NbIotRoundRobinScheduler or NbIotCoverageClassScheduler)
     */
    void SetSchedulerType(TypeId type);

    /**
     * \brief Set the scheduler type for eNB MAC (string version)
     * \param type scheduler TypeId string (e.g., "ns3::NbIotRoundRobinScheduler")
     */
    void SetSchedulerType(const std::string& type);

    /**
     * \brief Set scheduler attribute
     * \param n attribute name
     * \param v attribute value
     */
    void SetSchedulerAttribute(std::string n, const AttributeValue& v);

    /**
     * \brief Set eNB device attribute
     * \param n attribute name
     * \param v attribute value
     */
    void SetEnbDeviceAttribute(std::string n, const AttributeValue& v);

    /**
     * \brief Set UE device attribute
     * \param n attribute name
     * \param v attribute value
     */
    void SetUeDeviceAttribute(std::string n, const AttributeValue& v);

    /**
     * \brief Set eNB antenna model
     * \param type antenna model TypeId
     */
    void SetEnbAntennaModelType(TypeId type);

    /**
     * \brief Set eNB antenna attribute
     * \param n attribute name
     * \param v attribute value
     */
    void SetEnbAntennaModelAttribute(std::string n, const AttributeValue& v);

    /**
     * \brief Set UE antenna model
     * \param type antenna model TypeId
     */
    void SetUeAntennaModelType(TypeId type);

    /**
     * \brief Set UE antenna attribute
     * \param n attribute name
     * \param v attribute value
     */
    void SetUeAntennaModelAttribute(std::string n, const AttributeValue& v);

    /**
     * \brief Install NB-IoT eNB devices on nodes
     * \param c the node container
     * \return container of installed net devices
     */
    NetDeviceContainer InstallEnbDevice(NodeContainer c);

    /**
     * \brief Install NB-IoT UE devices on nodes
     * \param c the node container
     * \return container of installed net devices
     */
    NetDeviceContainer InstallUeDevice(NodeContainer c);

    /**
     * \brief Attach UEs to a specific eNB
     * \param ueDevices container of UE devices
     * \param enbDevice the eNB device
     */
    void Attach(NetDeviceContainer ueDevices, Ptr<NetDevice> enbDevice);

    /**
     * \brief Attach UE to the eNB with best RSRP
     * \param ueDevices container of UE devices
     * \param enbDevices container of eNB devices
     */
    void AttachToClosestEnb(NetDeviceContainer ueDevices, NetDeviceContainer enbDevices);

    /**
     * \brief Enable power saving mode for UEs
     * \param ueDevices container of UE devices
     * \param enablePsm enable PSM
     * \param enableEdrx enable eDRX
     */
    void EnablePowerSaving(NetDeviceContainer ueDevices, bool enablePsm, bool enableEdrx);

    /**
     * \brief Set PSM timers for UEs
     * \param ueDevices container of UE devices
     * \param t3324 active timer
     * \param t3412 TAU timer
     */
    void SetPsmTimers(NetDeviceContainer ueDevices, Time t3324, Time t3412);

    /**
     * \brief Set eDRX parameters for UEs
     * \param ueDevices container of UE devices
     * \param edrxCycle eDRX cycle duration
     * \param ptw paging time window
     */
    void SetEdrxParameters(NetDeviceContainer ueDevices, Time edrxCycle, Time ptw);

    /**
     * \brief Activate a data radio bearer for UEs
     * \param ueDevices container of UE devices
     * \param bearer the EPS bearer to activate
     */
    void ActivateDataRadioBearer(NetDeviceContainer ueDevices);

    /**
     * \brief Enable traces for PHY layer
     */
    void EnablePhyTraces();

    /**
     * \brief Enable traces for MAC layer
     */
    void EnableMacTraces();

    /**
     * \brief Enable traces for RLC layer
     */
    void EnableRlcTraces();

    /**
     * \brief Enable traces for PDCP layer
     */
    void EnablePdcpTraces();

    /**
     * \brief Enable all traces
     */
    void EnableTraces();

    /**
     * \brief Calculate distance between two nodes
     * \param a first node
     * \param b second node
     * \return distance in meters
     */
    static double GetDistanceBetweenNodes(Ptr<Node> a, Ptr<Node> b);

protected:
    void DoDispose() override;

private:
    /**
     * \brief Create and configure eNB net device
     * \param node the node
     * \return configured eNB device
     */
    Ptr<NbIotEnbNetDevice> CreateEnbDevice(Ptr<Node> node);

    /**
     * \brief Create and configure UE net device
     * \param node the node
     * \return configured UE device
     */
    Ptr<NbIotUeNetDevice> CreateUeDevice(Ptr<Node> node);

    /**
     * \brief Create and configure spectrum channel
     */
    void InitializeChannels();

    NbIotDeploymentMode m_deploymentMode;       //!< Deployment mode
    uint32_t m_dlEarfcn;                         //!< DL EARFCN
    uint32_t m_ulEarfcn;                         //!< UL EARFCN

    Ptr<SpectrumChannel> m_dlChannel;           //!< DL spectrum channel
    Ptr<SpectrumChannel> m_ulChannel;           //!< UL spectrum channel

    ObjectFactory m_pathlossModelFactory;       //!< Pathloss model factory
    ObjectFactory m_fadingModelFactory;         //!< Fading model factory
    ObjectFactory m_schedulerFactory;           //!< Scheduler factory
    ObjectFactory m_enbDeviceFactory;           //!< eNB device factory
    ObjectFactory m_ueDeviceFactory;            //!< UE device factory
    ObjectFactory m_enbAntennaFactory;          //!< eNB antenna factory
    ObjectFactory m_ueAntennaFactory;           //!< UE antenna factory

    uint16_t m_cellIdCounter;                   //!< Cell ID counter
    uint64_t m_imsiCounter;                     //!< IMSI counter

    bool m_phyTracesEnabled;                    //!< PHY traces enabled
    bool m_macTracesEnabled;                    //!< MAC traces enabled
    bool m_rlcTracesEnabled;                    //!< RLC traces enabled
    bool m_pdcpTracesEnabled;                   //!< PDCP traces enabled
};

/**
 * \ingroup nbiot
 * \brief Helper for NB-IoT PHY statistics output
 */
class NbIotPhyStatsHelper
{
public:
    NbIotPhyStatsHelper();
    ~NbIotPhyStatsHelper();

    /**
     * \brief Set output filename prefix
     * \param prefix filename prefix
     */
    void SetOutputPrefix(std::string prefix);

    /**
     * \brief Connect to PHY traces
     * \param dev the NB-IoT device
     */
    void ConnectToDevice(Ptr<NetDevice> dev);

    /**
     * \brief Write output files
     */
    void WriteOutput();

private:
    std::string m_outputPrefix;                 //!< Output filename prefix
};

/**
 * \ingroup nbiot
 * \brief Helper for NB-IoT energy consumption analysis
 */
class NbIotEnergyHelper
{
public:
    NbIotEnergyHelper();
    ~NbIotEnergyHelper();

    /**
     * \brief Install energy model on UE devices
     * \param ueDevices container of UE devices
     * \param initialEnergy initial battery energy in Joules
     */
    void InstallEnergyModel(NetDeviceContainer ueDevices, double initialEnergy);

    /**
     * \brief Get remaining energy for a device
     * \param dev the UE device
     * \return remaining energy in Joules
     */
    double GetRemainingEnergy(Ptr<NetDevice> dev) const;

    /**
     * \brief Get total energy consumed by a device
     * \param dev the UE device
     * \return energy consumed in Joules
     */
    double GetConsumedEnergy(Ptr<NetDevice> dev) const;

    /**
     * \brief Get estimated battery life for a device
     * \param dev the UE device
     * \return estimated lifetime in hours
     */
    double GetEstimatedBatteryLife(Ptr<NetDevice> dev) const;

private:
    std::map<Ptr<NetDevice>, double> m_initialEnergy; //!< Initial energy per device
};

/**
 * \ingroup nbiot
 * \brief Helper for NB-IoT coverage analysis
 */
class NbIotCoverageHelper
{
public:
    NbIotCoverageHelper();
    ~NbIotCoverageHelper();

    /**
     * \brief Calculate MCL for given distance and pathloss model
     * \param enbDev eNB device
     * \param ueDev UE device
     * \return MCL in dB
     */
    double CalculateMcl(Ptr<NbIotEnbNetDevice> enbDev,
                        Ptr<NbIotUeNetDevice> ueDev) const;

    /**
     * \brief Determine coverage class based on MCL
     * \param mcl maximum coupling loss in dB
     * \return coverage class
     */
    static NbIotCoverageClass DetermineCoverageClass(double mcl);

    /**
     * \brief Get required repetitions for coverage class
     * \param coverageClass the coverage class
     * \param channel the physical channel type
     * \return number of repetitions
     */
    static uint16_t GetRequiredRepetitions(NbIotCoverageClass coverageClass,
                                           NbIotPhysicalChannel channel);
};

} // namespace ns3

#endif /* NBIOT_HELPER_H */
