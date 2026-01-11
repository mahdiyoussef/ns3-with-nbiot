/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT UE Physical Layer implementation as per 3GPP TS 36.211
 */

#ifndef NBIOT_UE_PHY_H
#define NBIOT_UE_PHY_H

#include "nbiot-phy.h"
#include "nbiot-common.h"

#include <ns3/callback.h>
#include <ns3/event-id.h>

namespace ns3 {

class NbIotUeMac;
class NbIotUeNetDevice;

/**
 * \ingroup nbiot
 * \brief NB-IoT UE PHY layer
 *
 * This class implements the UE-side PHY layer for NB-IoT, including:
 * - Initial cell search (NPSS/NSSS detection)
 * - NPBCH decoding for MIB-NB
 * - NPDCCH monitoring and DCI decoding
 * - NPDSCH reception
 * - NPUSCH transmission (Format 1 and Format 2)
 * - NPRACH transmission
 * - Channel quality measurements
 *
 * \see 3GPP TS 36.211, TS 36.213
 */
class NbIotUePhy : public NbIotPhy
{
public:
    /**
     * \brief Get the TypeId
     * \return TypeId
     */
    static TypeId GetTypeId();

    /**
     * \brief Constructor
     */
    NbIotUePhy();

    /**
     * \brief Destructor
     */
    ~NbIotUePhy() override;

    // Inherited from Object
    void DoDispose() override;

    /**
     * \brief Set the MAC layer
     * \param mac The UE MAC layer
     */
    void SetMac(Ptr<NbIotUeMac> mac);

    /**
     * \brief Get the MAC layer
     * \return The UE MAC layer
     */
    Ptr<NbIotUeMac> GetMac() const;

    /**
     * \brief Set the UE RNTI
     * \param rnti Radio Network Temporary Identifier
     */
    void SetRnti(uint16_t rnti);

    /**
     * \brief Get the UE RNTI
     * \return RNTI
     */
    uint16_t GetRnti() const;

    /**
     * \brief Set the current coverage class
     * \param ceLevel Coverage enhancement level
     */
    void SetCoverageClass(NbIotCoverageClass ceLevel);

    /**
     * \brief Get the current coverage class
     * \return Coverage enhancement level
     */
    NbIotCoverageClass GetCoverageClass() const;

    /**
     * \brief Start initial cell search procedure
     *
     * Initiates the cell search procedure by looking for NPSS/NSSS.
     */
    void StartCellSearch();

    /**
     * \brief Indicate cell search completion
     * \param cellId Detected cell ID
     * \param success Whether search was successful
     */
    void CellSearchCompleted(uint16_t cellId, bool success);

    /**
     * \brief Start monitoring NPBCH for MIB-NB
     */
    void StartNpbchReception();

    /**
     * \brief Process received NPBCH and extract MIB-NB
     * \param mib Received MIB-NB
     */
    void ReceiveMib(const NbIotMib& mib);

    /**
     * \brief Start monitoring NPDCCH for DCI
     * \param searchSpace Search space type (1 = UE-specific, 2 = common)
     */
    void StartNpdcchMonitoring(uint8_t searchSpace);

    /**
     * \brief Stop monitoring NPDCCH
     */
    void StopNpdcchMonitoring();

    /**
     * \brief Process received DCI message
     * \param dci The DCI message
     */
    void ReceiveDci(const NbIotDci& dci);

    /**
     * \brief Start NPDSCH reception
     * \param tbs Transport block size (bits)
     * \param repetitions Number of repetitions
     * \param startSubframe Starting subframe
     */
    void StartNpdschReception(uint16_t tbs, uint8_t repetitions, uint16_t startSubframe);

    /**
     * \brief Process received NPDSCH transport block
     * \param packet Received packet
     * \param sinr SINR of reception
     */
    void ReceiveNpdsch(Ptr<Packet> packet, double sinr);

    /**
     * \brief Transmit data on NPUSCH Format 1
     * \param packet Packet to transmit
     * \param tbs Transport block size
     * \param numSubcarriers Number of subcarriers (1, 3, 6, or 12)
     * \param repetitions Number of repetitions
     * \param subcarrierSpacing Subcarrier spacing
     */
    void TransmitNpuschFormat1(Ptr<Packet> packet, uint16_t tbs, uint8_t numSubcarriers,
                                uint8_t repetitions, NbIotSubcarrierSpacing subcarrierSpacing);

    /**
     * \brief Transmit HARQ ACK/NACK on NPUSCH Format 2
     * \param ack True for ACK, false for NACK
     * \param repetitions Number of repetitions
     */
    void TransmitNpuschFormat2(bool ack, uint8_t repetitions);

    /**
     * \brief Transmit NPRACH preamble
     * \param preambleFormat Preamble format (0 or 1)
     * \param repetitions Number of repetitions
     * \param coverageClass Coverage class for this attempt
     */
    void TransmitNprach(NbIotNprachFormat preambleFormat, uint8_t repetitions,
                        NbIotCoverageClass coverageClass);

    /**
     * \brief Perform RSRP measurement
     * \return Measured RSRP in dBm
     */
    double MeasureRsrp();

    /**
     * \brief Perform RSRQ measurement
     * \return Measured RSRQ in dB
     */
    double MeasureRsrq();

    /**
     * \brief Get the latest channel quality measurements
     * \return Channel quality structure
     */
    NbIotChannelQuality GetChannelQuality() const;

    /**
     * \brief Determine coverage class based on RSRP
     * \param rsrp RSRP measurement in dBm
     * \return Recommended coverage class
     */
    NbIotCoverageClass DetermineCoverageClass(double rsrp) const;

    /**
     * \brief Set power control parameters
     * \param p0 P0 nominal (dBm)
     * \param alpha Path loss compensation factor (0-1)
     */
    void SetPowerControl(double p0, double alpha);

    /**
     * \brief Calculate uplink transmission power
     * \param numSubcarriers Number of subcarriers for transmission
     * \return Transmission power in dBm
     */
    double CalculateUlTxPower(uint8_t numSubcarriers);

    // Callbacks from eNB
    /**
     * \brief Callback type for cell search completion
     */
    typedef Callback<void, uint16_t, bool> CellSearchCallback;

    /**
     * \brief Set cell search completion callback
     * \param callback The callback
     */
    void SetCellSearchCallback(CellSearchCallback callback);

    /**
     * \brief Callback type for MIB-NB reception
     */
    typedef Callback<void, NbIotMib> MibReceivedCallback;

    /**
     * \brief Set MIB received callback
     * \param callback The callback
     */
    void SetMibReceivedCallback(MibReceivedCallback callback);

    /**
     * \brief Callback type for DCI reception
     */
    typedef Callback<void, NbIotDci> DciReceivedCallback;

    /**
     * \brief Set DCI received callback
     * \param callback The callback
     */
    void SetDciReceivedCallback(DciReceivedCallback callback);

    /**
     * \brief Callback type for NPDSCH reception
     */
    typedef Callback<void, Ptr<Packet>> NpdschReceivedCallback;

    /**
     * \brief Set NPDSCH received callback
     * \param callback The callback
     */
    void SetNpdschReceivedCallback(NpdschReceivedCallback callback);

    // Inherited from NbIotPhy
    void SendControlMessage(Ptr<NbIotControlMessage> msg) override;
    void ReceiveControlMessage(Ptr<NbIotControlMessage> msg) override;
    void StartSubframe() override;

protected:
    void DoInitialize() override;

private:
    /**
     * \brief Process NPDCCH candidates in the search space
     */
    void ProcessNpdcchCandidates();

    /**
     * \brief Complete NPUSCH transmission
     */
    void CompleteNpuschTransmission();

    /**
     * \brief Complete NPRACH transmission
     */
    void CompleteNprachTransmission();

    /**
     * \brief Update channel quality measurements
     */
    void UpdateChannelQuality();

    Ptr<NbIotUeMac> m_mac;              ///< MAC layer
    uint16_t m_rnti;                    ///< UE RNTI
    NbIotCoverageClass m_coverageClass; ///< Current coverage class

    // State variables
    bool m_isSearchingCell;             ///< Cell search in progress
    bool m_isMonitoringNpdcch;          ///< NPDCCH monitoring active
    bool m_isReceivingNpdsch;           ///< NPDSCH reception in progress
    bool m_isTransmittingNpusch;        ///< NPUSCH transmission in progress
    bool m_isTransmittingNprach;        ///< NPRACH transmission in progress

    // Reception state
    uint16_t m_npdschTbs;               ///< Expected TBS for NPDSCH
    uint8_t m_npdschRepetitions;        ///< NPDSCH repetitions
    uint8_t m_npdschRepetitionCount;    ///< Current repetition count
    uint16_t m_npdschStartSubframe;     ///< NPDSCH start subframe

    // Transmission state
    Ptr<Packet> m_pendingNpuschPacket;  ///< Packet pending for NPUSCH
    uint8_t m_npuschRepetitions;        ///< NPUSCH repetitions
    uint8_t m_npuschRepetitionCount;    ///< Current NPUSCH repetition
    uint8_t m_npuschNumSubcarriers;     ///< NPUSCH subcarriers
    uint8_t m_nprachRepetitions;        ///< NPRACH repetitions
    uint8_t m_nprachRepetitionCount;    ///< Current NPRACH repetition

    // Power control
    double m_p0Nominal;                 ///< P0 nominal (dBm)
    double m_alpha;                     ///< Path loss compensation factor
    double m_pathLoss;                  ///< Estimated path loss (dB)

    // Channel quality
    NbIotChannelQuality m_channelQuality; ///< Latest measurements

    // RSRP thresholds for coverage class selection (dBm)
    double m_rsrpThresholdCe0;          ///< RSRP threshold for CE Level 0
    double m_rsrpThresholdCe1;          ///< RSRP threshold for CE Level 1

    // Callbacks
    CellSearchCallback m_cellSearchCallback;
    MibReceivedCallback m_mibReceivedCallback;
    DciReceivedCallback m_dciReceivedCallback;
    NpdschReceivedCallback m_npdschReceivedCallback;

    // Events
    EventId m_npuschTxEvent;            ///< NPUSCH transmission event
    EventId m_nprachTxEvent;            ///< NPRACH transmission event
    EventId m_cellSearchEvent;          ///< Cell search event

    // Traced callbacks
    TracedCallback<uint16_t, double, double> m_rsrpRsrqTrace; ///< RSRP/RSRQ measurements
    TracedCallback<uint16_t, NbIotCoverageClass> m_coverageClassTrace; ///< Coverage class changes
};

} // namespace ns3

#endif /* NBIOT_UE_PHY_H */
