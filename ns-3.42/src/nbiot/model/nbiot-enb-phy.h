/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT eNB Physical Layer implementation as per 3GPP TS 36.211
 */

#ifndef NBIOT_ENB_PHY_H
#define NBIOT_ENB_PHY_H

#include "nbiot-phy.h"
#include "nbiot-common.h"

#include <ns3/callback.h>
#include <ns3/event-id.h>

#include <map>
#include <set>

namespace ns3 {

class NbIotEnbMac;
class NbIotEnbNetDevice;

/**
 * \ingroup nbiot
 * \brief NB-IoT eNB PHY layer
 *
 * This class implements the eNB-side PHY layer for NB-IoT, including:
 * - NPSS/NSSS transmission for synchronization
 * - NPBCH transmission (MIB-NB)
 * - NPDCCH transmission (DCI)
 * - NPDSCH transmission (downlink data and SIBs)
 * - NPUSCH reception (uplink data and HARQ feedback)
 * - NPRACH reception (random access)
 *
 * \see 3GPP TS 36.211, TS 36.213
 */
class NbIotEnbPhy : public NbIotPhy
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
    NbIotEnbPhy();

    /**
     * \brief Destructor
     */
    ~NbIotEnbPhy() override;

    // Inherited from Object
    void DoDispose() override;

    /**
     * \brief Set the MAC layer
     * \param mac The eNB MAC layer
     */
    void SetMac(Ptr<NbIotEnbMac> mac);

    /**
     * \brief Get the MAC layer
     * \return The eNB MAC layer
     */
    Ptr<NbIotEnbMac> GetMac() const;

    /**
     * \brief Set the MIB-NB for broadcast
     * \param mib The MIB-NB content
     */
    void SetMib(const NbIotMib& mib);

    /**
     * \brief Get the current MIB-NB
     * \return MIB-NB content
     */
    NbIotMib GetMib() const;

    /**
     * \brief Transmit NPSS (Narrowband Primary Synchronization Signal)
     *
     * NPSS is transmitted in subframe 5 of every frame.
     * \see 3GPP TS 36.211 Section 10.2.7.1
     */
    void TransmitNpss();

    /**
     * \brief Transmit NSSS (Narrowband Secondary Synchronization Signal)
     *
     * NSSS is transmitted in subframe 9 of even frames.
     * \see 3GPP TS 36.211 Section 10.2.7.2
     */
    void TransmitNsss();

    /**
     * \brief Transmit NPBCH (Narrowband Physical Broadcast Channel)
     *
     * NPBCH carries MIB-NB with 640 ms periodicity and 8 repetitions.
     * \see 3GPP TS 36.211 Section 10.2.4
     */
    void TransmitNpbch();

    /**
     * \brief Transmit NRS (Narrowband Reference Signal)
     *
     * NRS is transmitted for channel estimation and measurements.
     * \see 3GPP TS 36.211 Section 10.2.6
     */
    void TransmitNrs();

    /**
     * \brief Transmit DCI on NPDCCH
     * \param dci The DCI message to transmit
     * \param repetitions Number of repetitions
     * \param aggregationLevel Aggregation level (1 or 2)
     *
     * \see 3GPP TS 36.211 Section 10.2.5
     */
    void TransmitNpdcch(const NbIotDci& dci, uint16_t repetitions, uint8_t aggregationLevel);

    /**
     * \brief Transmit data on NPDSCH
     * \param packet Packet to transmit
     * \param rnti Destination UE RNTI
     * \param tbs Transport block size
     * \param repetitions Number of repetitions
     *
     * \see 3GPP TS 36.211 Section 10.2.3
     */
    void TransmitNpdsch(Ptr<Packet> packet, uint16_t rnti, uint16_t tbs, uint8_t repetitions);

    /**
     * \brief Start NPRACH reception window
     * \param coverageClass Coverage class for this NPRACH resource
     *
     * \see 3GPP TS 36.211 Section 10.1.6
     */
    void StartNprachReception(NbIotCoverageClass coverageClass);

    /**
     * \brief Stop NPRACH reception window
     */
    void StopNprachReception();

    /**
     * \brief Process received NPRACH preamble
     * \param preambleIndex Detected preamble index
     * \param timingAdvance Estimated timing advance
     * \param coverageClass Coverage class of the NPRACH resource
     */
    void ReceiveNprach(uint8_t preambleIndex, uint16_t timingAdvance,
                       NbIotCoverageClass coverageClass);

    /**
     * \brief Start NPUSCH reception
     * \param rnti Expected UE RNTI
     * \param tbs Expected transport block size
     * \param numSubcarriers Number of subcarriers
     * \param repetitions Number of repetitions
     * \param startSubframe Starting subframe
     */
    void StartNpuschReception(uint16_t rnti, uint16_t tbs, uint8_t numSubcarriers,
                               uint8_t repetitions, uint16_t startSubframe);

    /**
     * \brief Process received NPUSCH
     * \param rnti UE RNTI
     * \param packet Received packet (nullptr if decoding failed)
     * \param sinr SINR of reception
     */
    void ReceiveNpusch(uint16_t rnti, Ptr<Packet> packet, double sinr);

    /**
     * \brief Process received NPUSCH Format 2 (HARQ feedback)
     * \param rnti UE RNTI
     * \param ack True for ACK, false for NACK
     */
    void ReceiveNpuschFormat2(uint16_t rnti, bool ack);

    /**
     * \brief Add a UE to the list of attached UEs
     * \param rnti UE RNTI
     * \param coverageClass Initial coverage class
     */
    void AddUe(uint16_t rnti, NbIotCoverageClass coverageClass);

    /**
     * \brief Remove a UE from the list of attached UEs
     * \param rnti UE RNTI
     */
    void RemoveUe(uint16_t rnti);

    /**
     * \brief Update UE coverage class
     * \param rnti UE RNTI
     * \param coverageClass New coverage class
     */
    void UpdateUeCoverageClass(uint16_t rnti, NbIotCoverageClass coverageClass);

    /**
     * \brief Get the number of attached UEs
     * \return Number of UEs
     */
    uint32_t GetNumAttachedUes() const;

    // Callbacks
    /**
     * \brief Callback type for NPRACH reception
     */
    typedef Callback<void, uint8_t, uint16_t, NbIotCoverageClass> NprachReceivedCallback;

    /**
     * \brief Set NPRACH received callback
     * \param callback The callback
     */
    void SetNprachReceivedCallback(NprachReceivedCallback callback);

    /**
     * \brief Callback type for NPUSCH reception
     */
    typedef Callback<void, uint16_t, Ptr<Packet>, bool> NpuschReceivedCallback;

    /**
     * \brief Set NPUSCH received callback
     * \param callback The callback
     */
    void SetNpuschReceivedCallback(NpuschReceivedCallback callback);

    /**
     * \brief Callback type for HARQ feedback
     */
    typedef Callback<void, uint16_t, bool> HarqFeedbackCallback;

    /**
     * \brief Set HARQ feedback callback
     * \param callback The callback
     */
    void SetHarqFeedbackCallback(HarqFeedbackCallback callback);

    // Inherited from NbIotPhy
    void SendControlMessage(Ptr<NbIotControlMessage> msg) override;
    void ReceiveControlMessage(Ptr<NbIotControlMessage> msg) override;
    void StartSubframe() override;

protected:
    void DoInitialize() override;

private:
    /**
     * \brief Generate NPSS symbols
     * \return Complex NPSS symbols
     */
    std::vector<std::complex<double>> GenerateNpss();

    /**
     * \brief Generate NSSS symbols
     * \param nf Frame number for NSSS sequence
     * \return Complex NSSS symbols
     */
    std::vector<std::complex<double>> GenerateNsss(uint16_t nf);

    /**
     * \brief Encode MIB-NB for NPBCH
     * \return Encoded bits
     */
    std::vector<uint8_t> EncodeMibNb();

    /**
     * \brief Encode DCI for NPDCCH
     * \param dci DCI message
     * \return Encoded bits
     */
    std::vector<uint8_t> EncodeDci(const NbIotDci& dci);

    /**
     * \brief Check if this subframe is an NPSS subframe
     * \return True if NPSS should be transmitted
     */
    bool IsNpssSubframe() const;

    /**
     * \brief Check if this subframe is an NSSS subframe
     * \return True if NSSS should be transmitted
     */
    bool IsNsssSubframe() const;

    /**
     * \brief Check if this subframe is an NPBCH subframe
     * \return True if NPBCH should be transmitted
     */
    bool IsNpbchSubframe() const;

    /**
     * \brief Complete NPDCCH transmission
     * \param rnti Target UE RNTI
     */
    void CompleteNpdcchTransmission(uint16_t rnti);

    /**
     * \brief Complete NPDSCH transmission
     * \param rnti Target UE RNTI
     */
    void CompleteNpdschTransmission(uint16_t rnti);

    Ptr<NbIotEnbMac> m_mac;             ///< MAC layer

    // MIB-NB content
    NbIotMib m_mib;                     ///< Current MIB-NB

    // UE tracking
    struct UeInfo
    {
        uint16_t rnti;
        NbIotCoverageClass coverageClass;
        bool isReceivingNpusch;
        uint16_t npuschTbs;
        uint8_t npuschRepetitions;
    };
    std::map<uint16_t, UeInfo> m_attachedUes; ///< Attached UEs

    // Transmission state
    std::map<uint16_t, EventId> m_npdcchTxEvents;  ///< Pending NPDCCH transmissions
    std::map<uint16_t, EventId> m_npdschTxEvents;  ///< Pending NPDSCH transmissions

    // Reception state
    bool m_isReceivingNprach;           ///< NPRACH reception window active
    NbIotCoverageClass m_nprachCoverageClass; ///< Coverage class for current NPRACH

    std::set<uint16_t> m_activeNpuschReceptions; ///< UEs with active NPUSCH reception

    // Callbacks
    NprachReceivedCallback m_nprachReceivedCallback;
    NpuschReceivedCallback m_npuschReceivedCallback;
    HarqFeedbackCallback m_harqFeedbackCallback;

    // NPBCH state
    uint8_t m_npbchRepetitionIndex;     ///< Current NPBCH repetition (0-7)

    // Traced callbacks
    TracedCallback<uint16_t, uint16_t, uint16_t> m_npdschTxTrace; ///< NPDSCH Tx: cellId, rnti, tbs
    TracedCallback<uint8_t, uint16_t, NbIotCoverageClass> m_nprachRxTrace; ///< NPRACH Rx
    TracedCallback<uint16_t, uint16_t, bool> m_npuschRxTrace; ///< NPUSCH Rx: rnti, tbs, success
};

} // namespace ns3

#endif /* NBIOT_ENB_PHY_H */
