/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Physical Layer implementation as per 3GPP TS 36.211
 */

#ifndef NBIOT_PHY_H
#define NBIOT_PHY_H

#include "nbiot-common.h"

#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/packet.h>
#include <ns3/spectrum-channel.h>
#include <ns3/spectrum-phy.h>
#include <ns3/spectrum-value.h>
#include <ns3/mobility-model.h>
#include <ns3/net-device.h>
#include <ns3/traced-callback.h>
#include <ns3/nstime.h>
#include <ns3/event-id.h>

#include <complex>
#include <vector>
#include <map>
#include <functional>

namespace ns3 {

class NbIotNetDevice;
class NbIotSpectrumPhy;
class NbIotControlMessage;

/**
 * \ingroup nbiot
 * \brief NB-IoT resource grid for one subframe
 *
 * Implements the resource element mapping for NB-IoT as per 3GPP TS 36.211.
 * The grid contains 12 subcarriers x 14 OFDM symbols per subframe.
 */
class NbIotResourceGrid : public Object
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
    NbIotResourceGrid();

    /**
     * \brief Destructor
     */
    ~NbIotResourceGrid() override;

    /**
     * \brief Initialize the resource grid for a given cell ID
     * \param cellId Physical cell ID
     */
    void Initialize(uint16_t cellId);

    /**
     * \brief Clear the resource grid
     */
    void Clear();

    /**
     * \brief Check if a resource element is available
     * \param subcarrier Subcarrier index (0-11)
     * \param symbol OFDM symbol index (0-13)
     * \return True if available
     */
    bool IsResourceElementAvailable(uint8_t subcarrier, uint8_t symbol) const;

    /**
     * \brief Mark resource elements as occupied by a channel
     * \param subcarrier Starting subcarrier
     * \param symbol Starting symbol
     * \param numSubcarriers Number of subcarriers
     * \param numSymbols Number of symbols
     * \param channel Channel type
     */
    void AllocateResourceElements(uint8_t subcarrier, uint8_t symbol,
                                   uint8_t numSubcarriers, uint8_t numSymbols,
                                   NbIotPhysicalChannel channel);

    /**
     * \brief Set a modulated symbol at a resource element
     * \param subcarrier Subcarrier index
     * \param symbol OFDM symbol index
     * \param value Complex modulated value
     */
    void SetResourceElement(uint8_t subcarrier, uint8_t symbol, std::complex<double> value);

    /**
     * \brief Get a modulated symbol from a resource element
     * \param subcarrier Subcarrier index
     * \param symbol OFDM symbol index
     * \return Complex modulated value
     */
    std::complex<double> GetResourceElement(uint8_t subcarrier, uint8_t symbol) const;

    /**
     * \brief Get NRS positions for this cell
     * \return Vector of (subcarrier, symbol) pairs
     */
    std::vector<std::pair<uint8_t, uint8_t>> GetNrsPositions() const;

    /**
     * \brief Generate NRS sequence for channel estimation
     * \param slotNumber Slot number within the frame
     */
    void GenerateNrs(uint16_t slotNumber);

private:
    static constexpr uint8_t NUM_SUBCARRIERS = 12;
    static constexpr uint8_t NUM_SYMBOLS = 14; // Per subframe (2 slots)

    uint16_t m_cellId;
    std::array<std::array<std::complex<double>, NUM_SYMBOLS>, NUM_SUBCARRIERS> m_grid;
    std::array<std::array<bool, NUM_SYMBOLS>, NUM_SUBCARRIERS> m_occupied;
    std::array<std::array<NbIotPhysicalChannel, NUM_SYMBOLS>, NUM_SUBCARRIERS> m_channelType;
};

/**
 * \ingroup nbiot
 * \brief Abstract base class for NB-IoT PHY layer
 *
 * This class provides the common functionality for both UE and eNB PHY layers.
 * It implements resource grid management, channel coding/decoding, and
 * modulation/demodulation as per 3GPP TS 36.211, TS 36.212.
 *
 * \see 3GPP TS 36.211 V13.2.0 Section 10
 */
class NbIotPhy : public Object
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
    NbIotPhy();

    /**
     * \brief Destructor
     */
    ~NbIotPhy() override;

    // Inherited from Object
    void DoDispose() override;

    /**
     * \brief Set the deployment mode
     * \param mode Deployment mode (STANDALONE, GUARDBAND, INBAND)
     *
     * \see 3GPP TS 36.211 Section 10.1.3.5
     */
    void SetDeploymentMode(NbIotDeploymentMode mode);

    /**
     * \brief Get the deployment mode
     * \return Current deployment mode
     */
    NbIotDeploymentMode GetDeploymentMode() const;

    /**
     * \brief Set the deployment mode as uint8_t (for attribute system)
     * \param mode Deployment mode value
     */
    void SetDeploymentModeValue(uint8_t mode);

    /**
     * \brief Get the deployment mode as uint8_t (for attribute system)
     * \return Deployment mode value
     */
    uint8_t GetDeploymentModeValue() const;

    /**
     * \brief Set the physical cell ID
     * \param cellId Cell ID (0-503)
     */
    void SetCellId(uint16_t cellId);

    /**
     * \brief Get the physical cell ID
     * \return Cell ID
     */
    uint16_t GetCellId() const;

    /**
     * \brief Set the carrier frequency
     * \param frequencyHz Center frequency in Hz
     */
    void SetCarrierFrequency(double frequencyHz);

    /**
     * \brief Get the carrier frequency
     * \return Center frequency in Hz
     */
    double GetCarrierFrequency() const;

    /**
     * \brief Set the transmission power
     * \param powerDbm Transmission power in dBm
     */
    void SetTxPower(double powerDbm);

    /**
     * \brief Get the transmission power
     * \return Transmission power in dBm
     */
    double GetTxPower() const;

    /**
     * \brief Set the noise figure
     * \param noiseFigureDb Noise figure in dB
     */
    void SetNoiseFigure(double noiseFigureDb);

    /**
     * \brief Get the noise figure
     * \return Noise figure in dB
     */
    double GetNoiseFigure() const;

    /**
     * \brief Set the spectrum channel
     * \param channel The spectrum channel
     */
    void SetChannel(Ptr<SpectrumChannel> channel);

    /**
     * \brief Get the spectrum channel
     * \return The spectrum channel
     */
    Ptr<SpectrumChannel> GetChannel() const;

    /**
     * \brief Set the net device
     * \param device The net device
     */
    void SetDevice(Ptr<NbIotNetDevice> device);

    /**
     * \brief Get the net device
     * \return The net device
     */
    Ptr<NbIotNetDevice> GetDevice() const;

    /**
     * \brief Set the mobility model
     * \param mobility The mobility model
     */
    void SetMobility(Ptr<MobilityModel> mobility);

    /**
     * \brief Get the mobility model
     * \return The mobility model
     */
    Ptr<MobilityModel> GetMobility() const;

    /**
     * \brief Get the current subframe number
     * \return Subframe number (0-9)
     */
    uint8_t GetSubframeNumber() const;

    /**
     * \brief Get the current frame number
     * \return Frame number (0-1023)
     */
    uint16_t GetFrameNumber() const;

    /**
     * \brief Calculate SINR for received signal
     * \param signal Signal power (linear)
     * \param interference Interference power (linear)
     * \return SINR in dB
     */
    double CalculateSinr(double signal, double interference) const;

    /**
     * \brief Map bits to QPSK symbols
     * \param bits Input bit sequence
     * \return Complex symbols
     *
     * \see 3GPP TS 36.211 Section 10.2.6.2
     */
    std::vector<std::complex<double>> QpskModulate(const std::vector<uint8_t>& bits);

    /**
     * \brief Demap QPSK symbols to bits
     * \param symbols Complex symbols
     * \return Bit sequence
     */
    std::vector<uint8_t> QpskDemodulate(const std::vector<std::complex<double>>& symbols);

    /**
     * \brief Map bits to π/4-QPSK symbols (for single-tone uplink)
     * \param bits Input bit sequence
     * \return Complex symbols
     *
     * \see 3GPP TS 36.211 Section 10.1.3.2.1
     */
    std::vector<std::complex<double>> Pi4QpskModulate(const std::vector<uint8_t>& bits);

    /**
     * \brief Apply turbo coding
     * \param data Input data bits
     * \return Coded bits
     *
     * \see 3GPP TS 36.212 Section 5.1.3.2
     */
    std::vector<uint8_t> TurboEncode(const std::vector<uint8_t>& data);

    /**
     * \brief Apply turbo decoding
     * \param codedData Coded bits with soft values
     * \param iterations Number of decoder iterations
     * \return Decoded bits
     */
    std::vector<uint8_t> TurboDecode(const std::vector<double>& codedData, uint8_t iterations = 6);

    /**
     * \brief Apply rate matching
     * \param codedBits Input coded bits
     * \param outputLength Desired output length
     * \return Rate-matched bits
     */
    std::vector<uint8_t> RateMatch(const std::vector<uint8_t>& codedBits, size_t outputLength);

    /**
     * \brief Apply scrambling sequence
     * \param bits Input bits
     * \param rnti RNTI for UE-specific scrambling
     * \param subframe Subframe number
     * \return Scrambled bits
     *
     * \see 3GPP TS 36.211 Section 10.2.2.1
     */
    std::vector<uint8_t> Scramble(const std::vector<uint8_t>& bits, uint16_t rnti, uint8_t subframe);

    /**
     * \brief Apply descrambling
     * \param bits Scrambled bits
     * \param rnti RNTI
     * \param subframe Subframe number
     * \return Descrambled bits
     */
    std::vector<uint8_t> Descramble(const std::vector<uint8_t>& bits, uint16_t rnti, uint8_t subframe);

    /**
     * \brief Generate Gold sequence for scrambling
     * \param length Sequence length
     * \param cinit Initialization value
     * \return Pseudo-random sequence
     */
    std::vector<uint8_t> GenerateGoldSequence(size_t length, uint32_t cinit);

    /**
     * \brief Called at the start of each subframe
     */
    virtual void StartSubframe();

    /**
     * \brief Called at the end of each subframe
     */
    virtual void EndSubframe();

    /**
     * \brief Send a control message
     * \param msg The control message
     */
    virtual void SendControlMessage(Ptr<NbIotControlMessage> msg) = 0;

    /**
     * \brief Receive a control message
     * \param msg The control message
     */
    virtual void ReceiveControlMessage(Ptr<NbIotControlMessage> msg) = 0;

    // Tracing
    /**
     * \brief TracedCallback signature for PHY transmission events
     * \param cellId Cell ID
     * \param rnti UE RNTI
     * \param channel Physical channel
     * \param txPower Transmission power (dBm)
     */
    typedef void (*PhyTxCallback)(uint16_t cellId, uint16_t rnti, 
                                   NbIotPhysicalChannel channel, double txPower);

    /**
     * \brief TracedCallback signature for PHY reception events
     * \param cellId Cell ID
     * \param rnti UE RNTI
     * \param channel Physical channel
     * \param sinr SINR (dB)
     * \param success Whether decoding was successful
     */
    typedef void (*PhyRxCallback)(uint16_t cellId, uint16_t rnti,
                                   NbIotPhysicalChannel channel, double sinr, bool success);

protected:
    /**
     * \brief Initialize the PHY layer
     */
    void DoInitialize() override;

    /**
     * \brief Schedule the next subframe processing
     */
    void ScheduleNextSubframe();

    /**
     * \brief Generate the Narrowband Reference Signal (NRS)
     * \param subframe Subframe number
     * \return NRS symbols for this subframe
     *
     * \see 3GPP TS 36.211 Section 10.2.6
     */
    std::vector<std::complex<double>> GenerateNrs(uint8_t subframe);

    /**
     * \brief Perform channel estimation using NRS
     * \param receivedNrs Received NRS symbols
     * \param expectedNrs Expected NRS symbols
     * \return Channel estimates
     */
    std::vector<std::complex<double>> EstimateChannel(
        const std::vector<std::complex<double>>& receivedNrs,
        const std::vector<std::complex<double>>& expectedNrs);

    NbIotDeploymentMode m_deploymentMode;  ///< Current deployment mode
    uint16_t m_cellId;                      ///< Physical cell ID (0-503)
    double m_carrierFrequency;              ///< Carrier frequency (Hz)
    double m_txPowerDbm;                    ///< Transmission power (dBm)
    double m_noiseFigureDb;                 ///< Noise figure (dB)

    Ptr<SpectrumChannel> m_channel;         ///< Spectrum channel
    Ptr<NbIotNetDevice> m_device;           ///< Associated net device
    Ptr<MobilityModel> m_mobility;          ///< Mobility model
    Ptr<NbIotSpectrumPhy> m_spectrumPhy;    ///< Spectrum PHY

    uint16_t m_frameNumber;                 ///< Current frame number
    uint8_t m_subframeNumber;               ///< Current subframe number
    EventId m_subframeEvent;                ///< Subframe processing event

    Ptr<NbIotResourceGrid> m_resourceGrid;  ///< Resource grid

    // Traced callbacks
    TracedCallback<uint16_t, uint16_t, NbIotPhysicalChannel, double> m_phyTxTrace;
    TracedCallback<uint16_t, uint16_t, NbIotPhysicalChannel, double, bool> m_phyRxTrace;
};

/**
 * \ingroup nbiot
 * \brief NB-IoT spectrum PHY for signal transmission/reception
 *
 * This class interfaces with the NS-3 spectrum framework for
 * realistic signal propagation and interference modeling.
 */
class NbIotSpectrumPhy : public SpectrumPhy
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
    NbIotSpectrumPhy();

    /**
     * \brief Destructor
     */
    ~NbIotSpectrumPhy() override;

    // Inherited from SpectrumPhy
    void SetChannel(Ptr<SpectrumChannel> c) override;
    void SetMobility(Ptr<MobilityModel> m) override;
    void SetDevice(Ptr<NetDevice> d) override;
    Ptr<MobilityModel> GetMobility() const override;
    Ptr<NetDevice> GetDevice() const override;
    Ptr<const SpectrumModel> GetRxSpectrumModel() const override;
    Ptr<Object> GetAntenna() const override;
    void StartRx(Ptr<SpectrumSignalParameters> params) override;

    /**
     * \brief Set the antenna model
     * \param antenna The antenna model
     */
    void SetAntenna(Ptr<Object> antenna);

    /**
     * \brief Set the PHY layer
     * \param phy The NB-IoT PHY layer
     */
    void SetNbIotPhy(Ptr<NbIotPhy> phy);

    /**
     * \brief Start transmission of a signal
     * \param params Signal parameters
     */
    void StartTx(Ptr<SpectrumSignalParameters> params);

    /**
     * \brief Set the noise power spectral density
     * \param noisePsd Noise PSD
     */
    void SetNoisePowerSpectralDensity(Ptr<const SpectrumValue> noisePsd);

    /**
     * \brief Create the transmit power spectral density
     * \param powerDbm Transmission power in dBm
     * \return PSD
     */
    Ptr<SpectrumValue> CreateTxPowerSpectralDensity(double powerDbm);

    /**
     * \brief Callback type for reception end
     */
    typedef Callback<void, Ptr<Packet>, double> RxEndCallback;

    /**
     * \brief Set the reception end callback
     * \param callback The callback
     */
    void SetRxEndCallback(RxEndCallback callback);

private:
    Ptr<SpectrumChannel> m_channel;
    Ptr<MobilityModel> m_mobility;
    Ptr<NetDevice> m_device;
    Ptr<Object> m_antenna;
    Ptr<const SpectrumModel> m_rxSpectrumModel;
    Ptr<NbIotPhy> m_nbiotPhy;
    Ptr<const SpectrumValue> m_noisePsd;
    RxEndCallback m_rxEndCallback;
};

} // namespace ns3

#endif /* NBIOT_PHY_H */
