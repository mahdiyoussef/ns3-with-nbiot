/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Physical Layer implementation
 */

#include "nbiot-phy.h"
#include "nbiot-net-device.h"
#include "nbiot-control-messages.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/double.h>
#include <ns3/uinteger.h>
#include <ns3/pointer.h>
#include <ns3/spectrum-value.h>
#include <ns3/antenna-model.h>

#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotPhy");

// ====================== NbIotResourceGrid ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotResourceGrid);

TypeId
NbIotResourceGrid::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotResourceGrid")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotResourceGrid>();
    return tid;
}

NbIotResourceGrid::NbIotResourceGrid()
    : m_cellId(0)
{
    NS_LOG_FUNCTION(this);
    Clear();
}

NbIotResourceGrid::~NbIotResourceGrid()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotResourceGrid::Initialize(uint16_t cellId)
{
    NS_LOG_FUNCTION(this << cellId);
    m_cellId = cellId;
    Clear();
}

void
NbIotResourceGrid::Clear()
{
    NS_LOG_FUNCTION(this);
    for (auto& subcarrier : m_grid)
    {
        subcarrier.fill(std::complex<double>(0.0, 0.0));
    }
    for (auto& subcarrier : m_occupied)
    {
        subcarrier.fill(false);
    }
}

bool
NbIotResourceGrid::IsResourceElementAvailable(uint8_t subcarrier, uint8_t symbol) const
{
    if (subcarrier >= NUM_SUBCARRIERS || symbol >= NUM_SYMBOLS)
    {
        return false;
    }
    return !m_occupied[subcarrier][symbol];
}

void
NbIotResourceGrid::AllocateResourceElements(uint8_t subcarrier, uint8_t symbol,
                                             uint8_t numSubcarriers, uint8_t numSymbols,
                                             NbIotPhysicalChannel channel)
{
    NS_LOG_FUNCTION(this << static_cast<int>(subcarrier) << static_cast<int>(symbol)
                         << static_cast<int>(numSubcarriers) << static_cast<int>(numSymbols));

    for (uint8_t sc = subcarrier; sc < subcarrier + numSubcarriers && sc < NUM_SUBCARRIERS; ++sc)
    {
        for (uint8_t sym = symbol; sym < symbol + numSymbols && sym < NUM_SYMBOLS; ++sym)
        {
            m_occupied[sc][sym] = true;
            m_channelType[sc][sym] = channel;
        }
    }
}

void
NbIotResourceGrid::SetResourceElement(uint8_t subcarrier, uint8_t symbol, std::complex<double> value)
{
    if (subcarrier < NUM_SUBCARRIERS && symbol < NUM_SYMBOLS)
    {
        m_grid[subcarrier][symbol] = value;
    }
}

std::complex<double>
NbIotResourceGrid::GetResourceElement(uint8_t subcarrier, uint8_t symbol) const
{
    if (subcarrier < NUM_SUBCARRIERS && symbol < NUM_SYMBOLS)
    {
        return m_grid[subcarrier][symbol];
    }
    return std::complex<double>(0.0, 0.0);
}

std::vector<std::pair<uint8_t, uint8_t>>
NbIotResourceGrid::GetNrsPositions() const
{
    // NRS positions per 3GPP TS 36.211 Section 10.2.6
    // Positions depend on cell ID
    std::vector<std::pair<uint8_t, uint8_t>> positions;
    
    // Slot 0: symbols 5, 6 (within subframe symbols 5, 6)
    // Slot 1: symbols 5, 6 (within subframe symbols 12, 13)
    uint8_t vShift = m_cellId % 6;
    
    // Subcarriers: 0, 3 (or offset by vShift)
    for (uint8_t symbol : {5, 6, 12, 13})
    {
        for (uint8_t k = 0; k < 2; ++k)
        {
            uint8_t subcarrier = (3 * k + vShift) % 12;
            positions.push_back({subcarrier, symbol});
        }
    }
    
    return positions;
}

void
NbIotResourceGrid::GenerateNrs(uint16_t slotNumber)
{
    NS_LOG_FUNCTION(this << slotNumber);
    
    // Generate NRS sequence based on cell ID and slot number
    // Per 3GPP TS 36.211 Section 10.2.6
    
    auto positions = GetNrsPositions();
    uint32_t cinit = (7 * (slotNumber + 1) + 1) * (2 * m_cellId + 1) * 512 + m_cellId;
    
    // Simplified NRS generation (actual implementation uses Gold sequences)
    for (size_t i = 0; i < positions.size(); ++i)
    {
        double phase = 2.0 * M_PI * ((cinit + i) % 1024) / 1024.0;
        std::complex<double> nrsSymbol(std::cos(phase), std::sin(phase));
        SetResourceElement(positions[i].first, positions[i].second, nrsSymbol);
        m_occupied[positions[i].first][positions[i].second] = true;
    }
}

// ====================== NbIotPhy ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotPhy);

TypeId
NbIotPhy::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotPhy")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddAttribute("DeploymentMode",
                      "NB-IoT deployment mode (standalone, guard-band, in-band)",
                      UintegerValue(static_cast<uint8_t>(NbIotDeploymentMode::STANDALONE)),
                      MakeUintegerAccessor(&NbIotPhy::SetDeploymentModeValue,
                                           &NbIotPhy::GetDeploymentModeValue),
                      MakeUintegerChecker<uint8_t>(0, 2))
        .AddAttribute("CellId",
                      "Physical cell ID",
                      UintegerValue(0),
                      MakeUintegerAccessor(&NbIotPhy::m_cellId),
                      MakeUintegerChecker<uint16_t>(0, NbIotConstants::MAX_CELL_ID))
        .AddAttribute("CarrierFrequency",
                      "Carrier frequency in Hz",
                      DoubleValue(900e6),
                      MakeDoubleAccessor(&NbIotPhy::m_carrierFrequency),
                      MakeDoubleChecker<double>())
        .AddAttribute("TxPower",
                      "Transmission power in dBm",
                      DoubleValue(23.0),
                      MakeDoubleAccessor(&NbIotPhy::m_txPowerDbm),
                      MakeDoubleChecker<double>())
        .AddAttribute("NoiseFigure",
                      "Noise figure in dB",
                      DoubleValue(5.0),
                      MakeDoubleAccessor(&NbIotPhy::m_noiseFigureDb),
                      MakeDoubleChecker<double>())
        .AddTraceSource("PhyTx",
                        "Trace fired upon PHY transmission",
                        MakeTraceSourceAccessor(&NbIotPhy::m_phyTxTrace),
                        "ns3::NbIotPhy::PhyTxCallback")
        .AddTraceSource("PhyRx",
                        "Trace fired upon PHY reception",
                        MakeTraceSourceAccessor(&NbIotPhy::m_phyRxTrace),
                        "ns3::NbIotPhy::PhyRxCallback");
    return tid;
}

NbIotPhy::NbIotPhy()
    : m_deploymentMode(NbIotDeploymentMode::STANDALONE)
    , m_cellId(0)
    , m_carrierFrequency(900e6)
    , m_txPowerDbm(23.0)
    , m_noiseFigureDb(5.0)
    , m_frameNumber(0)
    , m_subframeNumber(0)
{
    NS_LOG_FUNCTION(this);
    m_resourceGrid = CreateObject<NbIotResourceGrid>();
}

NbIotPhy::~NbIotPhy()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotPhy::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_channel = nullptr;
    m_device = nullptr;
    m_mobility = nullptr;
    m_spectrumPhy = nullptr;
    m_resourceGrid = nullptr;
    if (m_subframeEvent.IsPending())
    {
        Simulator::Cancel(m_subframeEvent);
    }
    Object::DoDispose();
}

void
NbIotPhy::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    m_resourceGrid->Initialize(m_cellId);
    ScheduleNextSubframe();
    Object::DoInitialize();
}

void
NbIotPhy::SetDeploymentMode(NbIotDeploymentMode mode)
{
    NS_LOG_FUNCTION(this << static_cast<int>(mode));
    m_deploymentMode = mode;
}

NbIotDeploymentMode
NbIotPhy::GetDeploymentMode() const
{
    return m_deploymentMode;
}

void
NbIotPhy::SetCellId(uint16_t cellId)
{
    NS_LOG_FUNCTION(this << cellId);
    NS_ASSERT_MSG(cellId <= NbIotConstants::MAX_CELL_ID, "Cell ID out of range");
    m_cellId = cellId;
    m_resourceGrid->Initialize(cellId);
}

uint16_t
NbIotPhy::GetCellId() const
{
    return m_cellId;
}

void
NbIotPhy::SetCarrierFrequency(double frequencyHz)
{
    NS_LOG_FUNCTION(this << frequencyHz);
    m_carrierFrequency = frequencyHz;
}

double
NbIotPhy::GetCarrierFrequency() const
{
    return m_carrierFrequency;
}

void
NbIotPhy::SetTxPower(double powerDbm)
{
    NS_LOG_FUNCTION(this << powerDbm);
    m_txPowerDbm = powerDbm;
}

double
NbIotPhy::GetTxPower() const
{
    return m_txPowerDbm;
}

void
NbIotPhy::SetNoiseFigure(double noiseFigureDb)
{
    NS_LOG_FUNCTION(this << noiseFigureDb);
    m_noiseFigureDb = noiseFigureDb;
}

double
NbIotPhy::GetNoiseFigure() const
{
    return m_noiseFigureDb;
}

void
NbIotPhy::SetChannel(Ptr<SpectrumChannel> channel)
{
    NS_LOG_FUNCTION(this << channel);
    m_channel = channel;
}

Ptr<SpectrumChannel>
NbIotPhy::GetChannel() const
{
    return m_channel;
}

void
NbIotPhy::SetDevice(Ptr<NbIotNetDevice> device)
{
    NS_LOG_FUNCTION(this << device);
    m_device = device;
}

Ptr<NbIotNetDevice>
NbIotPhy::GetDevice() const
{
    return m_device;
}

void
NbIotPhy::SetMobility(Ptr<MobilityModel> mobility)
{
    NS_LOG_FUNCTION(this << mobility);
    m_mobility = mobility;
}

Ptr<MobilityModel>
NbIotPhy::GetMobility() const
{
    return m_mobility;
}

uint8_t
NbIotPhy::GetSubframeNumber() const
{
    return m_subframeNumber;
}

uint16_t
NbIotPhy::GetFrameNumber() const
{
    return m_frameNumber;
}

double
NbIotPhy::CalculateSinr(double signal, double interference) const
{
    // Calculate thermal noise
    double kT = 1.38e-23 * 290.0; // Boltzmann constant * temperature (K)
    double noisePowerW = kT * NbIotConstants::SYSTEM_BANDWIDTH_HZ;
    double noiseFigureLinear = std::pow(10.0, m_noiseFigureDb / 10.0);
    double totalNoiseW = noisePowerW * noiseFigureLinear;
    
    double sinrLinear = signal / (interference + totalNoiseW);
    return 10.0 * std::log10(sinrLinear);
}

std::vector<std::complex<double>>
NbIotPhy::QpskModulate(const std::vector<uint8_t>& bits)
{
    NS_LOG_FUNCTION(this << bits.size());
    
    std::vector<std::complex<double>> symbols;
    symbols.reserve(bits.size() / 2);
    
    // QPSK constellation: (1+j)/sqrt(2), (1-j)/sqrt(2), (-1+j)/sqrt(2), (-1-j)/sqrt(2)
    const double scale = 1.0 / std::sqrt(2.0);
    
    for (size_t i = 0; i + 1 < bits.size(); i += 2)
    {
        double real = bits[i] ? -scale : scale;
        double imag = bits[i + 1] ? -scale : scale;
        symbols.emplace_back(real, imag);
    }
    
    return symbols;
}

std::vector<uint8_t>
NbIotPhy::QpskDemodulate(const std::vector<std::complex<double>>& symbols)
{
    NS_LOG_FUNCTION(this << symbols.size());
    
    std::vector<uint8_t> bits;
    bits.reserve(symbols.size() * 2);
    
    for (const auto& sym : symbols)
    {
        bits.push_back(sym.real() < 0 ? 1 : 0);
        bits.push_back(sym.imag() < 0 ? 1 : 0);
    }
    
    return bits;
}

std::vector<std::complex<double>>
NbIotPhy::Pi4QpskModulate(const std::vector<uint8_t>& bits)
{
    NS_LOG_FUNCTION(this << bits.size());
    
    std::vector<std::complex<double>> symbols;
    symbols.reserve(bits.size() / 2);
    
    // π/4-QPSK with phase continuity
    double currentPhase = 0.0;
    const double phaseIncrement[4] = {M_PI / 4, 3 * M_PI / 4, -M_PI / 4, -3 * M_PI / 4};
    
    for (size_t i = 0; i + 1 < bits.size(); i += 2)
    {
        uint8_t index = (bits[i] << 1) | bits[i + 1];
        currentPhase += phaseIncrement[index];
        symbols.emplace_back(std::cos(currentPhase), std::sin(currentPhase));
    }
    
    return symbols;
}

std::vector<uint8_t>
NbIotPhy::TurboEncode(const std::vector<uint8_t>& data)
{
    NS_LOG_FUNCTION(this << data.size());
    
    // Simplified turbo encoding (rate 1/3)
    // Actual implementation would use the LTE turbo coder
    std::vector<uint8_t> coded;
    coded.reserve(data.size() * 3);
    
    for (uint8_t bit : data)
    {
        coded.push_back(bit);           // Systematic
        coded.push_back(bit);           // Parity 1 (simplified)
        coded.push_back(bit ^ 1);       // Parity 2 (simplified)
    }
    
    return coded;
}

std::vector<uint8_t>
NbIotPhy::TurboDecode(const std::vector<double>& codedData, uint8_t iterations)
{
    NS_LOG_FUNCTION(this << codedData.size() << static_cast<int>(iterations));
    
    // Simplified turbo decoding (hard decision)
    std::vector<uint8_t> decoded;
    decoded.reserve(codedData.size() / 3);
    
    for (size_t i = 0; i + 2 < codedData.size(); i += 3)
    {
        // Simple majority voting
        double sum = codedData[i] + codedData[i + 1] - codedData[i + 2];
        decoded.push_back(sum > 0 ? 1 : 0);
    }
    
    return decoded;
}

std::vector<uint8_t>
NbIotPhy::RateMatch(const std::vector<uint8_t>& codedBits, size_t outputLength)
{
    NS_LOG_FUNCTION(this << codedBits.size() << outputLength);
    
    std::vector<uint8_t> matched;
    matched.reserve(outputLength);
    
    if (outputLength <= codedBits.size())
    {
        // Puncturing
        size_t step = codedBits.size() / outputLength;
        for (size_t i = 0; i < outputLength && i * step < codedBits.size(); ++i)
        {
            matched.push_back(codedBits[i * step]);
        }
    }
    else
    {
        // Repetition
        for (size_t i = 0; i < outputLength; ++i)
        {
            matched.push_back(codedBits[i % codedBits.size()]);
        }
    }
    
    return matched;
}

std::vector<uint8_t>
NbIotPhy::GenerateGoldSequence(size_t length, uint32_t cinit)
{
    NS_LOG_FUNCTION(this << length << cinit);
    
    std::vector<uint8_t> sequence(length);
    
    // Initialize shift registers
    uint32_t x1 = 1; // x1(0) = 1, others = 0
    uint32_t x2 = cinit;
    
    // Advance x1 and x2 by 1600 positions (Nc = 1600)
    for (int n = 0; n < 1600; ++n)
    {
        x1 = ((x1 >> 3) ^ x1) & 1;
        x1 = (x1 << 30) | (x1 >> 1);
        x2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
        x2 = (x2 << 30) | (x2 >> 1);
    }
    
    // Generate sequence
    for (size_t n = 0; n < length; ++n)
    {
        uint8_t bit1 = ((x1 >> 3) ^ x1) & 1;
        uint8_t bit2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
        sequence[n] = bit1 ^ bit2;
        
        x1 = (x1 << 1) | bit1;
        x2 = (x2 << 1) | bit2;
    }
    
    return sequence;
}

std::vector<uint8_t>
NbIotPhy::Scramble(const std::vector<uint8_t>& bits, uint16_t rnti, uint8_t subframe)
{
    NS_LOG_FUNCTION(this << bits.size() << rnti << static_cast<int>(subframe));
    
    uint32_t cinit = rnti * 16384 + subframe * 512 + m_cellId;
    auto goldSeq = GenerateGoldSequence(bits.size(), cinit);
    
    std::vector<uint8_t> scrambled(bits.size());
    for (size_t i = 0; i < bits.size(); ++i)
    {
        scrambled[i] = bits[i] ^ goldSeq[i];
    }
    
    return scrambled;
}

std::vector<uint8_t>
NbIotPhy::Descramble(const std::vector<uint8_t>& bits, uint16_t rnti, uint8_t subframe)
{
    // Descrambling is the same as scrambling (XOR is its own inverse)
    return Scramble(bits, rnti, subframe);
}

std::vector<std::complex<double>>
NbIotPhy::GenerateNrs(uint8_t subframe)
{
    NS_LOG_FUNCTION(this << static_cast<int>(subframe));
    
    std::vector<std::complex<double>> nrs;
    
    // Generate NRS for both slots in the subframe
    for (uint8_t slot = 0; slot < 2; ++slot)
    {
        uint16_t slotNumber = m_frameNumber * 20 + subframe * 2 + slot;
        uint32_t cinit = (7 * (slotNumber + 1) + 1) * (2 * m_cellId + 1) * 512 + m_cellId;
        
        auto goldSeq = GenerateGoldSequence(4, cinit);
        
        for (size_t i = 0; i < goldSeq.size(); i += 2)
        {
            double real = (1 - 2 * goldSeq[i]) / std::sqrt(2.0);
            double imag = (1 - 2 * goldSeq[i + 1]) / std::sqrt(2.0);
            nrs.emplace_back(real, imag);
        }
    }
    
    return nrs;
}

std::vector<std::complex<double>>
NbIotPhy::EstimateChannel(const std::vector<std::complex<double>>& receivedNrs,
                           const std::vector<std::complex<double>>& expectedNrs)
{
    NS_LOG_FUNCTION(this);
    
    std::vector<std::complex<double>> estimates;
    estimates.reserve(receivedNrs.size());
    
    for (size_t i = 0; i < receivedNrs.size() && i < expectedNrs.size(); ++i)
    {
        if (std::abs(expectedNrs[i]) > 0.0)
        {
            estimates.push_back(receivedNrs[i] / expectedNrs[i]);
        }
        else
        {
            estimates.emplace_back(1.0, 0.0);
        }
    }
    
    return estimates;
}

void
NbIotPhy::StartSubframe()
{
    NS_LOG_FUNCTION(this << m_frameNumber << static_cast<int>(m_subframeNumber));
    
    // Clear resource grid for new subframe
    m_resourceGrid->Clear();
    
    // Generate NRS for this subframe
    m_resourceGrid->GenerateNrs(m_frameNumber * 20 + m_subframeNumber * 2);
}

void
NbIotPhy::EndSubframe()
{
    NS_LOG_FUNCTION(this);
    
    // Advance to next subframe
    m_subframeNumber++;
    if (m_subframeNumber >= NbIotConstants::SUBFRAMES_PER_FRAME)
    {
        m_subframeNumber = 0;
        m_frameNumber = (m_frameNumber + 1) % 1024;
    }
    
    ScheduleNextSubframe();
}

void
NbIotPhy::ScheduleNextSubframe()
{
    NS_LOG_FUNCTION(this);
    
    m_subframeEvent = Simulator::Schedule(
        MilliSeconds(NbIotConstants::SUBFRAME_DURATION_MS),
        &NbIotPhy::StartSubframe,
        this);
    
    Simulator::Schedule(
        MilliSeconds(NbIotConstants::SUBFRAME_DURATION_MS - 0.001),
        &NbIotPhy::EndSubframe,
        this);
}

// Helper functions for attribute access
void
NbIotPhy::SetDeploymentModeValue(uint8_t mode)
{
    m_deploymentMode = static_cast<NbIotDeploymentMode>(mode);
}

uint8_t
NbIotPhy::GetDeploymentModeValue() const
{
    return static_cast<uint8_t>(m_deploymentMode);
}

// ====================== NbIotSpectrumPhy ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotSpectrumPhy);

TypeId
NbIotSpectrumPhy::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotSpectrumPhy")
        .SetParent<SpectrumPhy>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotSpectrumPhy>();
    return tid;
}

NbIotSpectrumPhy::NbIotSpectrumPhy()
{
    NS_LOG_FUNCTION(this);
}

NbIotSpectrumPhy::~NbIotSpectrumPhy()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotSpectrumPhy::SetChannel(Ptr<SpectrumChannel> c)
{
    NS_LOG_FUNCTION(this << c);
    m_channel = c;
}

void
NbIotSpectrumPhy::SetMobility(Ptr<MobilityModel> m)
{
    NS_LOG_FUNCTION(this << m);
    m_mobility = m;
}

void
NbIotSpectrumPhy::SetDevice(Ptr<NetDevice> d)
{
    NS_LOG_FUNCTION(this << d);
    m_device = d;
}

Ptr<MobilityModel>
NbIotSpectrumPhy::GetMobility() const
{
    return m_mobility;
}

Ptr<NetDevice>
NbIotSpectrumPhy::GetDevice() const
{
    return m_device;
}

Ptr<const SpectrumModel>
NbIotSpectrumPhy::GetRxSpectrumModel() const
{
    return m_rxSpectrumModel;
}

Ptr<Object>
NbIotSpectrumPhy::GetAntenna() const
{
    return m_antenna;
}

void
NbIotSpectrumPhy::SetAntenna(Ptr<Object> antenna)
{
    NS_LOG_FUNCTION(this << antenna);
    m_antenna = antenna;
}

void
NbIotSpectrumPhy::SetNbIotPhy(Ptr<NbIotPhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    m_nbiotPhy = phy;
}

void
NbIotSpectrumPhy::StartRx(Ptr<SpectrumSignalParameters> params)
{
    NS_LOG_FUNCTION(this << params);
    
    // Calculate received power
    double rxPower = 0.0;
    if (params->psd)
    {
        for (auto it = params->psd->ConstValuesBegin(); it != params->psd->ConstValuesEnd(); ++it)
        {
            rxPower += (*it) * (NbIotConstants::SYSTEM_BANDWIDTH_HZ / 12.0); // Per subcarrier
        }
    }
    
    double rxPowerDbm = 10.0 * std::log10(rxPower * 1000.0);
    NS_LOG_DEBUG("Received signal with power " << rxPowerDbm << " dBm");
    
    // TODO: Process received signal and call m_rxEndCallback
}

void
NbIotSpectrumPhy::StartTx(Ptr<SpectrumSignalParameters> params)
{
    NS_LOG_FUNCTION(this << params);
    
    if (m_channel)
    {
        m_channel->StartTx(params);
    }
}

void
NbIotSpectrumPhy::SetNoisePowerSpectralDensity(Ptr<const SpectrumValue> noisePsd)
{
    NS_LOG_FUNCTION(this << noisePsd);
    m_noisePsd = noisePsd;
}

void
NbIotSpectrumPhy::SetRxEndCallback(RxEndCallback callback)
{
    m_rxEndCallback = callback;
}

Ptr<SpectrumValue>
NbIotSpectrumPhy::CreateTxPowerSpectralDensity(double powerDbm)
{
    NS_LOG_FUNCTION(this << powerDbm);
    
    // Convert dBm to Watts
    double powerW = std::pow(10.0, (powerDbm - 30.0) / 10.0);
    
    // Create PSD (power spread over 180 kHz bandwidth)
    double psdW = powerW / NbIotConstants::SYSTEM_BANDWIDTH_HZ;
    
    // Create and populate SpectrumValue (simplified)
    Ptr<SpectrumValue> psd;
    if (m_rxSpectrumModel)
    {
        psd = Create<SpectrumValue>(m_rxSpectrumModel);
        auto it = psd->ValuesBegin();
        while (it != psd->ValuesEnd())
        {
            *it = psdW;
            ++it;
        }
    }
    
    return psd;
}

} // namespace ns3
