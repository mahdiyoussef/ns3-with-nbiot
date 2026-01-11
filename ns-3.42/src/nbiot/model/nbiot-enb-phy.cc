/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT eNB Physical Layer implementation
 */

#include "nbiot-enb-phy.h"
#include "nbiot-enb-mac.h"
#include "nbiot-control-messages.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/double.h>
#include <ns3/uinteger.h>
#include <ns3/pointer.h>

#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotEnbPhy");

NS_OBJECT_ENSURE_REGISTERED(NbIotEnbPhy);

TypeId
NbIotEnbPhy::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotEnbPhy")
        .SetParent<NbIotPhy>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotEnbPhy>()
        // Note: TxPower attribute is inherited from NbIotPhy parent class
        .AddTraceSource("NpdschTx",
                        "NPDSCH transmission trace",
                        MakeTraceSourceAccessor(&NbIotEnbPhy::m_npdschTxTrace),
                        "ns3::NbIotEnbPhy::NpdschTxTracedCallback")
        .AddTraceSource("NprachRx",
                        "NPRACH reception trace",
                        MakeTraceSourceAccessor(&NbIotEnbPhy::m_nprachRxTrace),
                        "ns3::NbIotEnbPhy::NprachRxTracedCallback")
        .AddTraceSource("NpuschRx",
                        "NPUSCH reception trace",
                        MakeTraceSourceAccessor(&NbIotEnbPhy::m_npuschRxTrace),
                        "ns3::NbIotEnbPhy::NpuschRxTracedCallback");
    return tid;
}

NbIotEnbPhy::NbIotEnbPhy()
    : m_isReceivingNprach(false)
    , m_nprachCoverageClass(NbIotCoverageClass::CE_LEVEL_0)
    , m_npbchRepetitionIndex(0)
{
    NS_LOG_FUNCTION(this);
    
    // Initialize default MIB
    m_mib.systemFrameNumber = 0;
    m_mib.hyperFrameNumber = 0;
    m_mib.schedulingInfoSib1 = 0;
    m_mib.operationModeInfo = false;
    m_mib.deploymentMode = NbIotDeploymentMode::STANDALONE;
}

NbIotEnbPhy::~NbIotEnbPhy()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotEnbPhy::DoDispose()
{
    NS_LOG_FUNCTION(this);
    
    m_mac = nullptr;
    m_attachedUes.clear();
    
    for (auto& event : m_npdcchTxEvents)
    {
        if (event.second.IsPending())
        {
            Simulator::Cancel(event.second);
        }
    }
    m_npdcchTxEvents.clear();
    
    for (auto& event : m_npdschTxEvents)
    {
        if (event.second.IsPending())
        {
            Simulator::Cancel(event.second);
        }
    }
    m_npdschTxEvents.clear();
    
    NbIotPhy::DoDispose();
}

void
NbIotEnbPhy::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    NbIotPhy::DoInitialize();
}

void
NbIotEnbPhy::SetMac(Ptr<NbIotEnbMac> mac)
{
    NS_LOG_FUNCTION(this << mac);
    m_mac = mac;
}

Ptr<NbIotEnbMac>
NbIotEnbPhy::GetMac() const
{
    return m_mac;
}

void
NbIotEnbPhy::SetMib(const NbIotMib& mib)
{
    NS_LOG_FUNCTION(this);
    m_mib = mib;
}

NbIotMib
NbIotEnbPhy::GetMib() const
{
    return m_mib;
}

bool
NbIotEnbPhy::IsNpssSubframe() const
{
    // NPSS is transmitted in subframe 5 of every frame
    return m_subframeNumber == 5;
}

bool
NbIotEnbPhy::IsNsssSubframe() const
{
    // NSSS is transmitted in subframe 9 of even frames
    return (m_subframeNumber == 9) && ((m_frameNumber % 2) == 0);
}

bool
NbIotEnbPhy::IsNpbchSubframe() const
{
    // NPBCH is transmitted in subframe 0 of radio frames 0-7 within each 640 ms period
    if (m_subframeNumber != 0)
    {
        return false;
    }
    
    uint16_t frameWithinPeriod = m_frameNumber % 64; // 640 ms = 64 frames
    return frameWithinPeriod < 8;
}

void
NbIotEnbPhy::TransmitNpss()
{
    NS_LOG_FUNCTION(this);
    
    auto npssSymbols = GenerateNpss();
    
    // Map NPSS to resource grid (subcarriers 0-10 in symbols 3-13)
    // NPSS occupies 11 subcarriers
    m_resourceGrid->AllocateResourceElements(0, 3, 11, 11, NbIotPhysicalChannel::NPBCH);
    
    for (size_t i = 0; i < npssSymbols.size() && i < 11 * 11; ++i)
    {
        uint8_t subcarrier = i % 11;
        uint8_t symbol = 3 + (i / 11);
        m_resourceGrid->SetResourceElement(subcarrier, symbol, npssSymbols[i]);
    }
    
    m_phyTxTrace(m_cellId, 0, NbIotPhysicalChannel::NPBCH, m_txPowerDbm);
    
    NS_LOG_INFO("eNB " << m_cellId << " transmitted NPSS in frame " << m_frameNumber
                << " subframe " << static_cast<int>(m_subframeNumber));
}

void
NbIotEnbPhy::TransmitNsss()
{
    NS_LOG_FUNCTION(this);
    
    auto nsssSymbols = GenerateNsss(m_frameNumber);
    
    // Map NSSS to resource grid (similar to NPSS but in subframe 9)
    m_resourceGrid->AllocateResourceElements(0, 3, 12, 11, NbIotPhysicalChannel::NPBCH);
    
    for (size_t i = 0; i < nsssSymbols.size() && i < 12 * 11; ++i)
    {
        uint8_t subcarrier = i % 12;
        uint8_t symbol = 3 + (i / 12);
        m_resourceGrid->SetResourceElement(subcarrier, symbol, nsssSymbols[i]);
    }
    
    m_phyTxTrace(m_cellId, 0, NbIotPhysicalChannel::NPBCH, m_txPowerDbm);
    
    NS_LOG_INFO("eNB " << m_cellId << " transmitted NSSS in frame " << m_frameNumber
                << " subframe " << static_cast<int>(m_subframeNumber));
}

void
NbIotEnbPhy::TransmitNpbch()
{
    NS_LOG_FUNCTION(this);
    
    // Update MIB with current SFN
    m_mib.systemFrameNumber = (m_frameNumber >> 6) & 0x0F; // 4 MSBs
    
    auto mibBits = EncodeMibNb();
    
    // Apply channel coding and rate matching
    auto codedBits = TurboEncode(mibBits);
    auto rateMatchedBits = RateMatch(codedBits, 1600); // NPBCH has ~1600 coded bits per TTI
    
    // Scramble
    auto scrambledBits = Scramble(rateMatchedBits, 0xFFFF, m_subframeNumber);
    
    // Modulate
    auto symbols = QpskModulate(scrambledBits);
    
    // Map to resource grid
    // NPBCH uses specific RE positions avoiding NRS
    size_t symIdx = 0;
    for (uint8_t sym = 0; sym < 14 && symIdx < symbols.size(); ++sym)
    {
        for (uint8_t sc = 0; sc < 12 && symIdx < symbols.size(); ++sc)
        {
            if (m_resourceGrid->IsResourceElementAvailable(sc, sym))
            {
                m_resourceGrid->SetResourceElement(sc, sym, symbols[symIdx++]);
            }
        }
    }
    
    m_phyTxTrace(m_cellId, 0, NbIotPhysicalChannel::NPBCH, m_txPowerDbm);
    
    m_npbchRepetitionIndex = (m_npbchRepetitionIndex + 1) % 8;
    
    NS_LOG_INFO("eNB " << m_cellId << " transmitted NPBCH repetition "
                << static_cast<int>(m_npbchRepetitionIndex) << " in frame " << m_frameNumber);
}

void
NbIotEnbPhy::TransmitNrs()
{
    NS_LOG_FUNCTION(this);
    
    // NRS is generated within the resource grid
    m_resourceGrid->GenerateNrs(m_frameNumber * 20 + m_subframeNumber * 2);
}

void
NbIotEnbPhy::TransmitNpdcch(const NbIotDci& dci, uint16_t repetitions, uint8_t aggregationLevel)
{
    NS_LOG_FUNCTION(this << dci.rnti << repetitions << static_cast<int>(aggregationLevel));
    
    auto dciBits = EncodeDci(dci);
    
    // Apply channel coding (convolutional coding for DCI)
    // Rate 1/3 tail-biting convolutional code
    auto codedBits = TurboEncode(dciBits); // Simplified - should use convolutional coding
    
    // Rate matching based on aggregation level
    size_t outputSize = (aggregationLevel == 2) ? 288 : 144; // Approximate
    auto rateMatchedBits = RateMatch(codedBits, outputSize);
    
    // Scramble
    auto scrambledBits = Scramble(rateMatchedBits, dci.rnti, m_subframeNumber);
    
    // Modulate
    auto symbols = QpskModulate(scrambledBits);
    
    // Map to NPDCCH resource elements
    // NPDCCH uses specific subframes based on configuration
    m_resourceGrid->AllocateResourceElements(0, 0, 12, 14, NbIotPhysicalChannel::NPDCCH);
    
    size_t symIdx = 0;
    for (uint8_t sym = 0; sym < 14 && symIdx < symbols.size(); ++sym)
    {
        for (uint8_t sc = 0; sc < 12 && symIdx < symbols.size(); ++sc)
        {
            if (m_resourceGrid->IsResourceElementAvailable(sc, sym))
            {
                m_resourceGrid->SetResourceElement(sc, sym, symbols[symIdx++]);
            }
        }
    }
    
    m_phyTxTrace(m_cellId, dci.rnti, NbIotPhysicalChannel::NPDCCH, m_txPowerDbm);
    
    // Schedule transmission completion
    Time txDuration = MilliSeconds(NbIotConstants::SUBFRAME_DURATION_MS * repetitions);
    m_npdcchTxEvents[dci.rnti] = Simulator::Schedule(txDuration,
                                                      &NbIotEnbPhy::CompleteNpdcchTransmission,
                                                      this, dci.rnti);
    
    NS_LOG_INFO("eNB " << m_cellId << " transmitting NPDCCH for RNTI " << dci.rnti
                << " with " << repetitions << " repetitions, AL=" << static_cast<int>(aggregationLevel));
}

void
NbIotEnbPhy::TransmitNpdsch(Ptr<Packet> packet, uint16_t rnti, uint16_t tbs, uint8_t repetitions)
{
    NS_LOG_FUNCTION(this << packet << rnti << tbs << static_cast<int>(repetitions));
    
    // Get packet data
    std::vector<uint8_t> dataBits(packet->GetSize() * 8);
    // Simplified - actual implementation would serialize packet
    
    // Apply turbo coding
    auto codedBits = TurboEncode(dataBits);
    
    // Rate matching for NPDSCH
    // Calculate number of REs available (depends on subframe configuration)
    size_t numRes = 12 * 14 - 8; // Approximate, excluding NRS
    size_t outputSize = numRes * 2; // QPSK = 2 bits per RE
    auto rateMatchedBits = RateMatch(codedBits, outputSize);
    
    // Scramble
    auto scrambledBits = Scramble(rateMatchedBits, rnti, m_subframeNumber);
    
    // Modulate
    auto symbols = QpskModulate(scrambledBits);
    
    // Map to NPDSCH resource elements
    m_resourceGrid->AllocateResourceElements(0, 0, 12, 14, NbIotPhysicalChannel::NPDSCH);
    
    size_t symIdx = 0;
    for (uint8_t sym = 0; sym < 14 && symIdx < symbols.size(); ++sym)
    {
        for (uint8_t sc = 0; sc < 12 && symIdx < symbols.size(); ++sc)
        {
            if (m_resourceGrid->IsResourceElementAvailable(sc, sym))
            {
                m_resourceGrid->SetResourceElement(sc, sym, symbols[symIdx++]);
            }
        }
    }
    
    m_phyTxTrace(m_cellId, rnti, NbIotPhysicalChannel::NPDSCH, m_txPowerDbm);
    m_npdschTxTrace(m_cellId, rnti, tbs);
    
    // Schedule transmission completion
    Time txDuration = MilliSeconds(NbIotConstants::SUBFRAME_DURATION_MS * repetitions);
    m_npdschTxEvents[rnti] = Simulator::Schedule(txDuration,
                                                  &NbIotEnbPhy::CompleteNpdschTransmission,
                                                  this, rnti);
    
    NS_LOG_INFO("eNB " << m_cellId << " transmitting NPDSCH for RNTI " << rnti
                << ": TBS=" << tbs << ", repetitions=" << static_cast<int>(repetitions));
}

void
NbIotEnbPhy::StartNprachReception(NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << static_cast<int>(coverageClass));
    
    m_isReceivingNprach = true;
    m_nprachCoverageClass = coverageClass;
    
    NS_LOG_INFO("eNB " << m_cellId << " started NPRACH reception window for "
                << CoverageClassToString(coverageClass));
}

void
NbIotEnbPhy::StopNprachReception()
{
    NS_LOG_FUNCTION(this);
    
    m_isReceivingNprach = false;
    
    NS_LOG_INFO("eNB " << m_cellId << " stopped NPRACH reception window");
}

void
NbIotEnbPhy::ReceiveNprach(uint8_t preambleIndex, uint16_t timingAdvance,
                            NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << static_cast<int>(preambleIndex) << timingAdvance
                         << static_cast<int>(coverageClass));
    
    m_nprachRxTrace(preambleIndex, timingAdvance, coverageClass);
    
    NS_LOG_INFO("eNB " << m_cellId << " received NPRACH preamble " << static_cast<int>(preambleIndex)
                << " with TA=" << timingAdvance << " from " << CoverageClassToString(coverageClass));
    
    if (!m_nprachReceivedCallback.IsNull())
    {
        m_nprachReceivedCallback(preambleIndex, timingAdvance, coverageClass);
    }
    
    // Notify MAC layer for RAR scheduling
    if (m_mac)
    {
        // MAC will schedule RAR response
    }
}

void
NbIotEnbPhy::StartNpuschReception(uint16_t rnti, uint16_t tbs, uint8_t numSubcarriers,
                                   uint8_t repetitions, uint16_t startSubframe)
{
    NS_LOG_FUNCTION(this << rnti << tbs << static_cast<int>(numSubcarriers)
                         << static_cast<int>(repetitions) << startSubframe);
    
    auto it = m_attachedUes.find(rnti);
    if (it != m_attachedUes.end())
    {
        it->second.isReceivingNpusch = true;
        it->second.npuschTbs = tbs;
        it->second.npuschRepetitions = repetitions;
    }
    
    m_activeNpuschReceptions.insert(rnti);
    
    NS_LOG_INFO("eNB " << m_cellId << " expecting NPUSCH from RNTI " << rnti
                << ": TBS=" << tbs << ", subcarriers=" << static_cast<int>(numSubcarriers)
                << ", repetitions=" << static_cast<int>(repetitions));
}

void
NbIotEnbPhy::ReceiveNpusch(uint16_t rnti, Ptr<Packet> packet, double sinr)
{
    NS_LOG_FUNCTION(this << rnti << packet << sinr);
    
    m_activeNpuschReceptions.erase(rnti);
    
    auto it = m_attachedUes.find(rnti);
    if (it != m_attachedUes.end())
    {
        it->second.isReceivingNpusch = false;
    }
    
    // Check if decoding was successful
    const double sinrThreshold = -6.0; // dB
    bool success = (packet != nullptr) && (sinr >= sinrThreshold);
    
    uint16_t tbs = (it != m_attachedUes.end()) ? it->second.npuschTbs : 0;
    
    m_phyRxTrace(m_cellId, rnti, NbIotPhysicalChannel::NPUSCH, sinr, success);
    m_npuschRxTrace(rnti, tbs, success);
    
    NS_LOG_INFO("eNB " << m_cellId << " received NPUSCH from RNTI " << rnti
                << ": SINR=" << sinr << " dB, success=" << success);
    
    if (!m_npuschReceivedCallback.IsNull())
    {
        m_npuschReceivedCallback(rnti, packet, success);
    }
    
    // Notify MAC layer
    if (m_mac && success)
    {
        // MAC will process received data
    }
}

void
NbIotEnbPhy::ReceiveNpuschFormat2(uint16_t rnti, bool ack)
{
    NS_LOG_FUNCTION(this << rnti << ack);
    
    NS_LOG_INFO("eNB " << m_cellId << " received HARQ " << (ack ? "ACK" : "NACK")
                << " from RNTI " << rnti);
    
    if (!m_harqFeedbackCallback.IsNull())
    {
        m_harqFeedbackCallback(rnti, ack);
    }
    
    // Notify MAC layer for HARQ process handling
    if (m_mac)
    {
        // MAC will handle retransmission if needed
    }
}

void
NbIotEnbPhy::AddUe(uint16_t rnti, NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<int>(coverageClass));
    
    UeInfo info;
    info.rnti = rnti;
    info.coverageClass = coverageClass;
    info.isReceivingNpusch = false;
    info.npuschTbs = 0;
    info.npuschRepetitions = 0;
    
    m_attachedUes[rnti] = info;
    
    NS_LOG_INFO("eNB " << m_cellId << " added UE RNTI " << rnti
                << " with " << CoverageClassToString(coverageClass));
}

void
NbIotEnbPhy::RemoveUe(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    m_attachedUes.erase(rnti);
    m_activeNpuschReceptions.erase(rnti);
    
    auto npdcchIt = m_npdcchTxEvents.find(rnti);
    if (npdcchIt != m_npdcchTxEvents.end())
    {
        if (npdcchIt->second.IsPending())
        {
            Simulator::Cancel(npdcchIt->second);
        }
        m_npdcchTxEvents.erase(npdcchIt);
    }
    
    auto npdschIt = m_npdschTxEvents.find(rnti);
    if (npdschIt != m_npdschTxEvents.end())
    {
        if (npdschIt->second.IsPending())
        {
            Simulator::Cancel(npdschIt->second);
        }
        m_npdschTxEvents.erase(npdschIt);
    }
    
    NS_LOG_INFO("eNB " << m_cellId << " removed UE RNTI " << rnti);
}

void
NbIotEnbPhy::UpdateUeCoverageClass(uint16_t rnti, NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<int>(coverageClass));
    
    auto it = m_attachedUes.find(rnti);
    if (it != m_attachedUes.end())
    {
        it->second.coverageClass = coverageClass;
        NS_LOG_INFO("eNB " << m_cellId << " updated UE RNTI " << rnti
                    << " to " << CoverageClassToString(coverageClass));
    }
}

uint32_t
NbIotEnbPhy::GetNumAttachedUes() const
{
    return static_cast<uint32_t>(m_attachedUes.size());
}

void
NbIotEnbPhy::SetNprachReceivedCallback(NprachReceivedCallback callback)
{
    m_nprachReceivedCallback = callback;
}

void
NbIotEnbPhy::SetNpuschReceivedCallback(NpuschReceivedCallback callback)
{
    m_npuschReceivedCallback = callback;
}

void
NbIotEnbPhy::SetHarqFeedbackCallback(HarqFeedbackCallback callback)
{
    m_harqFeedbackCallback = callback;
}

void
NbIotEnbPhy::SendControlMessage(Ptr<NbIotControlMessage> msg)
{
    NS_LOG_FUNCTION(this << msg);
    
    // eNB sends control messages to UEs via downlink physical channels
}

void
NbIotEnbPhy::ReceiveControlMessage(Ptr<NbIotControlMessage> msg)
{
    NS_LOG_FUNCTION(this << msg);
    
    // Process received control message
}

void
NbIotEnbPhy::StartSubframe()
{
    NbIotPhy::StartSubframe();
    
    NS_LOG_DEBUG("eNB " << m_cellId << " starting subframe " << m_frameNumber
                 << "." << static_cast<int>(m_subframeNumber));
    
    // Always transmit NRS
    TransmitNrs();
    
    // Check for synchronization signal transmission
    if (IsNpssSubframe())
    {
        TransmitNpss();
    }
    
    if (IsNsssSubframe())
    {
        TransmitNsss();
    }
    
    // Check for NPBCH transmission
    if (IsNpbchSubframe())
    {
        TransmitNpbch();
    }
    
    // Request scheduling decisions from MAC
    if (m_mac)
    {
        // MAC will provide NPDCCH/NPDSCH scheduling for this subframe
    }
}

void
NbIotEnbPhy::CompleteNpdcchTransmission(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    m_npdcchTxEvents.erase(rnti);
    
    NS_LOG_DEBUG("eNB " << m_cellId << " completed NPDCCH transmission for RNTI " << rnti);
}

void
NbIotEnbPhy::CompleteNpdschTransmission(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    
    m_npdschTxEvents.erase(rnti);
    
    NS_LOG_DEBUG("eNB " << m_cellId << " completed NPDSCH transmission for RNTI " << rnti);
    
    // Notify MAC that transmission is complete, expect HARQ feedback
    if (m_mac)
    {
        // MAC will start HARQ feedback timer
    }
}

std::vector<std::complex<double>>
NbIotEnbPhy::GenerateNpss()
{
    NS_LOG_FUNCTION(this);
    
    // NPSS uses length-11 Zadoff-Chu sequence
    // Per 3GPP TS 36.211 Section 10.2.7.1.1
    
    std::vector<std::complex<double>> npss;
    npss.reserve(11 * 11); // 11 symbols, 11 subcarriers
    
    // Root index u = 5 for NB-IoT NPSS
    const int u = 5;
    const int Nzc = 11;
    
    for (int m = 0; m < 11; ++m) // Symbols
    {
        for (int n = 0; n < 11; ++n) // Subcarriers
        {
            double phase = -M_PI * u * n * (n + 1) / Nzc;
            npss.emplace_back(std::cos(phase), std::sin(phase));
        }
    }
    
    return npss;
}

std::vector<std::complex<double>>
NbIotEnbPhy::GenerateNsss(uint16_t nf)
{
    NS_LOG_FUNCTION(this << nf);
    
    // NSSS sequence depends on cell ID and frame number
    // Per 3GPP TS 36.211 Section 10.2.7.2.1
    
    std::vector<std::complex<double>> nsss;
    nsss.reserve(12 * 11); // 11 symbols, 12 subcarriers
    
    // Simplified NSSS generation
    uint32_t cinit = m_cellId + (nf % 2) * 504;
    auto goldSeq = GenerateGoldSequence(12 * 11 * 2, cinit);
    
    for (size_t i = 0; i + 1 < goldSeq.size(); i += 2)
    {
        double real = (1 - 2 * goldSeq[i]) / std::sqrt(2.0);
        double imag = (1 - 2 * goldSeq[i + 1]) / std::sqrt(2.0);
        nsss.emplace_back(real, imag);
    }
    
    return nsss;
}

std::vector<uint8_t>
NbIotEnbPhy::EncodeMibNb()
{
    NS_LOG_FUNCTION(this);
    
    // MIB-NB is 34 bits per 3GPP TS 36.331
    std::vector<uint8_t> mibBits(34, 0);
    
    // systemFrameNumber-MSB (4 bits)
    for (int i = 0; i < 4; ++i)
    {
        mibBits[i] = (m_mib.systemFrameNumber >> (3 - i)) & 1;
    }
    
    // hyperSFN-LSB (2 bits)
    mibBits[4] = (m_mib.hyperFrameNumber >> 1) & 1;
    mibBits[5] = m_mib.hyperFrameNumber & 1;
    
    // schedulingInfoSIB1 (4 bits)
    for (int i = 0; i < 4; ++i)
    {
        mibBits[6 + i] = (m_mib.schedulingInfoSib1 >> (3 - i)) & 1;
    }
    
    // operationModeInfo (4 bits minimum)
    mibBits[10] = static_cast<uint8_t>(m_mib.deploymentMode) >> 1;
    mibBits[11] = static_cast<uint8_t>(m_mib.deploymentMode) & 1;
    
    // Spare bits filled with 0
    
    return mibBits;
}

std::vector<uint8_t>
NbIotEnbPhy::EncodeDci(const NbIotDci& dci)
{
    NS_LOG_FUNCTION(this);
    
    std::vector<uint8_t> dciBits;
    
    switch (dci.format)
    {
        case NbIotDciFormat::DCI_N0:
            // DCI Format N0: 23 bits (without CRC)
            dciBits.resize(23);
            // Subcarrier indication (6 bits)
            for (int i = 0; i < 6; ++i)
            {
                dciBits[i] = (dci.subcarrierIndication >> (5 - i)) & 1;
            }
            // Resource assignment (3 bits)
            for (int i = 0; i < 3; ++i)
            {
                dciBits[6 + i] = (dci.resourceAssignment >> (2 - i)) & 1;
            }
            // Scheduling delay (2 bits)
            dciBits[9] = (dci.schedulingDelay >> 1) & 1;
            dciBits[10] = dci.schedulingDelay & 1;
            // MCS (4 bits)
            for (int i = 0; i < 4; ++i)
            {
                dciBits[11 + i] = (dci.mcs >> (3 - i)) & 1;
            }
            // Repetition number (3 bits)
            for (int i = 0; i < 3; ++i)
            {
                dciBits[15 + i] = (dci.repetitionNumber >> (2 - i)) & 1;
            }
            // NDI (1 bit)
            dciBits[18] = dci.ndi;
            // DCI subframe repetition (2 bits)
            // Spare/reserved
            break;
            
        case NbIotDciFormat::DCI_N1:
            // DCI Format N1: 23 bits (without CRC)
            dciBits.resize(23);
            // NPDCCH order indicator (1 bit)
            dciBits[0] = 0;
            // Scheduling delay (3 bits)
            for (int i = 0; i < 3; ++i)
            {
                dciBits[1 + i] = (dci.schedulingDelay >> (2 - i)) & 1;
            }
            // Resource assignment (3 bits)
            for (int i = 0; i < 3; ++i)
            {
                dciBits[4 + i] = (dci.resourceAssignment >> (2 - i)) & 1;
            }
            // MCS (4 bits)
            for (int i = 0; i < 4; ++i)
            {
                dciBits[7 + i] = (dci.mcs >> (3 - i)) & 1;
            }
            // Repetition number (4 bits)
            for (int i = 0; i < 4; ++i)
            {
                dciBits[11 + i] = (dci.repetitionNumber >> (3 - i)) & 1;
            }
            // NDI (1 bit)
            dciBits[15] = dci.ndi;
            // HARQ-ACK resource (4 bits)
            // DCI subframe repetition (2 bits)
            break;
            
        case NbIotDciFormat::DCI_N2:
            // DCI Format N2 for paging
            dciBits.resize(15);
            // Direct indication flag
            dciBits[0] = dci.directIndication ? 1 : 0;
            // Resource assignment and repetition
            break;
    }
    
    return dciBits;
}

} // namespace ns3
