/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT UE Physical Layer implementation
 */

#include "nbiot-ue-phy.h"
#include "nbiot-ue-mac.h"
#include "nbiot-control-messages.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/double.h>
#include <ns3/uinteger.h>
#include <ns3/boolean.h>
#include <ns3/pointer.h>

#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotUePhy");

NS_OBJECT_ENSURE_REGISTERED(NbIotUePhy);

TypeId
NbIotUePhy::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotUePhy")
        .SetParent<NbIotPhy>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotUePhy>()
        // Note: TxPower attribute is inherited from NbIotPhy parent class
        .AddAttribute("RsrpThresholdCe0",
                      "RSRP threshold for CE Level 0 (dBm)",
                      DoubleValue(-110.0),
                      MakeDoubleAccessor(&NbIotUePhy::m_rsrpThresholdCe0),
                      MakeDoubleChecker<double>())
        .AddAttribute("RsrpThresholdCe1",
                      "RSRP threshold for CE Level 1 (dBm)",
                      DoubleValue(-120.0),
                      MakeDoubleAccessor(&NbIotUePhy::m_rsrpThresholdCe1),
                      MakeDoubleChecker<double>())
        .AddAttribute("P0Nominal",
                      "P0 nominal for uplink power control (dBm)",
                      DoubleValue(-96.0),
                      MakeDoubleAccessor(&NbIotUePhy::m_p0Nominal),
                      MakeDoubleChecker<double>())
        .AddAttribute("Alpha",
                      "Path loss compensation factor (0-1)",
                      DoubleValue(1.0),
                      MakeDoubleAccessor(&NbIotUePhy::m_alpha),
                      MakeDoubleChecker<double>(0.0, 1.0))
        .AddTraceSource("RsrpRsrq",
                        "RSRP and RSRQ measurements",
                        MakeTraceSourceAccessor(&NbIotUePhy::m_rsrpRsrqTrace),
                        "ns3::NbIotUePhy::RsrpRsrqTracedCallback")
        .AddTraceSource("CoverageClass",
                        "Coverage class changes",
                        MakeTraceSourceAccessor(&NbIotUePhy::m_coverageClassTrace),
                        "ns3::NbIotUePhy::CoverageClassTracedCallback");
    return tid;
}

NbIotUePhy::NbIotUePhy()
    : m_rnti(0)
    , m_coverageClass(NbIotCoverageClass::CE_LEVEL_0)
    , m_isSearchingCell(false)
    , m_isMonitoringNpdcch(false)
    , m_isReceivingNpdsch(false)
    , m_isTransmittingNpusch(false)
    , m_isTransmittingNprach(false)
    , m_npdschTbs(0)
    , m_npdschRepetitions(0)
    , m_npdschRepetitionCount(0)
    , m_npdschStartSubframe(0)
    , m_npuschRepetitions(0)
    , m_npuschRepetitionCount(0)
    , m_npuschNumSubcarriers(12)
    , m_nprachRepetitions(0)
    , m_nprachRepetitionCount(0)
    , m_p0Nominal(-96.0)
    , m_alpha(1.0)
    , m_pathLoss(100.0)
    , m_rsrpThresholdCe0(-110.0)
    , m_rsrpThresholdCe1(-120.0)
{
    NS_LOG_FUNCTION(this);
    
    // Initialize channel quality
    m_channelQuality.rsrp = -100.0;
    m_channelQuality.rsrq = -10.0;
    m_channelQuality.sinr = 10.0;
    m_channelQuality.snr = 10.0;
    m_channelQuality.estimatedCoverageClass = NbIotCoverageClass::CE_LEVEL_0;
}

NbIotUePhy::~NbIotUePhy()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotUePhy::DoDispose()
{
    NS_LOG_FUNCTION(this);
    
    m_mac = nullptr;
    m_pendingNpuschPacket = nullptr;
    
    if (m_npuschTxEvent.IsPending())
    {
        Simulator::Cancel(m_npuschTxEvent);
    }
    if (m_nprachTxEvent.IsPending())
    {
        Simulator::Cancel(m_nprachTxEvent);
    }
    if (m_cellSearchEvent.IsPending())
    {
        Simulator::Cancel(m_cellSearchEvent);
    }
    
    NbIotPhy::DoDispose();
}

void
NbIotUePhy::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    NbIotPhy::DoInitialize();
}

void
NbIotUePhy::SetMac(Ptr<NbIotUeMac> mac)
{
    NS_LOG_FUNCTION(this << mac);
    m_mac = mac;
}

Ptr<NbIotUeMac>
NbIotUePhy::GetMac() const
{
    return m_mac;
}

void
NbIotUePhy::SetRnti(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    m_rnti = rnti;
}

uint16_t
NbIotUePhy::GetRnti() const
{
    return m_rnti;
}

void
NbIotUePhy::SetCoverageClass(NbIotCoverageClass ceLevel)
{
    NS_LOG_FUNCTION(this << static_cast<int>(ceLevel));
    
    if (m_coverageClass != ceLevel)
    {
        NS_LOG_INFO("UE " << m_rnti << " coverage class changed from "
                   << CoverageClassToString(m_coverageClass) << " to "
                   << CoverageClassToString(ceLevel));
        
        m_coverageClass = ceLevel;
        m_coverageClassTrace(m_rnti, ceLevel);
    }
}

NbIotCoverageClass
NbIotUePhy::GetCoverageClass() const
{
    return m_coverageClass;
}

void
NbIotUePhy::StartCellSearch()
{
    NS_LOG_FUNCTION(this);
    
    m_isSearchingCell = true;
    
    // Simulate cell search duration (typically 400-500 ms for NPSS/NSSS detection)
    // NPSS periodicity: 10 ms, NSSS periodicity: 20 ms
    Time searchDuration = MilliSeconds(500);
    
    m_cellSearchEvent = Simulator::Schedule(searchDuration, &NbIotUePhy::CellSearchCompleted,
                                            this, m_cellId, true);
    
    NS_LOG_INFO("UE started cell search, expected duration: " << searchDuration.GetMilliSeconds() << " ms");
}

void
NbIotUePhy::CellSearchCompleted(uint16_t cellId, bool success)
{
    NS_LOG_FUNCTION(this << cellId << success);
    
    m_isSearchingCell = false;
    
    if (success)
    {
        m_cellId = cellId;
        NS_LOG_INFO("Cell search completed successfully, detected cell ID: " << cellId);
        
        // Start NPBCH reception for MIB-NB
        StartNpbchReception();
    }
    else
    {
        NS_LOG_WARN("Cell search failed");
    }
    
    if (!m_cellSearchCallback.IsNull())
    {
        m_cellSearchCallback(cellId, success);
    }
}

void
NbIotUePhy::StartNpbchReception()
{
    NS_LOG_FUNCTION(this);
    
    // NPBCH has 640 ms periodicity with 8 repetitions
    // UE needs to accumulate samples over multiple repetitions for decoding
    
    NS_LOG_INFO("UE started NPBCH reception for MIB-NB");
    
    // Simulate MIB-NB reception (simplified - actual timing depends on NPBCH schedule)
    Time mibReceptionTime = MilliSeconds(80); // Minimum with good coverage
    
    Simulator::Schedule(mibReceptionTime, [this]() {
        NbIotMib mib;
        mib.systemFrameNumber = m_frameNumber >> 6; // 4 MSBs
        mib.hyperFrameNumber = 0;
        mib.schedulingInfoSib1 = 0;
        mib.operationModeInfo = false;
        mib.deploymentMode = m_deploymentMode;
        
        ReceiveMib(mib);
    });
}

void
NbIotUePhy::ReceiveMib(const NbIotMib& mib)
{
    NS_LOG_FUNCTION(this);
    
    NS_LOG_INFO("UE received MIB-NB: SFN=" << static_cast<int>(mib.systemFrameNumber)
                << ", mode=" << DeploymentModeToString(mib.deploymentMode));
    
    if (!m_mibReceivedCallback.IsNull())
    {
        m_mibReceivedCallback(mib);
    }
    
    // Notify MAC layer
    if (m_mac)
    {
        // MAC will handle SIB reading and connection procedures
    }
}

void
NbIotUePhy::StartNpdcchMonitoring(uint8_t searchSpace)
{
    NS_LOG_FUNCTION(this << static_cast<int>(searchSpace));
    
    m_isMonitoringNpdcch = true;
    
    NS_LOG_INFO("UE " << m_rnti << " started NPDCCH monitoring, search space type: "
                << static_cast<int>(searchSpace));
}

void
NbIotUePhy::StopNpdcchMonitoring()
{
    NS_LOG_FUNCTION(this);
    
    m_isMonitoringNpdcch = false;
    
    NS_LOG_INFO("UE " << m_rnti << " stopped NPDCCH monitoring");
}

void
NbIotUePhy::ReceiveDci(const NbIotDci& dci)
{
    NS_LOG_FUNCTION(this << static_cast<int>(dci.format));
    
    NS_LOG_INFO("UE " << m_rnti << " received DCI format N"
                << static_cast<int>(dci.format) << ", RNTI: " << dci.rnti);
    
    // Check if DCI is for this UE
    if (dci.rnti != m_rnti && dci.rnti != 0xFFFF) // 0xFFFF for common RNTI
    {
        NS_LOG_DEBUG("DCI not for this UE, ignoring");
        return;
    }
    
    if (!m_dciReceivedCallback.IsNull())
    {
        m_dciReceivedCallback(dci);
    }
    
    // Process DCI based on format
    switch (dci.format)
    {
        case NbIotDciFormat::DCI_N0:
            // Uplink grant - prepare NPUSCH transmission
            NS_LOG_INFO("Received uplink grant");
            break;
            
        case NbIotDciFormat::DCI_N1:
            // Downlink assignment - prepare NPDSCH reception
            {
                uint16_t tbs = GetTbsFromIndex(dci.mcs);
                uint8_t reps = NPDSCH_REPETITIONS[dci.repetitionNumber % NPDSCH_REPETITIONS.size()];
                StartNpdschReception(tbs, reps, m_subframeNumber + dci.schedulingDelay);
            }
            break;
            
        case NbIotDciFormat::DCI_N2:
            // Paging
            NS_LOG_INFO("Received paging indication");
            break;
    }
}

void
NbIotUePhy::StartNpdschReception(uint16_t tbs, uint8_t repetitions, uint16_t startSubframe)
{
    NS_LOG_FUNCTION(this << tbs << static_cast<int>(repetitions) << startSubframe);
    
    m_isReceivingNpdsch = true;
    m_npdschTbs = tbs;
    m_npdschRepetitions = repetitions;
    m_npdschRepetitionCount = 0;
    m_npdschStartSubframe = startSubframe;
    
    NS_LOG_INFO("UE " << m_rnti << " starting NPDSCH reception: TBS=" << tbs
                << ", repetitions=" << static_cast<int>(repetitions)
                << ", start SF=" << startSubframe);
}

void
NbIotUePhy::ReceiveNpdsch(Ptr<Packet> packet, double sinr)
{
    NS_LOG_FUNCTION(this << packet << sinr);
    
    m_isReceivingNpdsch = false;
    
    // Check if decoding was successful based on SINR
    // NPDSCH uses QPSK, so SINR threshold is lower
    const double sinrThreshold = -6.0; // dB, conservative for QPSK with repetitions
    
    bool success = sinr >= sinrThreshold;
    
    m_phyRxTrace(m_cellId, m_rnti, NbIotPhysicalChannel::NPDSCH, sinr, success);
    
    if (success)
    {
        NS_LOG_INFO("UE " << m_rnti << " successfully received NPDSCH, SINR=" << sinr << " dB");
        
        if (!m_npdschReceivedCallback.IsNull())
        {
            m_npdschReceivedCallback(packet);
        }
        
        // Send HARQ ACK
        TransmitNpuschFormat2(true, GetMaxRepetitions(m_coverageClass, NbIotPhysicalChannel::NPUSCH));
    }
    else
    {
        NS_LOG_WARN("UE " << m_rnti << " failed to decode NPDSCH, SINR=" << sinr << " dB");
        
        // Send HARQ NACK
        TransmitNpuschFormat2(false, GetMaxRepetitions(m_coverageClass, NbIotPhysicalChannel::NPUSCH));
    }
}

void
NbIotUePhy::TransmitNpuschFormat1(Ptr<Packet> packet, uint16_t tbs, uint8_t numSubcarriers,
                                   uint8_t repetitions, NbIotSubcarrierSpacing subcarrierSpacing)
{
    NS_LOG_FUNCTION(this << packet << tbs << static_cast<int>(numSubcarriers)
                         << static_cast<int>(repetitions));
    
    m_isTransmittingNpusch = true;
    m_pendingNpuschPacket = packet;
    m_npuschNumSubcarriers = numSubcarriers;
    m_npuschRepetitions = repetitions;
    m_npuschRepetitionCount = 0;
    
    // Calculate transmission duration based on configuration
    // Resource unit size depends on number of subcarriers
    uint16_t ruSlots;
    switch (numSubcarriers)
    {
        case 1:
            ruSlots = 16; // 8 ms for 15 kHz, 32 ms for 3.75 kHz
            break;
        case 3:
            ruSlots = 8;  // 4 ms
            break;
        case 6:
            ruSlots = 4;  // 2 ms
            break;
        case 12:
        default:
            ruSlots = 2;  // 1 ms
            break;
    }
    
    Time txDuration = MilliSeconds(ruSlots * 0.5 * repetitions);
    
    double txPower = CalculateUlTxPower(numSubcarriers);
    
    NS_LOG_INFO("UE " << m_rnti << " transmitting NPUSCH Format 1: TBS=" << tbs
                << ", subcarriers=" << static_cast<int>(numSubcarriers)
                << ", repetitions=" << static_cast<int>(repetitions)
                << ", TxPower=" << txPower << " dBm"
                << ", duration=" << txDuration.GetMilliSeconds() << " ms");
    
    m_phyTxTrace(m_cellId, m_rnti, NbIotPhysicalChannel::NPUSCH, txPower);
    
    m_npuschTxEvent = Simulator::Schedule(txDuration, &NbIotUePhy::CompleteNpuschTransmission, this);
}

void
NbIotUePhy::TransmitNpuschFormat2(bool ack, uint8_t repetitions)
{
    NS_LOG_FUNCTION(this << ack << static_cast<int>(repetitions));
    
    // NPUSCH Format 2 is single-tone only
    m_isTransmittingNpusch = true;
    m_npuschNumSubcarriers = 1;
    m_npuschRepetitions = repetitions;
    m_npuschRepetitionCount = 0;
    
    // Format 2 uses 1 slot per repetition
    Time txDuration = MilliSeconds(0.5 * repetitions);
    
    double txPower = CalculateUlTxPower(1);
    
    NS_LOG_INFO("UE " << m_rnti << " transmitting NPUSCH Format 2 ("
                << (ack ? "ACK" : "NACK") << "): repetitions="
                << static_cast<int>(repetitions) << ", TxPower=" << txPower << " dBm");
    
    m_phyTxTrace(m_cellId, m_rnti, NbIotPhysicalChannel::NPUSCH, txPower);
    
    m_npuschTxEvent = Simulator::Schedule(txDuration, &NbIotUePhy::CompleteNpuschTransmission, this);
}

void
NbIotUePhy::TransmitNprach(NbIotNprachFormat preambleFormat, uint8_t repetitions,
                            NbIotCoverageClass coverageClass)
{
    NS_LOG_FUNCTION(this << static_cast<int>(preambleFormat)
                         << static_cast<int>(repetitions)
                         << static_cast<int>(coverageClass));
    
    m_isTransmittingNprach = true;
    m_nprachRepetitions = repetitions;
    m_nprachRepetitionCount = 0;
    
    // NPRACH preamble duration depends on format
    // Format 0: 5.6 ms per symbol group (4 groups = 22.4 ms base)
    // Format 1: 16.8 ms per symbol group
    double baseDurationMs = (preambleFormat == NbIotNprachFormat::FORMAT_0) ? 22.4 : 67.2;
    Time txDuration = MilliSeconds(baseDurationMs * repetitions);
    
    double txPower = m_txPowerDbm; // Full power for NPRACH
    
    NS_LOG_INFO("UE transmitting NPRACH: format=" << static_cast<int>(preambleFormat)
                << ", repetitions=" << static_cast<int>(repetitions)
                << ", coverage class=" << CoverageClassToString(coverageClass)
                << ", duration=" << txDuration.GetMilliSeconds() << " ms");
    
    m_phyTxTrace(m_cellId, m_rnti, NbIotPhysicalChannel::NPRACH, txPower);
    
    m_nprachTxEvent = Simulator::Schedule(txDuration, &NbIotUePhy::CompleteNprachTransmission, this);
}

void
NbIotUePhy::CompleteNpuschTransmission()
{
    NS_LOG_FUNCTION(this);
    
    m_isTransmittingNpusch = false;
    m_pendingNpuschPacket = nullptr;
    
    NS_LOG_INFO("UE " << m_rnti << " completed NPUSCH transmission");
    
    // Notify MAC layer of transmission completion
    if (m_mac)
    {
        // MAC will handle HARQ feedback waiting
    }
}

void
NbIotUePhy::CompleteNprachTransmission()
{
    NS_LOG_FUNCTION(this);
    
    m_isTransmittingNprach = false;
    
    NS_LOG_INFO("UE completed NPRACH transmission, waiting for RAR");
    
    // Notify MAC layer to start RAR window monitoring
    if (m_mac)
    {
        // MAC will handle RAR reception
    }
}

double
NbIotUePhy::MeasureRsrp()
{
    NS_LOG_FUNCTION(this);
    
    // Measure RSRP based on NRS
    // Simplified model - actual implementation would use received NRS power
    double rsrp = m_channelQuality.rsrp;
    
    m_rsrpRsrqTrace(m_rnti, rsrp, m_channelQuality.rsrq);
    
    return rsrp;
}

double
NbIotUePhy::MeasureRsrq()
{
    NS_LOG_FUNCTION(this);
    
    // RSRQ = N * RSRP / RSSI, where N = number of RBs (1 for NB-IoT)
    // Simplified model
    return m_channelQuality.rsrq;
}

NbIotChannelQuality
NbIotUePhy::GetChannelQuality() const
{
    return m_channelQuality;
}

NbIotCoverageClass
NbIotUePhy::DetermineCoverageClass(double rsrp) const
{
    NS_LOG_FUNCTION(this << rsrp);
    
    if (rsrp >= m_rsrpThresholdCe0)
    {
        return NbIotCoverageClass::CE_LEVEL_0;
    }
    else if (rsrp >= m_rsrpThresholdCe1)
    {
        return NbIotCoverageClass::CE_LEVEL_1;
    }
    else
    {
        return NbIotCoverageClass::CE_LEVEL_2;
    }
}

void
NbIotUePhy::SetPowerControl(double p0, double alpha)
{
    NS_LOG_FUNCTION(this << p0 << alpha);
    m_p0Nominal = p0;
    m_alpha = alpha;
}

double
NbIotUePhy::CalculateUlTxPower(uint8_t numSubcarriers)
{
    NS_LOG_FUNCTION(this << static_cast<int>(numSubcarriers));
    
    // Per 3GPP TS 36.213 Section 16.2.1.1
    // P = min(Pcmax, P0 + alpha * PL + 10*log10(M))
    // where M is the number of subcarriers
    
    double pcmax = NbIotConstants::UE_MAX_TX_POWER_DBM;
    double p = m_p0Nominal + m_alpha * m_pathLoss + 10.0 * std::log10(static_cast<double>(numSubcarriers));
    
    return std::min(pcmax, p);
}

void
NbIotUePhy::SetCellSearchCallback(CellSearchCallback callback)
{
    m_cellSearchCallback = callback;
}

void
NbIotUePhy::SetMibReceivedCallback(MibReceivedCallback callback)
{
    m_mibReceivedCallback = callback;
}

void
NbIotUePhy::SetDciReceivedCallback(DciReceivedCallback callback)
{
    m_dciReceivedCallback = callback;
}

void
NbIotUePhy::SetNpdschReceivedCallback(NpdschReceivedCallback callback)
{
    m_npdschReceivedCallback = callback;
}

void
NbIotUePhy::SendControlMessage(Ptr<NbIotControlMessage> msg)
{
    NS_LOG_FUNCTION(this << msg);
    
    // UE sends control messages to eNB via uplink physical channels
    // This would be handled by the spectrum interface
}

void
NbIotUePhy::ReceiveControlMessage(Ptr<NbIotControlMessage> msg)
{
    NS_LOG_FUNCTION(this << msg);
    
    // Process received control message based on type
    // Dispatch to appropriate handler
}

void
NbIotUePhy::StartSubframe()
{
    NbIotPhy::StartSubframe();
    
    NS_LOG_DEBUG("UE " << m_rnti << " starting subframe " << m_frameNumber
                 << "." << static_cast<int>(m_subframeNumber));
    
    // Check if we should be monitoring NPDCCH
    if (m_isMonitoringNpdcch)
    {
        ProcessNpdcchCandidates();
    }
    
    // Update channel quality periodically
    if (m_subframeNumber == 0)
    {
        UpdateChannelQuality();
    }
}

void
NbIotUePhy::ProcessNpdcchCandidates()
{
    NS_LOG_FUNCTION(this);
    
    // Check search space for this subframe
    // Simplified - actual implementation would follow TS 36.213 Section 16.6
    
    // NPDCCH monitoring occasions depend on:
    // - Search space type (Type-1 UE-specific, Type-2 common)
    // - Coverage class (determines starting subframe and repetitions)
    // - UE-specific search space based on RNTI
}

void
NbIotUePhy::UpdateChannelQuality()
{
    NS_LOG_FUNCTION(this);
    
    // Perform channel quality measurements
    m_channelQuality.rsrp = MeasureRsrp();
    m_channelQuality.rsrq = MeasureRsrq();
    
    // Estimate SINR from RSRP and RSRQ
    // RSRQ = N * RSRP / RSSI => RSSI = RSRP / RSRQ (for N=1)
    double rssiDb = m_channelQuality.rsrp - m_channelQuality.rsrq;
    double interferenceDb = rssiDb - m_channelQuality.rsrp;
    m_channelQuality.sinr = m_channelQuality.rsrp - interferenceDb;
    
    // Update coverage class
    NbIotCoverageClass newClass = DetermineCoverageClass(m_channelQuality.rsrp);
    if (newClass != m_coverageClass)
    {
        SetCoverageClass(newClass);
    }
    
    m_channelQuality.estimatedCoverageClass = m_coverageClass;
}

} // namespace ns3
