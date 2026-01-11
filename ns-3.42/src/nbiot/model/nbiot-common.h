/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * This file contains common NB-IoT definitions as specified in 3GPP TS 36.211,
 * TS 36.212, TS 36.213, and TS 36.321 for Release 13/14.
 */

#ifndef NBIOT_COMMON_H
#define NBIOT_COMMON_H

#include <ns3/object.h>
#include <ns3/nstime.h>

#include <cstdint>
#include <vector>
#include <array>

namespace ns3 {

/**
 * \defgroup nbiot NB-IoT Module
 * \brief NB-IoT (Narrowband Internet of Things) simulation module for NS-3
 *
 * This module implements the NB-IoT physical and protocol stack according to
 * 3GPP Release 13/14 specifications. NB-IoT is designed for low-power wide-area
 * network (LPWAN) applications requiring extended coverage and long battery life.
 *
 * Key features:
 * - 180 kHz system bandwidth (1 Physical Resource Block)
 * - Three deployment modes: standalone, guard-band, and in-band
 * - Extended coverage with up to 164 dB Maximum Coupling Loss (MCL)
 * - Power Saving Mode (PSM) and extended DRX (eDRX)
 * - Optimized for massive IoT deployments
 *
 * \see 3GPP TS 36.211: Physical channels and modulation
 * \see 3GPP TS 36.213: Physical layer procedures
 * \see 3GPP TS 36.321: MAC protocol specification
 * \see 3GPP TS 36.331: RRC protocol specification
 */

/**
 * \ingroup nbiot
 * \brief NB-IoT deployment modes as per 3GPP TS 36.211 Section 10.1.3.5
 */
enum class NbIotDeploymentMode : uint8_t
{
    STANDALONE,   ///< Standalone deployment using dedicated spectrum (e.g., GSM refarming)
    GUARD_BAND,   ///< Guard-band deployment within LTE carrier guard band
    IN_BAND       ///< In-band deployment within LTE carrier using one PRB
};

/**
 * \ingroup nbiot
 * \brief NB-IoT operating bands as per 3GPP TS 36.101
 */
enum class NbIotOperatingBand : uint8_t
{
    BAND_1  = 1,   ///< 2100 MHz (FDD)
    BAND_2  = 2,   ///< 1900 MHz (FDD)
    BAND_3  = 3,   ///< 1800 MHz (FDD)
    BAND_5  = 5,   ///< 850 MHz (FDD)
    BAND_8  = 8,   ///< 900 MHz (FDD)
    BAND_12 = 12,  ///< 700 MHz (FDD)
    BAND_13 = 13,  ///< 700 MHz (FDD)
    BAND_17 = 17,  ///< 700 MHz (FDD)
    BAND_18 = 18,  ///< 850 MHz (FDD)
    BAND_19 = 19,  ///< 850 MHz (FDD)
    BAND_20 = 20,  ///< 800 MHz (FDD)
    BAND_25 = 25,  ///< 1900 MHz (FDD)
    BAND_26 = 26,  ///< 850 MHz (FDD)
    BAND_28 = 28,  ///< 700 MHz (FDD)
    BAND_66 = 66   ///< AWS-3 (FDD)
};

/**
 * \ingroup nbiot
 * \brief Coverage Enhancement (CE) levels as per 3GPP TS 36.321 and TS 36.331
 *
 * CE levels determine the number of repetitions for physical channels to achieve
 * extended coverage beyond normal LTE operation.
 */
enum class NbIotCoverageClass : uint8_t
{
    CE_LEVEL_0 = 0,  ///< Normal coverage (0 dB enhancement, MCL ~144 dB)
    CE_LEVEL_1 = 1,  ///< Medium coverage (5 dB enhancement, MCL ~154 dB)
    CE_LEVEL_2 = 2   ///< Extreme coverage (10 dB enhancement, MCL ~164 dB)
};

/**
 * \ingroup nbiot
 * \brief NB-IoT physical channel types as per 3GPP TS 36.211 Section 10
 */
enum class NbIotPhysicalChannel : uint8_t
{
    NPBCH,   ///< Narrowband Physical Broadcast Channel (Downlink)
    NPDCCH,  ///< Narrowband Physical Downlink Control Channel
    NPDSCH,  ///< Narrowband Physical Downlink Shared Channel
    NPUSCH,  ///< Narrowband Physical Uplink Shared Channel
    NPRACH   ///< Narrowband Physical Random Access Channel
};

/**
 * \ingroup nbiot
 * \brief NB-IoT physical signals as per 3GPP TS 36.211 Section 10.2.7
 */
enum class NbIotPhysicalSignal : uint8_t
{
    NRS,   ///< Narrowband Reference Signal
    NPSS,  ///< Narrowband Primary Synchronization Signal
    NSSS   ///< Narrowband Secondary Synchronization Signal
};

/**
 * \ingroup nbiot
 * \brief NPDCCH DCI formats as per 3GPP TS 36.212 Section 6.4.3
 */
enum class NbIotDciFormat : uint8_t
{
    DCI_N0,  ///< Uplink grant for NPUSCH Format 1
    DCI_N1,  ///< Downlink assignment for NPDSCH
    DCI_N2   ///< Paging
};

/**
 * \ingroup nbiot
 * \brief NPUSCH formats as per 3GPP TS 36.211 Section 10.1.3
 */
enum class NbIotNpuschFormat : uint8_t
{
    FORMAT_1,  ///< Uplink data transmission
    FORMAT_2   ///< HARQ-ACK transmission
};

/**
 * \ingroup nbiot
 * \brief NPRACH preamble formats as per 3GPP TS 36.211 Section 10.1.6
 */
enum class NbIotNprachFormat : uint8_t
{
    FORMAT_0,  ///< Preamble format 0 (3.75 kHz subcarrier spacing)
    FORMAT_1   ///< Preamble format 1 (1.25 kHz subcarrier spacing)
};

/**
 * \ingroup nbiot
 * \brief Subcarrier spacing for uplink as per 3GPP TS 36.211
 */
enum class NbIotSubcarrierSpacing : uint8_t
{
    SPACING_15_KHZ,   ///< 15 kHz subcarrier spacing (multi-tone)
    SPACING_3_75_KHZ  ///< 3.75 kHz subcarrier spacing (single-tone)
};

/**
 * \ingroup nbiot
 * \brief Modulation schemes used in NB-IoT as per 3GPP TS 36.211
 */
enum class NbIotModulation : uint8_t
{
    QPSK,      ///< QPSK for downlink and multi-tone uplink
    PI4_QPSK,  ///< π/4-QPSK for single-tone uplink (phase continuity)
    BPSK       ///< BPSK for specific control channels
};

/**
 * \ingroup nbiot
 * \brief RRC states for NB-IoT as per 3GPP TS 36.331
 */
enum class NbIotRrcState : uint8_t
{
    RRC_IDLE,       ///< UE is in idle mode
    RRC_CONNECTED,  ///< UE is connected
    RRC_INACTIVE    ///< UE is in inactive state (Release 15+)
};

/**
 * \ingroup nbiot
 * \brief Power Saving Mode states
 */
enum class NbIotPsmState : uint8_t
{
    PSM_ACTIVE,       ///< Normal operations, monitoring paging
    PSM_LIGHT_SLEEP,  ///< Reduced activity, still monitoring paging
    PSM_DEEP_SLEEP    ///< Deep sleep, no monitoring, only timer running
};

/**
 * \ingroup nbiot
 * \brief RLC modes as per 3GPP TS 36.322
 */
enum class NbIotRlcMode : uint8_t
{
    TM,  ///< Transparent Mode (no overhead, for broadcast)
    UM,  ///< Unacknowledged Mode (segmentation, no ARQ)
    AM   ///< Acknowledged Mode (segmentation with ARQ)
};

/**
 * \ingroup nbiot
 * \brief Logical channel types for NB-IoT
 */
enum class NbIotLogicalChannel : uint8_t
{
    BCCH,   ///< Broadcast Control Channel
    PCCH,   ///< Paging Control Channel
    CCCH,   ///< Common Control Channel
    DCCH,   ///< Dedicated Control Channel
    DTCH    ///< Dedicated Traffic Channel
};

/**
 * \ingroup nbiot
 * \brief Transport channel types for NB-IoT
 */
enum class NbIotTransportChannel : uint8_t
{
    BCH,     ///< Broadcast Channel
    PCH,     ///< Paging Channel
    DL_SCH,  ///< Downlink Shared Channel
    UL_SCH,  ///< Uplink Shared Channel
    RACH     ///< Random Access Channel
};

/**
 * \ingroup nbiot
 * \brief NB-IoT system constants as per 3GPP specifications
 */
struct NbIotConstants
{
    // Physical layer constants (TS 36.211)
    static constexpr double SYSTEM_BANDWIDTH_HZ = 180000.0;           ///< 180 kHz system bandwidth
    static constexpr uint8_t SUBCARRIERS_PER_PRB = 12;                ///< 12 subcarriers in 1 PRB
    static constexpr double DL_SUBCARRIER_SPACING_HZ = 15000.0;       ///< 15 kHz DL subcarrier spacing
    static constexpr double UL_SUBCARRIER_SPACING_15KHZ = 15000.0;    ///< 15 kHz UL subcarrier spacing
    static constexpr double UL_SUBCARRIER_SPACING_3_75KHZ = 3750.0;   ///< 3.75 kHz UL subcarrier spacing
    static constexpr uint8_t OFDM_SYMBOLS_PER_SLOT = 7;               ///< 7 OFDM symbols per slot (normal CP)
    static constexpr uint8_t SLOTS_PER_SUBFRAME = 2;                  ///< 2 slots per subframe
    static constexpr uint8_t SUBFRAMES_PER_FRAME = 10;                ///< 10 subframes per frame (10 ms)
    static constexpr double FRAME_DURATION_MS = 10.0;                 ///< 10 ms frame duration
    static constexpr double SUBFRAME_DURATION_MS = 1.0;               ///< 1 ms subframe duration
    static constexpr double SLOT_DURATION_MS = 0.5;                   ///< 0.5 ms slot duration

    // NPBCH constants (TS 36.211 Section 10.2.4)
    static constexpr double NPBCH_PERIOD_MS = 640.0;                  ///< 640 ms NPBCH period
    static constexpr uint8_t NPBCH_REPETITIONS = 8;                   ///< 8 repetitions per period
    static constexpr uint16_t NPBCH_RESOURCE_ELEMENTS = 3456;         ///< REs over 640 ms

    // NPDCCH constants (TS 36.211 Section 10.2.5)
    static constexpr uint8_t NPDCCH_MAX_AGGREGATION_LEVEL = 2;        ///< Max aggregation level

    // NPDSCH constants (TS 36.211 Section 10.2.3)
    static constexpr uint8_t NPDSCH_SCHEDULING_DELAY_SUBFRAMES = 4;   ///< Minimum scheduling delay

    // NPUSCH constants (TS 36.211 Section 10.1.3)
    static constexpr uint8_t NPUSCH_FORMAT1_MAX_TONES = 12;           ///< Max tones for Format 1
    static constexpr uint8_t NPUSCH_FORMAT2_TONES = 1;                ///< Single tone for Format 2

    // NPRACH constants (TS 36.211 Section 10.1.6)
    static constexpr uint8_t NPRACH_SYMBOL_GROUPS = 4;                ///< Number of symbol groups
    static constexpr uint8_t NPRACH_SYMBOLS_PER_GROUP = 5;            ///< Symbols per group

    // Coverage constants
    static constexpr double MCL_NORMAL_DB = 144.0;                    ///< Normal MCL (dB)
    static constexpr double MCL_ENHANCED_1_DB = 154.0;                ///< CE Level 1 MCL (dB)
    static constexpr double MCL_ENHANCED_2_DB = 164.0;                ///< CE Level 2 MCL (dB)

    // HARQ constants (TS 36.321)
    static constexpr uint8_t NUM_HARQ_PROCESSES = 8;                  ///< 8 HARQ processes (note: NB-IoT may use 2)
    static constexpr uint8_t MAX_HARQ_TX = 10;                        ///< Max HARQ transmissions
    static constexpr uint8_t HARQ_FEEDBACK_DELAY = 12;                ///< Typical feedback delay (subframes)

    // Physical cell ID range
    static constexpr uint16_t MAX_CELL_ID = 503;                      ///< Max physical cell ID

    // Power control
    static constexpr double UE_MAX_TX_POWER_DBM = 23.0;               ///< Max UE Tx power (dBm)
    static constexpr double ENB_MAX_TX_POWER_DBM = 46.0;              ///< Max eNB Tx power (dBm)
};

/**
 * \ingroup nbiot
 * \brief NPDCCH repetition values as per 3GPP TS 36.213 Table 16.6-1
 */
static const std::array<uint16_t, 12> NPDCCH_REPETITIONS = {
    1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048
};

/**
 * \ingroup nbiot
 * \brief NPDSCH repetition values as per 3GPP TS 36.213 Table 16.4.1.3-1
 */
static const std::array<uint8_t, 17> NPDSCH_REPETITIONS = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 20, 24, 28, 32
};

/**
 * \ingroup nbiot
 * \brief NPUSCH Format 1 repetition values as per 3GPP TS 36.213
 */
static const std::array<uint8_t, 8> NPUSCH_FORMAT1_REPETITIONS = {
    1, 2, 4, 8, 16, 32, 64, 128
};

/**
 * \ingroup nbiot
 * \brief NPRACH repetition values per coverage class as per 3GPP TS 36.211
 */
static const std::array<uint8_t, 8> NPRACH_REPETITIONS = {
    1, 2, 4, 8, 16, 32, 64, 128
};

/**
 * \ingroup nbiot
 * \brief Transport Block Size (TBS) table for NPDSCH as per 3GPP TS 36.213 Table 16.4.1.5.1-1
 */
static const std::array<uint16_t, 14> NPDSCH_TBS_TABLE = {
    16, 32, 56, 88, 120, 152, 184, 208, 256, 328, 408, 504, 584, 680
};

/**
 * \ingroup nbiot
 * \brief Number of uplink subcarriers for different NPUSCH configurations
 */
static const std::array<uint8_t, 4> NPUSCH_SUBCARRIER_OPTIONS = {
    1, 3, 6, 12
};

/**
 * \ingroup nbiot
 * \brief eDRX cycle lengths in seconds as per 3GPP TS 36.321
 */
static const std::array<double, 6> EDRX_CYCLE_LENGTHS_S = {
    5.12, 10.24, 20.48, 40.96, 81.92, 163.84
};

/**
 * \ingroup nbiot
 * \brief Paging cycle lengths in radio frames as per 3GPP TS 36.331
 */
static const std::array<uint16_t, 3> PAGING_CYCLES_RF = {
    256, 512, 1024  // 2.56s, 5.12s, 10.24s
};

/**
 * \ingroup nbiot
 * \brief Structure representing an NB-IoT resource element
 */
struct NbIotResourceElement
{
    uint8_t subcarrierIndex;   ///< Subcarrier index (0-11)
    uint8_t symbolIndex;       ///< OFDM symbol index (0-13)
    uint16_t subframeIndex;    ///< Subframe index within hyper-frame
    bool isOccupied;           ///< Whether RE is occupied
    NbIotPhysicalChannel channelType; ///< Channel occupying this RE
};

/**
 * \ingroup nbiot
 * \brief Structure representing an NB-IoT resource unit
 *
 * A resource unit (RU) is the basic unit for NPUSCH allocation.
 * The number of subcarriers and slots depends on the configuration.
 */
struct NbIotResourceUnit
{
    uint8_t numSubcarriers;    ///< Number of subcarriers (1, 3, 6, or 12)
    uint8_t numSlots;          ///< Number of slots in the RU
    uint16_t startSubframe;    ///< Starting subframe number
    uint8_t startSubcarrier;   ///< Starting subcarrier index
};

/**
 * \ingroup nbiot
 * \brief Structure for HARQ process information
 */
struct NbIotHarqProcess
{
    uint8_t processId;         ///< HARQ process ID (0-7)
    bool isActive;             ///< Whether process is active
    uint8_t txCount;           ///< Number of transmissions
    uint16_t tbs;              ///< Transport block size (bits)
    uint8_t repetitions;       ///< Number of repetitions
    bool ndi;                  ///< New Data Indicator
    Time lastTxTime;           ///< Time of last transmission
};

/**
 * \ingroup nbiot
 * \brief Structure for DCI (Downlink Control Information) messages
 */
struct NbIotDci
{
    NbIotDciFormat format;     ///< DCI format (N0, N1, N2)
    uint16_t rnti;             ///< Radio Network Temporary Identifier
    
    // Common fields
    uint8_t ndi;               ///< New Data Indicator
    uint8_t harqProcessNum;    ///< HARQ process number
    uint8_t mcs;               ///< Modulation and Coding Scheme
    uint8_t repetitionNumber;  ///< Repetition number index
    uint8_t resourceAssignment; ///< Resource assignment
    
    // Format N0 specific (uplink grant)
    uint8_t subcarrierIndication; ///< Subcarrier indication
    uint8_t schedulingDelay;   ///< Scheduling delay
    
    // Format N1 specific (downlink assignment)
    uint8_t sfAssignment;      ///< Subframe assignment
    
    // Format N2 specific (paging)
    bool directIndication;     ///< Direct indication flag
};

/**
 * \ingroup nbiot
 * \brief Structure for MIB-NB (Master Information Block)
 */
struct NbIotMib
{
    uint8_t systemFrameNumber;     ///< System Frame Number (4 MSBs)
    uint8_t hyperFrameNumber;      ///< Hyper Frame Number
    uint8_t schedulingInfoSib1;    ///< SIB1-NB scheduling info
    bool operationModeInfo;        ///< Operation mode information
    uint8_t spare;                 ///< Spare bits
    NbIotDeploymentMode deploymentMode; ///< Deployment mode
};

/**
 * \ingroup nbiot
 * \brief Structure for channel quality measurements
 */
struct NbIotChannelQuality
{
    double rsrp;               ///< Reference Signal Received Power (dBm)
    double rsrq;               ///< Reference Signal Received Quality (dB)
    double sinr;               ///< Signal to Interference plus Noise Ratio (dB)
    double snr;                ///< Signal to Noise Ratio (dB)
    NbIotCoverageClass estimatedCoverageClass; ///< Estimated coverage class
};

/**
 * \ingroup nbiot
 * \brief Structure for power saving mode configuration
 */
struct NbIotPsmConfig
{
    Time t3324;                ///< Active timer (after data transfer)
    Time t3412;                ///< Periodic TAU timer
    bool enabled;              ///< Whether PSM is enabled
};

/**
 * \ingroup nbiot
 * \brief Structure for extended DRX configuration
 */
struct NbIotEdrxConfig
{
    double cycleLengthSeconds; ///< eDRX cycle length
    double pagingTimeWindowSeconds; ///< Paging time window
    bool enabled;              ///< Whether eDRX is enabled
};

/**
 * \ingroup nbiot
 * \brief Structure for random access parameters
 */
struct NbIotRachConfig
{
    uint8_t preambleRepetitions; ///< NPRACH preamble repetitions
    NbIotNprachFormat preambleFormat; ///< Preamble format
    uint16_t rarWindow;        ///< RAR window size (subframes)
    uint8_t maxPreambleTx;     ///< Max preamble transmissions
    uint8_t backoffIndicator;  ///< Backoff indicator
    std::array<uint8_t, 3> coverageClassThresholds; ///< RSRP thresholds for CE levels
};

/**
 * \ingroup nbiot
 * \brief Convert deployment mode enum to string
 * \param mode The deployment mode
 * \return String representation
 */
inline std::string DeploymentModeToString(NbIotDeploymentMode mode)
{
    switch (mode)
    {
        case NbIotDeploymentMode::STANDALONE: return "Standalone";
        case NbIotDeploymentMode::GUARD_BAND: return "Guard-Band";
        case NbIotDeploymentMode::IN_BAND: return "In-Band";
        default: return "Unknown";
    }
}

/**
 * \ingroup nbiot
 * \brief Convert coverage class enum to string
 * \param ceLevel The coverage enhancement level
 * \return String representation
 */
inline std::string CoverageClassToString(NbIotCoverageClass ceLevel)
{
    switch (ceLevel)
    {
        case NbIotCoverageClass::CE_LEVEL_0: return "CE-Level-0 (Normal)";
        case NbIotCoverageClass::CE_LEVEL_1: return "CE-Level-1 (Medium)";
        case NbIotCoverageClass::CE_LEVEL_2: return "CE-Level-2 (Extreme)";
        default: return "Unknown";
    }
}

/**
 * \ingroup nbiot
 * \brief Get the maximum number of repetitions for a coverage class
 * \param ceLevel The coverage enhancement level
 * \param channel The physical channel
 * \return Maximum repetitions
 */
inline uint16_t GetMaxRepetitions(NbIotCoverageClass ceLevel, NbIotPhysicalChannel channel)
{
    switch (ceLevel)
    {
        case NbIotCoverageClass::CE_LEVEL_0:
            return (channel == NbIotPhysicalChannel::NPDCCH) ? 64 : 8;
        case NbIotCoverageClass::CE_LEVEL_1:
            return (channel == NbIotPhysicalChannel::NPDCCH) ? 256 : 32;
        case NbIotCoverageClass::CE_LEVEL_2:
            return (channel == NbIotPhysicalChannel::NPDCCH) ? 2048 : 128;
        default:
            return 1;
    }
}

/**
 * \ingroup nbiot
 * \brief Calculate TBS index from MCS for NB-IoT
 * \param mcs Modulation and Coding Scheme index
 * \return TBS index
 *
 * \see 3GPP TS 36.213 Table 16.4.1.5.1-1
 */
inline uint8_t GetTbsIndexFromMcs(uint8_t mcs)
{
    // Simplified mapping for NB-IoT (always QPSK)
    return (mcs < NPDSCH_TBS_TABLE.size()) ? mcs : static_cast<uint8_t>(NPDSCH_TBS_TABLE.size() - 1);
}

/**
 * \ingroup nbiot
 * \brief Get TBS (bits) from TBS index
 * \param tbsIndex TBS index
 * \return Transport block size in bits
 */
inline uint16_t GetTbsFromIndex(uint8_t tbsIndex)
{
    if (tbsIndex < NPDSCH_TBS_TABLE.size())
    {
        return NPDSCH_TBS_TABLE[tbsIndex];
    }
    return NPDSCH_TBS_TABLE.back();
}

/**
 * \ingroup nbiot
 * \brief Get TBS (bits) from MCS and number of resource blocks
 * \param mcs Modulation and coding scheme index
 * \param nRbs Number of resource blocks (typically 1 for NB-IoT)
 * \return Transport block size in bits
 */
inline uint16_t GetTbsFromMcsAndRbs(uint8_t mcs, [[maybe_unused]] uint8_t nRbs)
{
    // For NB-IoT, MCS directly maps to TBS index (simplified)
    // In full implementation, this would use 3GPP TS 36.213 tables
    uint8_t tbsIndex = (mcs < NPDSCH_TBS_TABLE.size()) ? mcs : static_cast<uint8_t>(NPDSCH_TBS_TABLE.size() - 1);
    return NPDSCH_TBS_TABLE[tbsIndex];
}

/**
 * \brief Stream insertion operator for NbIotCoverageClass
 * \param os Output stream
 * \param cc Coverage class
 * \return Output stream
 */
inline std::ostream&
operator<<(std::ostream& os, NbIotCoverageClass cc)
{
    switch (cc)
    {
    case NbIotCoverageClass::CE_LEVEL_0:
        os << "CE_LEVEL_0";
        break;
    case NbIotCoverageClass::CE_LEVEL_1:
        os << "CE_LEVEL_1";
        break;
    case NbIotCoverageClass::CE_LEVEL_2:
        os << "CE_LEVEL_2";
        break;
    default:
        os << "UNKNOWN";
    }
    return os;
}

/**
 * \brief Stream insertion operator for NbIotDeploymentMode
 * \param os Output stream
 * \param mode Deployment mode
 * \return Output stream
 */
inline std::ostream&
operator<<(std::ostream& os, NbIotDeploymentMode mode)
{
    switch (mode)
    {
    case NbIotDeploymentMode::STANDALONE:
        os << "STANDALONE";
        break;
    case NbIotDeploymentMode::GUARD_BAND:
        os << "GUARD_BAND";
        break;
    case NbIotDeploymentMode::IN_BAND:
        os << "IN_BAND";
        break;
    default:
        os << "UNKNOWN";
    }
    return os;
}

/**
 * \brief Stream insertion operator for NbIotRrcState
 * \param os Output stream
 * \param state RRC state
 * \return Output stream
 */
inline std::ostream&
operator<<(std::ostream& os, NbIotRrcState state)
{
    switch (state)
    {
    case NbIotRrcState::RRC_IDLE:
        os << "RRC_IDLE";
        break;
    case NbIotRrcState::RRC_CONNECTED:
        os << "RRC_CONNECTED";
        break;
    case NbIotRrcState::RRC_INACTIVE:
        os << "RRC_INACTIVE";
        break;
    default:
        os << "UNKNOWN";
    }
    return os;
}

/**
 * \brief Stream insertion operator for NbIotPhysicalChannel
 * \param os Output stream
 * \param channel Physical channel
 * \return Output stream
 */
inline std::ostream&
operator<<(std::ostream& os, NbIotPhysicalChannel channel)
{
    switch (channel)
    {
    case NbIotPhysicalChannel::NPBCH:
        os << "NPBCH";
        break;
    case NbIotPhysicalChannel::NPDCCH:
        os << "NPDCCH";
        break;
    case NbIotPhysicalChannel::NPDSCH:
        os << "NPDSCH";
        break;
    case NbIotPhysicalChannel::NPUSCH:
        os << "NPUSCH";
        break;
    case NbIotPhysicalChannel::NPRACH:
        os << "NPRACH";
        break;
    default:
        os << "UNKNOWN";
    }
    return os;
}

} // namespace ns3

#endif /* NBIOT_COMMON_H */
