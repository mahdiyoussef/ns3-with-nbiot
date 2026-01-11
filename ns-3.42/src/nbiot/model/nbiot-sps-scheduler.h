/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Semi-Persistent Scheduling (SPS) Implementation
 *
 * This scheduler implements Semi-Persistent Scheduling as described in:
 * "Latency Reduction for Narrowband LTE with Semi-Persistent Scheduling"
 * by Zubair Amjad et al., IEEE IDAACS 2018
 *
 * Key features:
 * - Pre-allocates periodic uplink resources to UEs
 * - Eliminates SR/SG control overhead (saves ~7ms)
 * - Suitable for periodic MTC/URLLC traffic
 */

#ifndef NBIOT_SPS_SCHEDULER_H
#define NBIOT_SPS_SCHEDULER_H

#include "nbiot-enb-mac.h"
#include "nbiot-common.h"

#include <ns3/object.h>
#include <ns3/nstime.h>
#include <ns3/event-id.h>

#include <map>
#include <vector>
#include <set>

namespace ns3 {

/**
 * \ingroup nbiot
 * \brief SPS configuration for a UE
 */
struct NbIotSpsConfig
{
    uint16_t rnti;                  ///< UE identifier
    Time spsInterval;               ///< Periodic transmission interval
    uint8_t numSubcarriers;         ///< Allocated subcarriers (1, 3, 6, or 12)
    uint8_t mcs;                    ///< Modulation and coding scheme
    uint16_t tbSize;                ///< Transport block size
    uint8_t repetitions;            ///< Number of repetitions
    Time nextAllocation;            ///< Time of next allocation
    bool active;                    ///< Whether SPS is active for this UE
    uint8_t harqProcess;            ///< HARQ process for SPS
    
    NbIotSpsConfig()
        : rnti(0)
        , spsInterval(MilliSeconds(20))
        , numSubcarriers(12)
        , mcs(4)
        , tbSize(88)
        , repetitions(1)
        , nextAllocation(Seconds(0))
        , active(false)
        , harqProcess(0)
    {
    }
};

/**
 * \ingroup nbiot
 * \brief Semi-Persistent Scheduler for NB-IoT
 *
 * Implements SPS for uplink transmission:
 * - Allocates periodic resources without SR/SG exchange
 * - Pre-configured transmission opportunities
 * - Suitable for periodic traffic patterns
 * - Reduces latency by eliminating control signaling overhead
 *
 * The scheduler operates in two modes:
 * 1. SPS mode: Pre-allocated periodic grants (no SR needed)
 * 2. Dynamic mode: Falls back to SR-based scheduling for non-SPS UEs
 */
class NbIotSpsScheduler : public NbIotScheduler
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
    NbIotSpsScheduler();
    
    /**
     * \brief Destructor
     */
    ~NbIotSpsScheduler() override;
    
    // Inherited from NbIotScheduler
    std::vector<NbIotDlAssignmentResult> ScheduleDl(uint8_t availableRbs) override;
    std::vector<NbIotUlGrantResult> ScheduleUl(uint8_t availableRbs) override;
    
    /**
     * \brief Configure SPS for a UE
     * \param rnti UE identifier
     * \param interval Periodic transmission interval
     * \param numSubcarriers Number of subcarriers to allocate
     */
    void ConfigureSps(uint16_t rnti, Time interval, uint8_t numSubcarriers = 12);
    
    /**
     * \brief Activate SPS for a UE
     * \param rnti UE identifier
     */
    void ActivateSps(uint16_t rnti);
    
    /**
     * \brief Deactivate SPS for a UE
     * \param rnti UE identifier
     */
    void DeactivateSps(uint16_t rnti);
    
    /**
     * \brief Check if SPS is active for a UE
     * \param rnti UE identifier
     * \return True if SPS is active
     */
    bool IsSpsActive(uint16_t rnti) const;
    
    /**
     * \brief Get SPS configuration for a UE
     * \param rnti UE identifier
     * \return Pointer to SPS configuration or nullptr
     */
    const NbIotSpsConfig* GetSpsConfig(uint16_t rnti) const;
    
    /**
     * \brief Set the SPS interval for new UEs
     * \param interval Default SPS interval
     */
    void SetDefaultSpsInterval(Time interval);
    
    /**
     * \brief Get the default SPS interval
     * \return Default SPS interval
     */
    Time GetDefaultSpsInterval() const;
    
    /**
     * \brief Enable/disable automatic SPS activation for new UEs
     * \param enable True to auto-activate SPS for new connected UEs
     */
    void SetAutoActivateSps(bool enable);
    
    /**
     * \brief Get number of SPS-active UEs
     * \return Count of UEs with active SPS
     */
    uint32_t GetActiveSpsCount() const;
    
    /**
     * \brief Get statistics on SPS performance
     * \param paddingTransmissions Output: Number of padding transmissions
     * \param dataTransmissions Output: Number of data transmissions
     */
    void GetSpsStatistics(uint32_t& paddingTransmissions, uint32_t& dataTransmissions) const;
    
protected:
    void DoDispose() override;
    
private:
    /**
     * \brief Schedule SPS grants for current subframe
     * \param currentTime Current simulation time
     * \param availableRbs Available resource blocks
     * \return Vector of UL grants for SPS UEs
     */
    std::vector<NbIotUlGrantResult> ScheduleSpsGrants(Time currentTime, uint8_t availableRbs);
    
    /**
     * \brief Schedule dynamic grants for non-SPS UEs
     * \param availableRbs Available resource blocks after SPS allocation
     * \return Vector of UL grants for dynamic UEs
     */
    std::vector<NbIotUlGrantResult> ScheduleDynamicGrants(uint8_t availableRbs);
    
    /**
     * \brief Calculate MCS and TBS for given channel conditions
     * \param cqi Channel quality indicator
     * \param numSubcarriers Number of subcarriers
     * \param mcs Output: Selected MCS
     * \param tbSize Output: Transport block size
     */
    void SelectMcsAndTbs(uint8_t cqi, uint8_t numSubcarriers, 
                         uint8_t& mcs, uint16_t& tbSize);
    
    /**
     * \brief Update SPS allocation time for next period
     * \param rnti UE identifier
     */
    void UpdateNextAllocation(uint16_t rnti);
    
    /// SPS configurations per UE
    std::map<uint16_t, NbIotSpsConfig> m_spsConfigs;
    
    /// Default SPS interval for new UEs
    Time m_defaultSpsInterval;
    
    /// Auto-activate SPS for new UEs
    bool m_autoActivateSps;
    
    /// Statistics
    uint32_t m_paddingTransmissions;
    uint32_t m_dataTransmissions;
    
    /// Last scheduled RNTI for dynamic scheduling (round-robin fallback)
    uint16_t m_lastDynamicRnti;
    
    /// Current subframe number for tracking SPS periods
    uint64_t m_subframeCounter;
};

} // namespace ns3

#endif /* NBIOT_SPS_SCHEDULER_H */
