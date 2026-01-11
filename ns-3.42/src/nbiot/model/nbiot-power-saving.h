/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Power Saving Mechanisms Header
 *
 * References:
 * - 3GPP TS 23.682 Architecture enhancements for MTC
 * - 3GPP TS 24.008 Mobile radio interface Layer 3
 * - 3GPP TS 36.304 UE procedures in idle mode
 */

#ifndef NBIOT_POWER_SAVING_H
#define NBIOT_POWER_SAVING_H

#include "nbiot-common.h"

#include <ns3/object.h>
#include <ns3/ptr.h>
#include <ns3/traced-callback.h>
#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/callback.h>

namespace ns3 {

class NbIotUePhy;
class NbIotUeMac;
class NbIotUeRrc;

// NbIotPsmState is defined in nbiot-common.h with values: PSM_ACTIVE, PSM_LIGHT_SLEEP, PSM_DEEP_SLEEP

/**
 * \ingroup nbiot
 *
 * \brief Extended DRX state
 */
enum class NbIotEdrxState : uint8_t
{
    ACTIVE,             ///< UE is active
    SLEEP,              ///< UE is sleeping (between PTW)
    PAGING_WINDOW       ///< UE is in Paging Time Window (PTW)
};

/**
 * \ingroup nbiot
 *
 * \brief PSM timer values
 */
struct NbIotPsmTimers
{
    Time t3324;         ///< Active timer - time in idle before PSM
    Time t3412;         ///< Extended TAU timer - periodic TAU timer
    Time t3412ext;      ///< Extended T3412 for PSM
};

/**
 * \ingroup nbiot
 *
 * \brief eDRX parameters
 */
struct NbIotEdrxParams
{
    Time edrxCycle;     ///< eDRX cycle length (20.48s to 2621.44s)
    Time pagingTimeWindow; ///< PTW length (2.56s to 40.96s)
    uint8_t pagingHyperFrame;  ///< H-SFN for paging
};

/**
 * \ingroup nbiot
 *
 * \brief Power Saving Mode (PSM) manager for NB-IoT UE
 *
 * Implements PSM as specified in 3GPP TS 23.682 and TS 24.008.
 * PSM allows the UE to enter a very low power consumption state
 * for extended periods while remaining registered to the network.
 */
class NbIotPsmManager : public Object
{
public:
    static TypeId GetTypeId();
    
    NbIotPsmManager();
    ~NbIotPsmManager() override;
    
    /**
     * \brief Set PHY layer for power control
     * \param phy Pointer to UE PHY
     */
    void SetPhy(Ptr<NbIotUePhy> phy);
    
    /**
     * \brief Set MAC layer
     * \param mac Pointer to UE MAC
     */
    void SetMac(Ptr<NbIotUeMac> mac);
    
    /**
     * \brief Set RRC layer
     * \param rrc Pointer to UE RRC
     */
    void SetRrc(Ptr<NbIotUeRrc> rrc);
    
    /**
     * \brief Enable PSM
     * \param enable True to enable
     */
    void SetEnabled(bool enable);
    
    /**
     * \brief Check if PSM is enabled
     * \return True if enabled
     */
    bool IsEnabled() const;
    
    /**
     * \brief Configure PSM timers
     * \param timers Timer configuration
     */
    void SetTimers(const NbIotPsmTimers& timers);
    
    /**
     * \brief Get PSM timers
     * \return Current timer configuration
     */
    NbIotPsmTimers GetTimers() const;
    
    /**
     * \brief Get current PSM state
     * \return PSM state
     */
    NbIotPsmState GetState() const;
    
    /**
     * \brief Notify UE entered idle mode
     * Start T3324 (active timer)
     */
    void NotifyIdleMode();
    
    /**
     * \brief Notify UE activity (data transmission)
     * Reset T3324
     */
    void NotifyActivity();
    
    /**
     * \brief Notify TAU required
     * Called when T3412 expires
     */
    void NotifyTauRequired();
    
    /**
     * \brief Force exit from PSM
     * Called for mobile-originated communication
     */
    void ExitPsm();
    
    /**
     * \brief Get time spent in PSM
     * \return Duration in PSM state
     */
    Time GetPsmDuration() const;
    
    /**
     * \brief Get current power consumption (mW)
     * \return Estimated power consumption
     */
    double GetPowerConsumption() const;
    
    /**
     * \brief Callback for PSM state changes
     */
    typedef Callback<void, NbIotPsmState, NbIotPsmState> StateChangeCallback;
    
    /**
     * \brief Set state change callback
     * \param cb Callback function
     */
    void SetStateChangeCallback(StateChangeCallback cb);
    
protected:
    void DoDispose() override;
    
private:
    /**
     * \brief T3324 expiry handler
     * Enter PSM
     */
    void T3324Expiry();
    
    /**
     * \brief T3412 expiry handler
     * Perform TAU
     */
    void T3412Expiry();
    
    /**
     * \brief Switch to new PSM state
     * \param newState New state
     */
    void SwitchToState(NbIotPsmState newState);
    
    /**
     * \brief Enter power saving mode
     */
    void EnterPsm();
    
    /**
     * \brief Perform Tracking Area Update
     */
    void PerformTau();
    
    bool m_enabled;                         ///< PSM enabled flag
    NbIotPsmState m_state;                  ///< Current state
    NbIotPsmTimers m_timers;                ///< Timer configuration
    
    Ptr<NbIotUePhy> m_phy;
    Ptr<NbIotUeMac> m_mac;
    Ptr<NbIotUeRrc> m_rrc;
    
    EventId m_t3324Event;                   ///< Active timer event
    EventId m_t3412Event;                   ///< TAU timer event
    
    Time m_psmEntryTime;                    ///< Time when entered PSM
    Time m_totalPsmTime;                    ///< Total time in PSM
    
    StateChangeCallback m_stateChangeCallback;
    
    /// Traces
    TracedCallback<NbIotPsmState, NbIotPsmState> m_stateTransitionTrace;
    TracedCallback<Time> m_psmDurationTrace;
    
public:
    // Power consumption values (mW)
    static constexpr double POWER_ACTIVE = 500.0;
    static constexpr double POWER_IDLE = 50.0;
    static constexpr double POWER_PSM = 0.015;  // ~15 uW in deep sleep
};

/**
 * \ingroup nbiot
 *
 * \brief Extended DRX (eDRX) manager for NB-IoT UE
 *
 * Implements eDRX as specified in 3GPP TS 36.304.
 * eDRX extends the DRX cycle to reduce paging monitoring frequency
 * while maintaining some network reachability.
 */
class NbIotEdrxManager : public Object
{
public:
    static TypeId GetTypeId();
    
    NbIotEdrxManager();
    ~NbIotEdrxManager() override;
    
    /**
     * \brief Set PHY layer
     * \param phy Pointer to UE PHY
     */
    void SetPhy(Ptr<NbIotUePhy> phy);
    
    /**
     * \brief Set MAC layer
     * \param mac Pointer to UE MAC
     */
    void SetMac(Ptr<NbIotUeMac> mac);
    
    /**
     * \brief Enable eDRX
     * \param enable True to enable
     */
    void SetEnabled(bool enable);
    
    /**
     * \brief Check if eDRX is enabled
     * \return True if enabled
     */
    bool IsEnabled() const;
    
    /**
     * \brief Configure eDRX parameters
     * \param params eDRX configuration
     */
    void SetParams(const NbIotEdrxParams& params);
    
    /**
     * \brief Get eDRX parameters
     * \return Current eDRX configuration
     */
    NbIotEdrxParams GetParams() const;
    
    /**
     * \brief Get current eDRX state
     * \return eDRX state
     */
    NbIotEdrxState GetState() const;
    
    /**
     * \brief Start eDRX operation
     * Called when UE enters idle mode
     */
    void Start();
    
    /**
     * \brief Stop eDRX operation
     * Called when UE becomes active
     */
    void Stop();
    
    /**
     * \brief Check if currently in PTW
     * \return True if in Paging Time Window
     */
    bool IsInPagingWindow() const;
    
    /**
     * \brief Get time until next PTW
     * \return Time until next paging window
     */
    Time GetTimeToNextPtw() const;
    
    /**
     * \brief Get estimated battery life extension factor
     * \return Extension factor compared to legacy DRX
     */
    double GetBatteryLifeExtension() const;
    
    /**
     * \brief Callback for eDRX state changes
     */
    typedef Callback<void, NbIotEdrxState, NbIotEdrxState> StateChangeCallback;
    
    /**
     * \brief Set state change callback
     * \param cb Callback function
     */
    void SetStateChangeCallback(StateChangeCallback cb);
    
protected:
    void DoDispose() override;
    
private:
    /**
     * \brief Calculate next PTW start time
     * \return Time of next PTW
     */
    Time CalculateNextPtwStart() const;
    
    /**
     * \brief PTW start handler
     */
    void PtwStart();
    
    /**
     * \brief PTW end handler
     */
    void PtwEnd();
    
    /**
     * \brief Switch to new eDRX state
     * \param newState New state
     */
    void SwitchToState(NbIotEdrxState newState);
    
    bool m_enabled;                         ///< eDRX enabled flag
    NbIotEdrxState m_state;                 ///< Current state
    NbIotEdrxParams m_params;               ///< eDRX configuration
    bool m_running;                         ///< eDRX operation in progress
    
    Ptr<NbIotUePhy> m_phy;
    Ptr<NbIotUeMac> m_mac;
    
    EventId m_ptwStartEvent;                ///< PTW start event
    EventId m_ptwEndEvent;                  ///< PTW end event
    
    Time m_currentPtwStart;                 ///< Current PTW start time
    Time m_lastPtwEnd;                      ///< Last PTW end time
    
    StateChangeCallback m_stateChangeCallback;
    
    /// Traces
    TracedCallback<NbIotEdrxState, NbIotEdrxState> m_stateTransitionTrace;
    TracedCallback<Time, Time> m_ptwTrace;  ///< PTW start, duration
    
public:
    // Power consumption values (mW)
    static constexpr double POWER_ACTIVE = 500.0;
    static constexpr double POWER_PTW = 100.0;  // During paging window
    static constexpr double POWER_SLEEP = 1.0;   // Between PTWs
};

/**
 * \ingroup nbiot
 *
 * \brief Combined power saving controller
 *
 * Manages both PSM and eDRX to optimize power consumption
 * based on application requirements and network configuration.
 */
class NbIotPowerSavingController : public Object
{
public:
    static TypeId GetTypeId();
    
    NbIotPowerSavingController();
    ~NbIotPowerSavingController() override;
    
    /**
     * \brief Set PSM manager
     * \param psm PSM manager
     */
    void SetPsmManager(Ptr<NbIotPsmManager> psm);
    
    /**
     * \brief Set eDRX manager
     * \param edrx eDRX manager
     */
    void SetEdrxManager(Ptr<NbIotEdrxManager> edrx);
    
    /**
     * \brief Get PSM manager
     * \return PSM manager
     */
    Ptr<NbIotPsmManager> GetPsmManager() const;
    
    /**
     * \brief Get eDRX manager
     * \return eDRX manager
     */
    Ptr<NbIotEdrxManager> GetEdrxManager() const;
    
    /**
     * \brief Enable power saving features
     * \param enablePsm Enable PSM
     * \param enableEdrx Enable eDRX
     */
    void Enable(bool enablePsm, bool enableEdrx);
    
    /**
     * \brief Notify UE state change (idle/connected)
     * \param connected True if RRC connected
     */
    void NotifyConnectionState(bool connected);
    
    /**
     * \brief Notify data activity
     */
    void NotifyDataActivity();
    
    /**
     * \brief Get estimated battery life
     * \param batteryCapacity Battery capacity in mAh
     * \param voltage Nominal voltage (V)
     * \return Estimated battery life in hours
     */
    double GetEstimatedBatteryLife(double batteryCapacity, double voltage) const;
    
    /**
     * \brief Get average power consumption
     * \return Average power in mW
     */
    double GetAveragePower() const;
    
protected:
    void DoDispose() override;
    
private:
    Ptr<NbIotPsmManager> m_psm;
    Ptr<NbIotEdrxManager> m_edrx;
    
    bool m_connected;                       ///< RRC connection state
    Time m_lastActivity;                    ///< Time of last data activity
    
    // Statistics
    Time m_activeTime;
    Time m_idleTime;
    Time m_psmTime;
    Time m_edrxSleepTime;
    
    /// Traces
    TracedCallback<double> m_powerTrace;    ///< Current power consumption
};

/**
 * \ingroup nbiot
 *
 * \brief Convert PSM state to string
 * \param state PSM state
 * \return String representation
 */
inline std::string PsmStateToString(NbIotPsmState state)
{
    switch (state)
    {
        case NbIotPsmState::PSM_ACTIVE: return "ACTIVE";
        case NbIotPsmState::PSM_LIGHT_SLEEP: return "IDLE_MONITORING";
        case NbIotPsmState::PSM_DEEP_SLEEP: return "POWER_SAVING";
        default: return "Unknown";
    }
}

/**
 * \ingroup nbiot
 *
 * \brief Convert eDRX state to string
 * \param state eDRX state
 * \return String representation
 */
inline std::string EdrxStateToString(NbIotEdrxState state)
{
    switch (state)
    {
        case NbIotEdrxState::ACTIVE: return "ACTIVE";
        case NbIotEdrxState::SLEEP: return "SLEEP";
        case NbIotEdrxState::PAGING_WINDOW: return "PAGING_WINDOW";
        default: return "Unknown";
    }
}

} // namespace ns3

#endif /* NBIOT_POWER_SAVING_H */
