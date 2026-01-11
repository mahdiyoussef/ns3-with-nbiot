/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Power Saving Mechanisms implementation
 */

#include "nbiot-power-saving.h"
#include "nbiot-ue-phy.h"
#include "nbiot-ue-mac.h"
#include "nbiot-rrc.h"

#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/double.h>
#include <ns3/boolean.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NbIotPowerSaving");

// ====================== NbIotPsmManager ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotPsmManager);

TypeId
NbIotPsmManager::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotPsmManager")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotPsmManager>()
        .AddAttribute("Enabled",
                      "Enable PSM",
                      BooleanValue(true),
                      MakeBooleanAccessor(&NbIotPsmManager::m_enabled),
                      MakeBooleanChecker())
        .AddTraceSource("StateTransition",
                        "PSM state transition",
                        MakeTraceSourceAccessor(&NbIotPsmManager::m_stateTransitionTrace),
                        "ns3::NbIotPsmManager::StateTransitionTracedCallback")
        .AddTraceSource("PsmDuration",
                        "PSM duration trace",
                        MakeTraceSourceAccessor(&NbIotPsmManager::m_psmDurationTrace),
                        "ns3::NbIotPsmManager::PsmDurationTracedCallback");
    return tid;
}

NbIotPsmManager::NbIotPsmManager()
    : m_enabled(true)
    , m_state(NbIotPsmState::PSM_ACTIVE)
    , m_totalPsmTime(Seconds(0))
{
    NS_LOG_FUNCTION(this);
    
    // Default timer values per 3GPP TS 24.008
    m_timers.t3324 = Seconds(10);       // Active timer (2s to 31 min)
    m_timers.t3412 = Hours(1);          // TAU timer (default 1 hour)
    m_timers.t3412ext = Hours(310);     // Extended TAU (up to 310 hours)
}

NbIotPsmManager::~NbIotPsmManager()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotPsmManager::DoDispose()
{
    m_phy = nullptr;
    m_mac = nullptr;
    m_rrc = nullptr;
    
    if (m_t3324Event.IsPending())
    {
        Simulator::Cancel(m_t3324Event);
    }
    if (m_t3412Event.IsPending())
    {
        Simulator::Cancel(m_t3412Event);
    }
    
    Object::DoDispose();
}

void NbIotPsmManager::SetPhy(Ptr<NbIotUePhy> phy) { m_phy = phy; }
void NbIotPsmManager::SetMac(Ptr<NbIotUeMac> mac) { m_mac = mac; }
void NbIotPsmManager::SetRrc(Ptr<NbIotUeRrc> rrc) { m_rrc = rrc; }
void NbIotPsmManager::SetEnabled(bool enable) { m_enabled = enable; }
bool NbIotPsmManager::IsEnabled() const { return m_enabled; }
void NbIotPsmManager::SetTimers(const NbIotPsmTimers& timers) { m_timers = timers; }
NbIotPsmTimers NbIotPsmManager::GetTimers() const { return m_timers; }
NbIotPsmState NbIotPsmManager::GetState() const { return m_state; }

void
NbIotPsmManager::SetStateChangeCallback(StateChangeCallback cb)
{
    m_stateChangeCallback = cb;
}

void
NbIotPsmManager::NotifyIdleMode()
{
    NS_LOG_FUNCTION(this);
    
    if (!m_enabled)
    {
        return;
    }
    
    SwitchToState(NbIotPsmState::PSM_LIGHT_SLEEP);
    
    // Start T3324 (active timer)
    if (m_t3324Event.IsPending())
    {
        Simulator::Cancel(m_t3324Event);
    }
    
    m_t3324Event = Simulator::Schedule(m_timers.t3324,
                                        &NbIotPsmManager::T3324Expiry,
                                        this);
    
    NS_LOG_INFO("PSM: Started T3324 (" << m_timers.t3324.As(Time::S) << ")");
}

void
NbIotPsmManager::NotifyActivity()
{
    NS_LOG_FUNCTION(this);
    
    if (m_state == NbIotPsmState::PSM_DEEP_SLEEP)
    {
        ExitPsm();
    }
    else if (m_state == NbIotPsmState::PSM_LIGHT_SLEEP)
    {
        // Reset T3324
        if (m_t3324Event.IsPending())
        {
            Simulator::Cancel(m_t3324Event);
        }
        m_t3324Event = Simulator::Schedule(m_timers.t3324,
                                            &NbIotPsmManager::T3324Expiry,
                                            this);
    }
    
    SwitchToState(NbIotPsmState::PSM_ACTIVE);
}

void
NbIotPsmManager::NotifyTauRequired()
{
    NS_LOG_FUNCTION(this);
    
    // Exit PSM to perform TAU
    ExitPsm();
    PerformTau();
}

void
NbIotPsmManager::ExitPsm()
{
    NS_LOG_FUNCTION(this);
    
    if (m_state != NbIotPsmState::PSM_DEEP_SLEEP)
    {
        return;
    }
    
    // Calculate time spent in PSM
    Time psmDuration = Simulator::Now() - m_psmEntryTime;
    m_totalPsmTime += psmDuration;
    
    m_psmDurationTrace(psmDuration);
    
    NS_LOG_INFO("PSM: Exiting after " << psmDuration.As(Time::S));
    
    // Wake up PHY
    if (m_phy)
    {
        // In real implementation, would power up radio
    }
    
    SwitchToState(NbIotPsmState::PSM_ACTIVE);
}

Time
NbIotPsmManager::GetPsmDuration() const
{
    if (m_state == NbIotPsmState::PSM_DEEP_SLEEP)
    {
        return m_totalPsmTime + (Simulator::Now() - m_psmEntryTime);
    }
    return m_totalPsmTime;
}

double
NbIotPsmManager::GetPowerConsumption() const
{
    switch (m_state)
    {
        case NbIotPsmState::PSM_ACTIVE:
            return POWER_ACTIVE;
        case NbIotPsmState::PSM_LIGHT_SLEEP:
            return POWER_IDLE;
        case NbIotPsmState::PSM_DEEP_SLEEP:
            return POWER_PSM;
        default:
            return POWER_ACTIVE;
    }
}

void
NbIotPsmManager::T3324Expiry()
{
    NS_LOG_FUNCTION(this);
    
    NS_LOG_INFO("PSM: T3324 expired, entering power saving mode");
    
    EnterPsm();
}

void
NbIotPsmManager::T3412Expiry()
{
    NS_LOG_FUNCTION(this);
    
    NS_LOG_INFO("PSM: T3412 expired, TAU required");
    
    NotifyTauRequired();
}

void
NbIotPsmManager::SwitchToState(NbIotPsmState newState)
{
    if (m_state == newState)
    {
        return;
    }
    
    NbIotPsmState oldState = m_state;
    m_state = newState;
    
    m_stateTransitionTrace(oldState, newState);
    
    if (!m_stateChangeCallback.IsNull())
    {
        m_stateChangeCallback(oldState, newState);
    }
    
    NS_LOG_INFO("PSM state: " << PsmStateToString(oldState)
                << " -> " << PsmStateToString(newState));
}

void
NbIotPsmManager::EnterPsm()
{
    NS_LOG_FUNCTION(this);
    
    m_psmEntryTime = Simulator::Now();
    
    // Power down PHY
    if (m_phy)
    {
        // In real implementation, would power down radio
    }
    
    SwitchToState(NbIotPsmState::PSM_DEEP_SLEEP);
    
    // Start T3412 for periodic TAU
    if (m_t3412Event.IsPending())
    {
        Simulator::Cancel(m_t3412Event);
    }
    
    Time tauTimer = m_timers.t3412ext > Seconds(0) ? m_timers.t3412ext : m_timers.t3412;
    m_t3412Event = Simulator::Schedule(tauTimer,
                                        &NbIotPsmManager::T3412Expiry,
                                        this);
    
    NS_LOG_INFO("PSM: Entered power saving mode, T3412=" << tauTimer.As(Time::H));
}

void
NbIotPsmManager::PerformTau()
{
    NS_LOG_FUNCTION(this);
    
    NS_LOG_INFO("PSM: Performing Tracking Area Update");
    
    // In real implementation, would trigger RRC connection and TAU procedure
    if (m_rrc)
    {
        m_rrc->Connect(NbIotRrcEstablishmentCause::MO_SIGNALLING);
    }
}

// ====================== NbIotEdrxManager ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotEdrxManager);

TypeId
NbIotEdrxManager::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotEdrxManager")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotEdrxManager>()
        .AddAttribute("Enabled",
                      "Enable eDRX",
                      BooleanValue(true),
                      MakeBooleanAccessor(&NbIotEdrxManager::m_enabled),
                      MakeBooleanChecker())
        .AddTraceSource("StateTransition",
                        "eDRX state transition",
                        MakeTraceSourceAccessor(&NbIotEdrxManager::m_stateTransitionTrace),
                        "ns3::NbIotEdrxManager::StateTransitionTracedCallback")
        .AddTraceSource("Ptw",
                        "Paging Time Window trace",
                        MakeTraceSourceAccessor(&NbIotEdrxManager::m_ptwTrace),
                        "ns3::NbIotEdrxManager::PtwTracedCallback");
    return tid;
}

NbIotEdrxManager::NbIotEdrxManager()
    : m_enabled(true)
    , m_state(NbIotEdrxState::ACTIVE)
    , m_running(false)
{
    NS_LOG_FUNCTION(this);
    
    // Default eDRX parameters for NB-IoT
    m_params.edrxCycle = Seconds(81.92);        // Default cycle
    m_params.pagingTimeWindow = Seconds(2.56);   // Minimum PTW
    m_params.pagingHyperFrame = 0;
}

NbIotEdrxManager::~NbIotEdrxManager()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotEdrxManager::DoDispose()
{
    m_phy = nullptr;
    m_mac = nullptr;
    
    if (m_ptwStartEvent.IsPending())
    {
        Simulator::Cancel(m_ptwStartEvent);
    }
    if (m_ptwEndEvent.IsPending())
    {
        Simulator::Cancel(m_ptwEndEvent);
    }
    
    Object::DoDispose();
}

void NbIotEdrxManager::SetPhy(Ptr<NbIotUePhy> phy) { m_phy = phy; }
void NbIotEdrxManager::SetMac(Ptr<NbIotUeMac> mac) { m_mac = mac; }
void NbIotEdrxManager::SetEnabled(bool enable) { m_enabled = enable; }
bool NbIotEdrxManager::IsEnabled() const { return m_enabled; }
void NbIotEdrxManager::SetParams(const NbIotEdrxParams& params) { m_params = params; }
NbIotEdrxParams NbIotEdrxManager::GetParams() const { return m_params; }
NbIotEdrxState NbIotEdrxManager::GetState() const { return m_state; }

void
NbIotEdrxManager::SetStateChangeCallback(StateChangeCallback cb)
{
    m_stateChangeCallback = cb;
}

void
NbIotEdrxManager::Start()
{
    NS_LOG_FUNCTION(this);
    
    if (!m_enabled || m_running)
    {
        return;
    }
    
    m_running = true;
    
    // Schedule first PTW
    Time nextPtwStart = CalculateNextPtwStart();
    
    if (nextPtwStart > Seconds(0))
    {
        SwitchToState(NbIotEdrxState::SLEEP);
        m_ptwStartEvent = Simulator::Schedule(nextPtwStart,
                                               &NbIotEdrxManager::PtwStart,
                                               this);
        
        NS_LOG_INFO("eDRX: Started, next PTW in " << nextPtwStart.As(Time::S));
    }
    else
    {
        PtwStart();
    }
}

void
NbIotEdrxManager::Stop()
{
    NS_LOG_FUNCTION(this);
    
    m_running = false;
    
    if (m_ptwStartEvent.IsPending())
    {
        Simulator::Cancel(m_ptwStartEvent);
    }
    if (m_ptwEndEvent.IsPending())
    {
        Simulator::Cancel(m_ptwEndEvent);
    }
    
    SwitchToState(NbIotEdrxState::ACTIVE);
    
    NS_LOG_INFO("eDRX: Stopped");
}

bool
NbIotEdrxManager::IsInPagingWindow() const
{
    return m_state == NbIotEdrxState::PAGING_WINDOW;
}

Time
NbIotEdrxManager::GetTimeToNextPtw() const
{
    if (m_state == NbIotEdrxState::PAGING_WINDOW)
    {
        return Seconds(0);
    }
    
    return CalculateNextPtwStart();
}

double
NbIotEdrxManager::GetBatteryLifeExtension() const
{
    // Calculate extension factor compared to legacy DRX (1.28s)
    Time legacyDrx = Seconds(1.28);
    return m_params.edrxCycle.GetSeconds() / legacyDrx.GetSeconds();
}

Time
NbIotEdrxManager::CalculateNextPtwStart() const
{
    // Simplified calculation based on H-SFN
    // In real implementation, would use precise H-SFN calculation
    Time now = Simulator::Now();
    Time cycleStart = Seconds(std::floor(now.GetSeconds() / m_params.edrxCycle.GetSeconds())
                              * m_params.edrxCycle.GetSeconds());
    Time nextCycleStart = cycleStart + m_params.edrxCycle;
    
    return nextCycleStart - now;
}

void
NbIotEdrxManager::PtwStart()
{
    NS_LOG_FUNCTION(this);
    
    m_currentPtwStart = Simulator::Now();
    
    SwitchToState(NbIotEdrxState::PAGING_WINDOW);
    
    // Wake up PHY for paging monitoring
    if (m_phy)
    {
        // In real implementation, would power up receiver
    }
    
    // Schedule PTW end
    m_ptwEndEvent = Simulator::Schedule(m_params.pagingTimeWindow,
                                         &NbIotEdrxManager::PtwEnd,
                                         this);
    
    m_ptwTrace(m_currentPtwStart, m_params.pagingTimeWindow);
    
    NS_LOG_INFO("eDRX: PTW started, duration=" << m_params.pagingTimeWindow.As(Time::S));
}

void
NbIotEdrxManager::PtwEnd()
{
    NS_LOG_FUNCTION(this);
    
    m_lastPtwEnd = Simulator::Now();
    
    // Power down PHY
    if (m_phy)
    {
        // In real implementation, would power down receiver
    }
    
    SwitchToState(NbIotEdrxState::SLEEP);
    
    // Schedule next PTW
    Time nextPtwStart = CalculateNextPtwStart();
    m_ptwStartEvent = Simulator::Schedule(nextPtwStart,
                                           &NbIotEdrxManager::PtwStart,
                                           this);
    
    NS_LOG_INFO("eDRX: PTW ended, next PTW in " << nextPtwStart.As(Time::S));
}

void
NbIotEdrxManager::SwitchToState(NbIotEdrxState newState)
{
    if (m_state == newState)
    {
        return;
    }
    
    NbIotEdrxState oldState = m_state;
    m_state = newState;
    
    m_stateTransitionTrace(oldState, newState);
    
    if (!m_stateChangeCallback.IsNull())
    {
        m_stateChangeCallback(oldState, newState);
    }
    
    NS_LOG_DEBUG("eDRX state: " << EdrxStateToString(oldState)
                 << " -> " << EdrxStateToString(newState));
}

// ====================== NbIotPowerSavingController ======================

NS_OBJECT_ENSURE_REGISTERED(NbIotPowerSavingController);

TypeId
NbIotPowerSavingController::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NbIotPowerSavingController")
        .SetParent<Object>()
        .SetGroupName("NbIot")
        .AddConstructor<NbIotPowerSavingController>()
        .AddTraceSource("Power",
                        "Current power consumption",
                        MakeTraceSourceAccessor(&NbIotPowerSavingController::m_powerTrace),
                        "ns3::NbIotPowerSavingController::PowerTracedCallback");
    return tid;
}

NbIotPowerSavingController::NbIotPowerSavingController()
    : m_connected(false)
    , m_activeTime(Seconds(0))
    , m_idleTime(Seconds(0))
    , m_psmTime(Seconds(0))
    , m_edrxSleepTime(Seconds(0))
{
    NS_LOG_FUNCTION(this);
}

NbIotPowerSavingController::~NbIotPowerSavingController()
{
    NS_LOG_FUNCTION(this);
}

void
NbIotPowerSavingController::DoDispose()
{
    m_psm = nullptr;
    m_edrx = nullptr;
    Object::DoDispose();
}

void NbIotPowerSavingController::SetPsmManager(Ptr<NbIotPsmManager> psm) { m_psm = psm; }
void NbIotPowerSavingController::SetEdrxManager(Ptr<NbIotEdrxManager> edrx) { m_edrx = edrx; }
Ptr<NbIotPsmManager> NbIotPowerSavingController::GetPsmManager() const { return m_psm; }
Ptr<NbIotEdrxManager> NbIotPowerSavingController::GetEdrxManager() const { return m_edrx; }

void
NbIotPowerSavingController::Enable(bool enablePsm, bool enableEdrx)
{
    NS_LOG_FUNCTION(this << enablePsm << enableEdrx);
    
    if (m_psm)
    {
        m_psm->SetEnabled(enablePsm);
    }
    if (m_edrx)
    {
        m_edrx->SetEnabled(enableEdrx);
    }
}

void
NbIotPowerSavingController::NotifyConnectionState(bool connected)
{
    NS_LOG_FUNCTION(this << connected);
    
    m_connected = connected;
    
    if (connected)
    {
        // RRC connected - stop eDRX, exit PSM if needed
        if (m_edrx)
        {
            m_edrx->Stop();
        }
        if (m_psm)
        {
            m_psm->NotifyActivity();
        }
    }
    else
    {
        // RRC idle - start power saving features
        if (m_edrx && m_edrx->IsEnabled())
        {
            m_edrx->Start();
        }
        if (m_psm && m_psm->IsEnabled())
        {
            m_psm->NotifyIdleMode();
        }
    }
}

void
NbIotPowerSavingController::NotifyDataActivity()
{
    NS_LOG_FUNCTION(this);
    
    m_lastActivity = Simulator::Now();
    
    if (m_psm)
    {
        m_psm->NotifyActivity();
    }
}

double
NbIotPowerSavingController::GetEstimatedBatteryLife(double batteryCapacity, double voltage) const
{
    double avgPower = GetAveragePower();
    
    if (avgPower <= 0)
    {
        return 0;
    }
    
    // Battery energy in mWh
    double batteryEnergy = batteryCapacity * voltage;
    
    // Life in hours
    return batteryEnergy / avgPower;
}

double
NbIotPowerSavingController::GetAveragePower() const
{
    // Calculate weighted average power consumption
    Time totalTime = m_activeTime + m_idleTime + m_psmTime + m_edrxSleepTime;
    
    if (totalTime <= Seconds(0))
    {
        return 0;
    }
    
    double totalEnergy = 0;
    
    // Active power
    totalEnergy += NbIotPsmManager::POWER_ACTIVE * m_activeTime.GetSeconds();
    
    // Idle power
    totalEnergy += NbIotPsmManager::POWER_IDLE * m_idleTime.GetSeconds();
    
    // PSM power
    totalEnergy += NbIotPsmManager::POWER_PSM * m_psmTime.GetSeconds();
    
    // eDRX sleep power
    totalEnergy += NbIotEdrxManager::POWER_SLEEP * m_edrxSleepTime.GetSeconds();
    
    // Average in mW
    return totalEnergy / totalTime.GetSeconds();
}

} // namespace ns3
