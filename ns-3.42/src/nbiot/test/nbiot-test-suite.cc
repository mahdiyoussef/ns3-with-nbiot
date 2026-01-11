/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: NB-IoT Module Development
 *
 * NB-IoT Test Suite
 */

#include "ns3/test.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/double.h"
#include "ns3/boolean.h"
#include "ns3/pointer.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/constant-position-mobility-model.h"

#include "ns3/nbiot-common.h"
#include "ns3/nbiot-phy.h"
#include "ns3/nbiot-ue-phy.h"
#include "ns3/nbiot-enb-phy.h"
#include "ns3/nbiot-mac.h"
#include "ns3/nbiot-ue-mac.h"
#include "ns3/nbiot-enb-mac.h"
#include "ns3/nbiot-rlc.h"
#include "ns3/nbiot-pdcp.h"
#include "ns3/nbiot-rrc.h"
#include "ns3/nbiot-power-saving.h"
#include "ns3/nbiot-net-device.h"
#include "ns3/nbiot-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NbIotTestSuite");

/**
 * \ingroup nbiot-tests
 * \brief Test case for NB-IoT common types and helpers
 */
class NbIotCommonTestCase : public TestCase
{
public:
    NbIotCommonTestCase();
    virtual ~NbIotCommonTestCase();

private:
    void DoRun() override;
};

NbIotCommonTestCase::NbIotCommonTestCase()
    : TestCase("NB-IoT common types test")
{
}

NbIotCommonTestCase::~NbIotCommonTestCase()
{
}

void
NbIotCommonTestCase::DoRun()
{
    // Test constants
    NS_TEST_ASSERT_MSG_EQ(NbIotConstants::SYSTEM_BANDWIDTH_HZ, 180000.0,
                          "NB-IoT bandwidth should be 180 kHz");
    NS_TEST_ASSERT_MSG_EQ(NbIotConstants::SUBCARRIERS_PER_PRB, 12,
                          "NB-IoT should have 12 subcarriers");
    NS_TEST_ASSERT_MSG_EQ(NbIotConstants::DL_SUBCARRIER_SPACING_HZ, 15000.0,
                          "DL subcarrier spacing should be 15 kHz");
    NS_TEST_ASSERT_MSG_EQ(NbIotConstants::SUBFRAMES_PER_FRAME, 10,
                          "Should have 10 subframes per frame");
    NS_TEST_ASSERT_MSG_EQ(NbIotConstants::FRAME_DURATION_MS, 10.0,
                          "Frame duration should be 10 ms");
    
    // Test repetition tables
    NS_TEST_ASSERT_MSG_EQ(NPDCCH_REPETITIONS[2], 4,
                          "NPDCCH repetitions index 2 should be 4");
    NS_TEST_ASSERT_MSG_EQ(NPDSCH_REPETITIONS[7], 8,
                          "NPDSCH repetitions index 7 should be 8");
}

/**
 * \ingroup nbiot-tests
 * \brief Test case for NB-IoT PHY layer
 */
class NbIotPhyTestCase : public TestCase
{
public:
    NbIotPhyTestCase();
    virtual ~NbIotPhyTestCase();

private:
    void DoRun() override;
};

NbIotPhyTestCase::NbIotPhyTestCase()
    : TestCase("NB-IoT PHY layer test")
{
}

NbIotPhyTestCase::~NbIotPhyTestCase()
{
}

void
NbIotPhyTestCase::DoRun()
{
    // Test UE PHY creation
    Ptr<NbIotUePhy> uePhy = CreateObject<NbIotUePhy>();
    NS_TEST_ASSERT_MSG_NE(uePhy, nullptr, "UE PHY should be created");

    // Test eNB PHY creation
    Ptr<NbIotEnbPhy> enbPhy = CreateObject<NbIotEnbPhy>();
    NS_TEST_ASSERT_MSG_NE(enbPhy, nullptr, "eNB PHY should be created");

    // Test deployment mode setting
    uePhy->SetDeploymentMode(NbIotDeploymentMode::STANDALONE);
    NS_TEST_ASSERT_MSG_EQ(uePhy->GetDeploymentMode(), NbIotDeploymentMode::STANDALONE,
                          "Deployment mode should be STANDALONE");

    uePhy->SetDeploymentMode(NbIotDeploymentMode::IN_BAND);
    NS_TEST_ASSERT_MSG_EQ(uePhy->GetDeploymentMode(), NbIotDeploymentMode::IN_BAND,
                          "Deployment mode should be IN_BAND");
}

/**
 * \ingroup nbiot-tests
 * \brief Test case for NB-IoT MAC layer
 */
class NbIotMacTestCase : public TestCase
{
public:
    NbIotMacTestCase();
    virtual ~NbIotMacTestCase();

private:
    void DoRun() override;
};

NbIotMacTestCase::NbIotMacTestCase()
    : TestCase("NB-IoT MAC layer test")
{
}

NbIotMacTestCase::~NbIotMacTestCase()
{
}

void
NbIotMacTestCase::DoRun()
{
    // Test UE MAC creation
    Ptr<NbIotUeMac> ueMac = CreateObject<NbIotUeMac>();
    NS_TEST_ASSERT_MSG_NE(ueMac, nullptr, "UE MAC should be created");

    // Test eNB MAC creation
    Ptr<NbIotEnbMac> enbMac = CreateObject<NbIotEnbMac>();
    NS_TEST_ASSERT_MSG_NE(enbMac, nullptr, "eNB MAC should be created");
}

/**
 * \ingroup nbiot-tests
 * \brief Test case for NB-IoT RLC layer
 */
class NbIotRlcTestCase : public TestCase
{
public:
    NbIotRlcTestCase();
    virtual ~NbIotRlcTestCase();

private:
    void DoRun() override;
};

NbIotRlcTestCase::NbIotRlcTestCase()
    : TestCase("NB-IoT RLC layer test")
{
}

NbIotRlcTestCase::~NbIotRlcTestCase()
{
}

void
NbIotRlcTestCase::DoRun()
{
    // Test RLC TM creation
    Ptr<NbIotRlcTm> rlcTm = CreateObject<NbIotRlcTm>();
    NS_TEST_ASSERT_MSG_NE(rlcTm, nullptr, "RLC TM should be created");

    // Test RLC UM creation
    Ptr<NbIotRlcUm> rlcUm = CreateObject<NbIotRlcUm>();
    NS_TEST_ASSERT_MSG_NE(rlcUm, nullptr, "RLC UM should be created");

    // Test RLC AM creation
    Ptr<NbIotRlcAm> rlcAm = CreateObject<NbIotRlcAm>();
    NS_TEST_ASSERT_MSG_NE(rlcAm, nullptr, "RLC AM should be created");
}

/**
 * \ingroup nbiot-tests
 * \brief Test case for NB-IoT PDCP layer
 */
class NbIotPdcpTestCase : public TestCase
{
public:
    NbIotPdcpTestCase();
    virtual ~NbIotPdcpTestCase();

private:
    void DoRun() override;
};

NbIotPdcpTestCase::NbIotPdcpTestCase()
    : TestCase("NB-IoT PDCP layer test")
{
}

NbIotPdcpTestCase::~NbIotPdcpTestCase()
{
}

void
NbIotPdcpTestCase::DoRun()
{
    // Test PDCP SRB creation
    Ptr<NbIotPdcpSrb> pdcpSrb = CreateObject<NbIotPdcpSrb>();
    NS_TEST_ASSERT_MSG_NE(pdcpSrb, nullptr, "PDCP SRB should be created");

    // Test PDCP DRB creation
    Ptr<NbIotPdcpDrb> pdcpDrb = CreateObject<NbIotPdcpDrb>();
    NS_TEST_ASSERT_MSG_NE(pdcpDrb, nullptr, "PDCP DRB should be created");
}

/**
 * \ingroup nbiot-tests
 * \brief Test case for NB-IoT RRC layer
 */
class NbIotRrcTestCase : public TestCase
{
public:
    NbIotRrcTestCase();
    virtual ~NbIotRrcTestCase();

private:
    void DoRun() override;
};

NbIotRrcTestCase::NbIotRrcTestCase()
    : TestCase("NB-IoT RRC layer test")
{
}

NbIotRrcTestCase::~NbIotRrcTestCase()
{
}

void
NbIotRrcTestCase::DoRun()
{
    // Test UE RRC creation
    Ptr<NbIotUeRrc> ueRrc = CreateObject<NbIotUeRrc>();
    NS_TEST_ASSERT_MSG_NE(ueRrc, nullptr, "UE RRC should be created");

    // Test eNB RRC creation
    Ptr<NbIotEnbRrc> enbRrc = CreateObject<NbIotEnbRrc>();
    NS_TEST_ASSERT_MSG_NE(enbRrc, nullptr, "eNB RRC should be created");
}

/**
 * \ingroup nbiot-tests
 * \brief Test case for NB-IoT Power Saving features
 */
class NbIotPowerSavingTestCase : public TestCase
{
public:
    NbIotPowerSavingTestCase();
    virtual ~NbIotPowerSavingTestCase();

private:
    void DoRun() override;
};

NbIotPowerSavingTestCase::NbIotPowerSavingTestCase()
    : TestCase("NB-IoT Power Saving test")
{
}

NbIotPowerSavingTestCase::~NbIotPowerSavingTestCase()
{
}

void
NbIotPowerSavingTestCase::DoRun()
{
    // Test PSM manager creation
    Ptr<NbIotPsmManager> psm = CreateObject<NbIotPsmManager>();
    NS_TEST_ASSERT_MSG_NE(psm, nullptr, "PSM manager should be created");

    // Test PSM configuration
    psm->SetEnabled(true);
    NS_TEST_ASSERT_MSG_EQ(psm->IsEnabled(), true, "PSM should be enabled");

    psm->SetEnabled(false);
    NS_TEST_ASSERT_MSG_EQ(psm->IsEnabled(), false, "PSM should be disabled");

    // Test eDRX manager creation and configuration
    Ptr<NbIotEdrxManager> edrx = CreateObject<NbIotEdrxManager>();
    NS_TEST_ASSERT_MSG_NE(edrx, nullptr, "eDRX manager should be created");

    edrx->SetEnabled(true);
    NS_TEST_ASSERT_MSG_EQ(edrx->IsEnabled(), true, "eDRX should be enabled");
}

/**
 * \ingroup nbiot-tests
 * \brief Test case for NB-IoT Net Device
 */
class NbIotNetDeviceTestCase : public TestCase
{
public:
    NbIotNetDeviceTestCase();
    virtual ~NbIotNetDeviceTestCase();

private:
    void DoRun() override;
};

NbIotNetDeviceTestCase::NbIotNetDeviceTestCase()
    : TestCase("NB-IoT Net Device test")
{
}

NbIotNetDeviceTestCase::~NbIotNetDeviceTestCase()
{
}

void
NbIotNetDeviceTestCase::DoRun()
{
    // Create a node
    Ptr<Node> node = CreateObject<Node>();

    // Test UE device creation
    Ptr<NbIotUeNetDevice> ueDev = CreateObject<NbIotUeNetDevice>();
    NS_TEST_ASSERT_MSG_NE(ueDev, nullptr, "UE device should be created");
    ueDev->SetNode(node);
    NS_TEST_ASSERT_MSG_EQ(ueDev->GetNode(), node, "Node should be set correctly");

    // Test eNB device creation
    Ptr<Node> enbNode = CreateObject<Node>();
    Ptr<NbIotEnbNetDevice> enbDev = CreateObject<NbIotEnbNetDevice>();
    NS_TEST_ASSERT_MSG_NE(enbDev, nullptr, "eNB device should be created");
    enbDev->SetNode(enbNode);
    NS_TEST_ASSERT_MSG_EQ(enbDev->GetNode(), enbNode, "Node should be set correctly");
}

/**
 * \ingroup nbiot-tests
 * \brief Test case for NB-IoT Helper
 */
class NbIotHelperTestCase : public TestCase
{
public:
    NbIotHelperTestCase();
    virtual ~NbIotHelperTestCase();

private:
    void DoRun() override;
};

NbIotHelperTestCase::NbIotHelperTestCase()
    : TestCase("NB-IoT Helper test")
{
}

NbIotHelperTestCase::~NbIotHelperTestCase()
{
}

void
NbIotHelperTestCase::DoRun()
{
    // Test helper creation
    Ptr<NbIotHelper> helper = CreateObject<NbIotHelper>();
    NS_TEST_ASSERT_MSG_NE(helper, nullptr, "Helper should be created");

    // Test deployment mode setting
    helper->SetDeploymentMode(NbIotDeploymentMode::STANDALONE);
    NS_TEST_ASSERT_MSG_EQ(helper->GetDeploymentMode(), NbIotDeploymentMode::STANDALONE,
                          "Deployment mode should be STANDALONE");

    helper->SetDeploymentMode(NbIotDeploymentMode::GUARD_BAND);
    NS_TEST_ASSERT_MSG_EQ(helper->GetDeploymentMode(), NbIotDeploymentMode::GUARD_BAND,
                          "Deployment mode should be GUARD_BAND");
}

/**
 * \ingroup nbiot-tests
 * \brief NB-IoT Test Suite
 */
class NbIotTestSuite : public TestSuite
{
public:
    NbIotTestSuite();
};

NbIotTestSuite::NbIotTestSuite()
    : TestSuite("nbiot", Type::UNIT)
{
    AddTestCase(new NbIotCommonTestCase, TestCase::Duration::QUICK);
    AddTestCase(new NbIotPhyTestCase, TestCase::Duration::QUICK);
    AddTestCase(new NbIotMacTestCase, TestCase::Duration::QUICK);
    AddTestCase(new NbIotRlcTestCase, TestCase::Duration::QUICK);
    AddTestCase(new NbIotPdcpTestCase, TestCase::Duration::QUICK);
    AddTestCase(new NbIotRrcTestCase, TestCase::Duration::QUICK);
    AddTestCase(new NbIotPowerSavingTestCase, TestCase::Duration::QUICK);
    AddTestCase(new NbIotNetDeviceTestCase, TestCase::Duration::QUICK);
    AddTestCase(new NbIotHelperTestCase, TestCase::Duration::QUICK);
}

static NbIotTestSuite g_nbiotTestSuite;

