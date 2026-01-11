# NB-IoT Module for NS-3

## Overview

This module implements NB-IoT (Narrowband Internet of Things) for NS-3, following 3GPP Release 13/14 specifications. NB-IoT is a Low Power Wide Area Network (LPWAN) technology designed for massive IoT deployments with extended coverage and long battery life.

## Features

### Physical Layer (PHY)
- **180 kHz bandwidth** (single PRB)
- **12 subcarriers** for NB-IoT carrier
- **Three deployment modes**:
  - Standalone (independent carrier)
  - Guard-band (in LTE guard band)
  - In-band (within LTE carrier)
- **Physical channels**:
  - NPBCH (NB Physical Broadcast Channel)
  - NPDCCH (NB Physical Downlink Control Channel)
  - NPDSCH (NB Physical Downlink Shared Channel)
  - NPUSCH Format 1 and Format 2 (NB Physical Uplink Shared Channel)
  - NPRACH (NB Physical Random Access Channel)
- **Synchronization signals**: NPSS, NSSS, NRS

### MAC Layer
- **MAC PDU handling** with proper subheaders
- **HARQ support** for both DL and UL
- **Buffer Status Reporting (BSR)**
- **Power Headroom Reporting (PHR)**
- **Random Access procedure** with multiple NPRACH resources
- **Schedulers**:
  - Round-robin scheduler
  - Coverage-class aware scheduler

### RLC Layer
- **Three modes**:
  - Transparent Mode (TM)
  - Unacknowledged Mode (UM)
  - Acknowledged Mode (AM) with ARQ

### PDCP Layer
- **Header compression** (simplified ROHC)
- **Sequence numbering**
- **In-order delivery**
- **Duplicate detection**

### RRC Layer
- **State machines**: IDLE, CONNECTED
- **System Information** (MIB, SIB1, SIB2)
- **Connection establishment/release**
- **Timers**: T300, T301, T311

### Power Saving
- **Power Saving Mode (PSM)**:
  - T3324 (Active Timer): 2s to 31 min
  - T3412 (TAU Timer): up to 310 hours
- **Extended DRX (eDRX)**:
  - eDRX cycles: up to 2.91 hours
  - Paging Time Window (PTW): 2.56s to 40.96s

### Coverage Enhancement
- **Three coverage classes**:
  - CE Level 0: MCL ≤ 144 dB (normal)
  - CE Level 1: MCL ≤ 154 dB (extended)
  - CE Level 2: MCL ≤ 164 dB (extreme)
- **Repetition-based enhancement** for all channels

## Installation

1. Copy the `nbiot` folder to `ns-3.42/src/`
2. Configure NS-3 with the new module:
   ```bash
   ./ns3 configure --enable-examples --enable-tests
   ```
3. Build:
   ```bash
   ./ns3 build
   ```

## Usage

### Basic Example

```cpp
#include "ns3/nbiot-helper.h"
#include "ns3/nbiot-common.h"

// Create nodes
NodeContainer enbNode, ueNodes;
enbNode.Create(1);
ueNodes.Create(10);

// Create NB-IoT helper
Ptr<NbIotHelper> helper = CreateObject<NbIotHelper>();
helper->SetDeploymentMode(NbIotDeploymentMode::STANDALONE);

// Install devices
NetDeviceContainer enbDevs = helper->InstallEnbDevice(enbNode);
NetDeviceContainer ueDevs = helper->InstallUeDevice(ueNodes);

// Attach UEs to eNB
helper->Attach(ueDevs, enbDevs.Get(0));

// Enable power saving
helper->EnablePowerSaving(ueDevs, true, true);
helper->SetPsmTimers(ueDevs, Seconds(10), Hours(1));
helper->SetEdrxParameters(ueDevs, Seconds(81.92), Seconds(2.56));
```

### Running the Example

```bash
./ns3 run "nbiot-standalone-example --numUes=10 --simTime=100"
```

### Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `--numUes` | Number of UE devices | 10 |
| `--simTime` | Simulation time (seconds) | 100 |
| `--cellRadius` | Cell radius (meters) | 500 |
| `--enablePsm` | Enable PSM | true |
| `--enableEdrx` | Enable eDRX | true |
| `--t3324` | T3324 timer (seconds) | 10 |
| `--t3412` | T3412 timer (seconds) | 3600 |
| `--edrxCycle` | eDRX cycle (seconds) | 81.92 |
| `--ptw` | Paging time window (seconds) | 2.56 |
| `--deployment` | Deployment mode | standalone |

## API Reference

### NbIotHelper

Main helper class for configuring NB-IoT networks.

```cpp
void SetDeploymentMode(NbIotDeploymentMode mode);
NetDeviceContainer InstallEnbDevice(NodeContainer c);
NetDeviceContainer InstallUeDevice(NodeContainer c);
void Attach(NetDeviceContainer ueDevices, Ptr<NetDevice> enbDevice);
void EnablePowerSaving(NetDeviceContainer ueDevices, bool psm, bool edrx);
```

### NbIotUeNetDevice

UE network device with complete protocol stack.

```cpp
Ptr<NbIotUePhy> GetPhy() const;
Ptr<NbIotUeMac> GetMac() const;
Ptr<NbIotUeRrc> GetRrc() const;
Ptr<NbIotPowerSavingController> GetPowerSavingController() const;
```

### NbIotEnbNetDevice

eNB network device with scheduler support.

```cpp
void SetCellId(uint16_t cellId);
void SetTxPower(double power);
Ptr<NbIotEnbMac> GetMac() const;
```

## 3GPP Specifications

This module is based on the following 3GPP specifications:

- **TS 36.211**: Physical channels and modulation
- **TS 36.212**: Multiplexing and channel coding
- **TS 36.213**: Physical layer procedures
- **TS 36.214**: Physical layer measurements
- **TS 36.300**: E-UTRAN overall description
- **TS 36.321**: MAC protocol specification
- **TS 36.322**: RLC protocol specification
- **TS 36.323**: PDCP protocol specification
- **TS 36.331**: RRC protocol specification
- **TS 23.682**: Architecture for Machine-Type Communications
- **TS 24.008**: Mobile radio interface layer 3 specification
- **TS 36.304**: UE procedures in idle mode

## File Structure

```
src/nbiot/
├── CMakeLists.txt
├── doc/
│   └── nbiot.rst
├── examples/
│   └── nbiot-standalone-example.cc
├── helper/
│   ├── nbiot-helper.cc
│   └── nbiot-helper.h
├── model/
│   ├── nbiot-common.h
│   ├── nbiot-control-messages.cc
│   ├── nbiot-control-messages.h
│   ├── nbiot-enb-mac.cc
│   ├── nbiot-enb-mac.h
│   ├── nbiot-enb-phy.cc
│   ├── nbiot-enb-phy.h
│   ├── nbiot-mac.cc
│   ├── nbiot-mac.h
│   ├── nbiot-net-device.cc
│   ├── nbiot-net-device.h
│   ├── nbiot-pdcp.cc
│   ├── nbiot-pdcp.h
│   ├── nbiot-phy.cc
│   ├── nbiot-phy.h
│   ├── nbiot-power-saving.cc
│   ├── nbiot-power-saving.h
│   ├── nbiot-rlc.cc
│   ├── nbiot-rlc.h
│   ├── nbiot-rrc.cc
│   ├── nbiot-rrc.h
│   ├── nbiot-ue-mac.cc
│   ├── nbiot-ue-mac.h
│   ├── nbiot-ue-phy.cc
│   └── nbiot-ue-phy.h
└── test/
    └── nbiot-test-suite.cc
```

## Testing

Run the test suite:

```bash
./test.py -s nbiot
```

## Authors

NB-IoT Module Development Team

## License

GPL-2.0-only
