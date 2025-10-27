# AXI to APB Bridge for Hardware Acceleration

![SystemVerilog](https://img.shields.io/badge/language-SystemVerilog-orange.svg)

A protocol converter bridge that converts AXI4 protocol to APB4, connecting AXI masters with APB peripherals in SoC systems.

## Table of Contents

- [AXI to APB Bridge for Hardware Acceleration](#axi-to-apb-bridge-for-hardware-acceleration)
  - [Table of Contents](#table-of-contents)
  - [Theoretical Foundations](#theoretical-foundations)
    - [AMBA Protocol Overview](#amba-protocol-overview)
    - [AXI4 Protocol Characteristics](#axi4-protocol-characteristics)
      - [Channel Architecture](#channel-architecture)
      - [Transaction Properties](#transaction-properties)
      - [Handshake Protocol](#handshake-protocol)
    - [APB4 Protocol Characteristics](#apb4-protocol-characteristics)
      - [Transaction Phases](#transaction-phases)
      - [Key Characteristics](#key-characteristics)
    - [Bridge Design Principles](#bridge-design-principles)
      - [Protocol Conversion Challenges](#protocol-conversion-challenges)
      - [Design Solutions](#design-solutions)
    - [FIFO Buffering Theory](#fifo-buffering-theory)
    - [FIFO Buffering Theory](#fifo-buffering-theory-1)
      - [Purpose of FIFO Buffering](#purpose-of-fifo-buffering)
      - [FIFO Design Considerations](#fifo-design-considerations)
      - [Mathematical Analysis](#mathematical-analysis)
  - [Overview](#overview)
    - [Technical Specifications](#technical-specifications)
  - [System Architecture](#system-architecture)
  - [Module Structure](#module-structure)
    - [FIFO Bit Packing](#fifo-bit-packing)
  - [🔄 State Machine Diagram](#-state-machine-diagram)
    - [State Descriptions](#state-descriptions)

## Theoretical Foundations

### AMBA Protocol Overview

The **ARM Advanced Microcontroller Bus Architecture (AMBA)** is an open-standard, on-chip interconnect specification for the connection and management of functional blocks in system-on-a-chip (SoC) designs.
AMBA defines several bus protocols:

- **AXI (Advanced eXtensible Interface)**: High-performance, high-frequency system backbone
- **APB (Advanced Peripheral Bus)**: Low-power, simple peripheral interface
- **AHB (Advanced High-performance Bus)**: High-performance system bus

### AXI4 Protocol Characteristics

**AXI4** is designed for high-bandwidth, low-latency interconnects with the following key features:

#### Channel Architecture
AXI4 uses **5 independent channels** for maximum throughput:

| Channel | Direction | Purpose | Key Signals |
|---|---|---|---|
| **AW** (Address Write) | Master → Slave | Write address/control | `awaddr`, `awlen`, `awsize`, `awburst` |
| **W** (Write Data) | Master → Slave | Write data | `wdata`, `wstrb`, `wlast` |
| **B** (Write Response) | Slave → Master | Write completion | `bresp`, `bid` |
| **AR** (Address Read) | Master → Slave | Read address/control | `araddr`, `arlen`, `arsize`, `arburst` |
| **R** (Read Data) | Slave → Master | Read data | `rdata`, `rresp`, `rlast` |

#### Transaction Properties
- **Burst Support**: Up to 256 beats per transaction (`awlen[7:0]`)
- **Outstanding Transactions**: Multiple concurrent transactions via ID tagging
- **Variable Transfer Size**: 1, 2, 4, 8, 16, 32, 64, 128 bytes per beat
- **Burst Types**: FIXED, INCR, WRAP addressing modes

#### Handshake Protocol
Every AXI channel uses **ready/valid handshaking**:
```
Master ──► valid ──► Slave
       ◄── ready ◄──
```
Transfer occurs when both `valid` and `ready` are HIGH.

### APB4 Protocol Characteristics

**APB4** is optimized for low-power, low-complexity peripheral access:

#### Transaction Phases
APB uses a **2-phase protocol**:

1. **SETUP Phase**: Address and control signals stable
2. **ACCESS Phase**: Data transfer occurs

#### Key Characteristics
- **Single Transaction**: No burst support
- **Simple Handshake**: Uses `PSEL`, `PENABLE`, `PREADY`
- **Low Power**: Minimal switching activity
- **Fixed 32-bit Width**: No variable transfer sizes

### Bridge Design Principles

#### Protocol Conversion Challenges

1. **Bandwidth Mismatch**: AXI (burst) → APB (single)
2. **Timing Domains**: Different handshake mechanisms
3. **Transaction Ordering**: Maintain AXI ordering semantics
4. **Error Handling**: Convert response codes appropriately

#### Design Solutions

**1. FIFO Buffering**
```
AXI Burst (8 beats) → [FIFO Buffer] → APB Single × 8
```

**2. State Machine Control**
```
AXI Request → Parse → APB Sequence → AXI Response
```

**3. Transaction Splitting**
```
AXI: INCR4 @ 0x1000 → APB: 0x1000, 0x1004, 0x1008, 0x100C
```

### FIFO Buffering Theory

### FIFO Buffering Theory

#### Purpose of FIFO Buffering

**Decoupling**: Separate AXI and APB timing domains
**Burst Handling**: Store multiple beats from AXI burst
**Performance**: Reduce blocking between protocols

#### FIFO Design Considerations

**1. Depth Calculation**
```
FIFO_DEPTH ≥ MAX_BURST_LENGTH × PIPELINE_STAGES
For AXI4: FIFO_DEPTH ≥ 256 × 2 = 512 (theoretical)
Practical: 16-32 entries for most applications
```

**2. Width Optimization**
Pack related signals together to minimize memory usage:
```verilog
// Request FIFO: 53 bits
{addr[31:0], write, id[3:0], len[7:0], burst[1:0], size[2:0], prot[2:0]}

// Write Data FIFO: 41 bits
{data[31:0], strb[3:0], last, id[3:0]}
```

**3. Flow Control**
- **Back-pressure**: Assert `ready=0` when FIFO full
- **Empty Detection**: Trigger FSM when data available
- **Almost Full/Empty**: Prevent overflow/underflow

#### Mathematical Analysis

**Throughput Calculation**:
```
T_axi = f_clk / (burst_length × cycles_per_beat)
T_apb = f_clk / (2 × transactions)  // 2-phase protocol
T_bridge = min(T_axi, T_apb)
```

**Latency Analysis**:
```
Latency = T_parse + T_fifo + T_apb + T_response
        = 1 + 1 + 2 + 1 = 5 cycles (typical)
```

##  Overview

The AXI to APB Bridge converts AXI4 protocol to APB4 with key features:
- ✅ Full AXI4 burst transaction support
- ✅ FIFO buffering for optimal performance
- ✅ Error handling and timeout detection
- ✅ AMBA 4.0 specification compliance

### Technical Specifications

| Parameter | Value | Description |
|-----------|-------|------------|
| **Data Width** | 32-bit | AXI/APB data width |
| **Address Width** | 32-bit | Address bus width |
| **ID Width** | 4-bit | AXI transaction ID |
| **FIFO Depth** | 16 entries | Buffer depth |
| **Clock Frequency** | 100 MHz | Target frequency |

##  System Architecture
![System Architecture](SystemArchitecture.png)

##  Module Structure

```
AXI-LOCAL/
├── core.sv                     #  Top-level integration
├── axi_interface.sv            #  AXI protocol handler  
├── fsm.sv                      #  State machine controller
├── fifo_read.sv                #  Read data FIFO (38-bit)
├── fifo_write.sv               # Write data FIFO (40-bit)
├── apb_interface.sv            #  APB output interface
├── testbench.sv                #  SystemVerilog testbench
├── axi_slave.sv                # Enterprise wrapper
├── UVM/                        # UVM verification environment
├── VIVADO/                     # Xilinx project files
└── QUARTUS/                    # Intel project files
```

### FIFO Bit Packing

| FIFO | Width | Content |
|------|-------|---------|
| **Request** | 53-bit | `addr[31:0] + write + id[3:0] + len[7:0] + burst[1:0] + size[2:0] + prot[2:0]` |
| **Write Data** | 41-bit | `data[31:0] + strb[3:0] + last + id[3:0]` |
| **Read Data** | 39-bit | `data[31:0] + id[3:0] + last + resp[1:0]` |

## 🔄 State Machine Diagram

The FSM implements **11 states** with comprehensive error handling:
![Finite State Diagram](FiniteStateDiagram.png)

### State Descriptions

| State | Purpose | Next State Conditions |
|-------|---------|----------------------|
| **IDLE** | Wait for requests | `!req_fifo_empty` → FETCH_CMD |
| **FETCH_CMD** | Read command from FIFO | Always → WAIT_CMD_DATA |
| **WAIT_CMD_DATA** | Process command | `write=1` → FETCH_DATA, `write=0` → APB_SETUP |
| **FETCH_DATA** | Read write data | Always → WAIT_WRITE_DATA |
| **WAIT_WRITE_DATA** | Prepare write data | Always → APB_SETUP |
| **APB_SETUP** | APB setup phase | Always → APB_ACCESS |
| **APB_ACCESS** | APB access phase | `pready & !pslverr & write` → WRITE_RESP<br>`pready & !pslverr & !write` → READ_STORE<br>`timeout \| pslverr` → ERROR |
| **WRITE_RESP** | Send write response | `bresp_ready` → IDLE |
| **READ_STORE** | Store read data | `!rdata_fifo_full` → IDLE<br>`rdata_fifo_full` → ERROR |
| **ERROR** | Handle errors | `bresp_ready` → IDLE |

