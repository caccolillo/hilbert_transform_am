# Hilbert Transform AM Link

## Overview

An FPGA-based amplitude modulation (AM) communication system using Hilbert transform signal processing for real-time modulation and demodulation.

**Repository**: [caccolillo/hilbert_transform_am](https://github.com/caccolillo/hilbert_transform_am)

---

## What is This?

This project implements a complete AM radio link on Xilinx FPGA hardware. It uses the Hilbert transform to generate analytic signals for envelope detection and demodulation, demonstrating practical DSP techniques in communication systems.

**Key Features**:
- Real-time AM modulation and demodulation
- Hilbert transform-based envelope detection
- Audio input/output processing
- FPGA hardware acceleration

---

## Project Structure

```
hilbert_transform_am/
├── audio/          # Audio processing components
├── docs/           # Documentation
├── hls/            # High-Level Synthesis C/C++ code
├── petalinux/      # Embedded Linux configuration
├── simulink/       # MATLAB models and simulations
├── vitis/          # Software application (Xilinx Vitis)
├── vivado/         # FPGA design files (Xilinx Vivado)
├── README.PDF
└── README.md

```

---

## Technical Background

### Hilbert Transform

The Hilbert transform shifts all frequency components of a signal by -90°, enabling creation of **analytic signals**:

```
z(t) = x(t) + j·H{x(t)}
```

Where the magnitude |z(t)| gives the signal envelope, critical for AM demodulation.

### AM Modulation

Standard amplitude modulation:
```
s(t) = [A_c + m(t)] · cos(2πf_c·t)
```

Where m(t) is the message signal and f_c is the carrier frequency.

---

## System Architecture


**Hardware Platform**: Ultra96 v2
- ARM processor (Linux) for control
- FPGA fabric for DSP processing

---

## Requirements

### Software
- Xilinx Vivado Design Suite (2022.1+)
- Xilinx Vitis Platform
- MATLAB/Simulink (optional, for simulation)
- PetaLinux Tools

---

## Applications

- **Education**: Learning DSP and communication theory
- **Amateur Radio**: Voice communication systems
- **Research**: Advanced modulation techniques
- **SDR**: Software Defined Radio implementation

---



