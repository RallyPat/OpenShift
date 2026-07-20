# revL — universal multi-transmission controller PCB

> **Status:** in design, **pre-layout**. Architecture, netlist and BOM are
> locked; KiCad/Altium routing is the remaining handoff task.

revL is a new SMD board built around the **Raspberry Pi RP2350B** (48 GPIO)
that drives **any** transmission OpenShift supports by exposing a superset of
actuator/sensor channels addressed by the `transictrl` calibration
`io_mapping` model.

**Bring-up priority:** 4L60E/4L80E first (only test unit on hand) → ZF 8HP
(design goal, gated on obtaining a unit) → remaining transmissions.

## Files in this folder

| File | Purpose |
| ---- | ------- |
| [`DESIGN.md`](DESIGN.md) | Full design spec: architecture, block diagram, locked decisions, power tree, solenoid-driver topology, GPIO netlist, 4-layer stack, transmission coverage matrix, verification plan |
| [`revL.BOM`](revL.BOM) | Reference bill of materials (RP2350B, INA240A2, MAX9926, MCP2518FD/2562FD, MCP3208, PCA9555, TPS54331 buck, LM74610, …) |
| [`NETLIST.tsv`](NETLIST.tsv) | GPIO → net → connector pin map for EDA import |
| [`gerber/revL.GKO`](gerber/revL.GKO) | Reference board-outline Gerber (RS-274X text, 120 × 100 mm) |

## What's new vs. revK (the current daily-driver SMD board)

- **Multi-transmission**: one board for 4L60E/4L80E, 722.6, 6L80, 4R70W, 8HP,
  10-speed — backward compatible with the 722.6.
- **Mixed solenoid drivers**: 6 current-controlled (INA240 + sense R, closed
  loop) **and** 4 on/off low-side — covers proportional EDS/EPC/TCC *and*
  on/off shift solenoids.
- **CAN-FD** (MCP2518FD + MCP2562FD) for 8HP/10-speed and modern-vehicle bus.
- **External SPI ADC** (MCP3208) for current feedback + line pressure + EGT.
- **3× VR/Hall speed inputs** (2× MAX9926).
- **Buck regulator** (replaces the 7805), **reverse-polarity protection**
  (LM74610), **I²C LED expander** (PCA9555).
- **Fixes** the known revH–K issues: N2/N3 series resistor → 0 Ω strap; enlarged
  output wire holes; internal EGT circuit.

## 4L60E channel mapping (primary test unit)

| Transmission solenoid | revL ch | Driver |
| -- | --: | -- |
| SS-A / SS-B (shift) | 6 / 7 | on/off (Group B) |
| EPC (line pressure) | 5 | closed-loop current (Group A) |
| TCC (lockup) | 0 | open-loop PWM (Group A) |
| 3-2 control | 1 | open-loop PWM (Group A) |

See `DESIGN.md` §10.1 for the full mapping and §11 for the bench bring-up plan.

## Firmware

The matching firmware pin config is
[`transictrl/fw/fw-rp2350/src/rev_l.rs`](../../transictrl/fw/fw-rp2350/src/rev_l.rs)
(`RevLPinConfig::revl()`). Closed-loop current control (`current_pi`) and the
CAN receive path (`can`) are implemented and host-tested in `transictrl-hal`.
