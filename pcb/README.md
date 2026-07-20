revisions of PCB

H - first pcb ever, works and is tested, needs external temperature circuit for transmission and engine temps

I - second revision TH, very similar with H but now with internal temperature circuit

J - did not pass testing, not included.

K - first SMD revision, tested working and one that I currently use when driving.

L - NEW (in design). universal multi-transmission controller on RP2350B.
    Adds 6x current-controlled + 4x on/off solenoid drivers (covers EDS/EPC/TCC
    and on/off shift solenoids for 722.6, 4L60E/4L80E, 6L80, 4R70W, 8HP, 10-speed),
    3x VR/Hall speed inputs, CAN-FD (MCP2518FD), external SPI ADC (MCP3208),
    I2C expander for LEDs, buck regulator (no 7805), and reverse-polarity protection.
    Superset of revK channels -> stays backward compatible with the 722.6.
    See revL/DESIGN.md for the full spec, BOM, netlist and reference Gerber outline.
    (pre-layout: EDA routing is the remaining handoff task.)


firmware is compatible for all revisions, things to be noted is that revH has small output wire holes, which can be bit difficult.

and n2/n3rpm resistor in series should be removed and replaced with wire on all revisions.

I like to use 10A fuse on 12V power input just to be on safe side, works with 14V+ voltages, but cleaner the voltage better. 

not tested on gasoline engines how spark affects yet.
 
