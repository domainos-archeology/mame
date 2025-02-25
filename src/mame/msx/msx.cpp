// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*
** msx.cpp : driver for MSX
**
** cpc300:
**  To get out of the MSX Tutor press the SELECT key. Entering SET SYSTEM 1 should
**  disable the MSX Tutor on next boot and SET SYSTEM 0 should enable.
**
** tpp311:
**  This machine is supposed to boot into logo; it was made to only run logo.
**
** tps312:
**  - To get into MSX-WRITE type: CALL WRITE
**  - To get into MSX-PLAN type: CALL MSXPLAN
**
**
** There really were a lot of different MSX systems released by quite a few
** manufacturers. Until we have identified all the unique characteristics of
** each machine the machines will stay listed separately. After that
** de-duplication of machine descriptions and rom sets may happen.
**
**
** Todo/known issues:
** - piopx7/piopx7uk/piopxv60: Pioneer System Remote (home entertainment/Laserdisc control) not implemented
** - piopx7: Dump is from a PAL (EU/AU) machine, we have no known good dumps from JP or US NTSC machines
** - spc800: Haven't been able to test operation of the han rom yet
** - svi728: Expansion slot not emulated
** - svi738: v9938 not emulated
** - svi738: rs232c not emulated
** - hx10: Expansion slot not emulated (hx10s also??)
** - y503iir: Keyboard not responding correctly
** - y503iir, y503iir2: RTC not emulated
** - y503iir, y503iir2: Net not emulated
** - y503iir, y503iir2: Floppy support broken
** - cpc300: Config for MSX Tutor ON/OFF is not saved
** - fs4600: Kanji12 not emulated; how to trigger usage of kanji12??
** - fsa1fm: Firmware not emulated
** - fsa1fm: kanji12 not emulated
** - fsa1fm: Modem not emulated
** - nms8280, nms8280g: Digitizer functionality not emulated
** - vg8230j: Floppy support broken?
** - hbf1: Does not boot. This seems to be caused by a race condition between setting the VBlank bit in the
**         VDP status register and the z80 taking the interrupt. Currently the interrupt gets taken before the
**         bit can be read, so the code goes into an infinite loop.
** - hbf12: Does not boot; see hbf1.
** - tpc310: Floppy support broken
**           7fbb <- c7  => seek 199???
** - hx23f: The builtin word processor displays white squares instead of text
** - expert3i: IDE not emulated
** - expert3t: Turbo not emulated
** - expertac: Does not boot
** - fsa1gt: Add Turbo-R support
** - fsa1st: Add Turbo-R support
** - canonv20e/f/g/s: Investigate different keyboard layouts
** - canonv30: Mapper RAM size unknown
** - mx101: External antenna not emulated
** - pv7: Add support for KB-7 (8KB ram + 2 cartslots)
** - cpc50a/cpc50b: Remove keyboard; and add an external keyboard??
** - cpc51/cpc61: Remove keyboard and add a keyboard connector
** - cpc50a/cpc50b/cpc51: Boot to a black screen, is this correct?
** - mbh2: speed controller not implemented
** - mbh70: Verify firmware operation
** - kmc5000: Floppy support broken
** - mlg3: rs232c not emulated
** - perfect1: Firmware broken
** - mpc2500f: Fix keyboard layout?
** - nms8260: HDD not emulated
** - mpc27: Light pen not emulated
** - phc77: firmware not emulated
** - phc77: printer not emulated
** - hx21, hx22: Hook up kanji rom
** - hx21, hx22: Does not start firmware
** - victhc90/95/95a: Turbo/2nd cpu not supported.
** - victhc90/95/95a: Firmware not working.
** - y503iiir/e: Fix keyboard support
** - y503iiir/e: Floppy support broken
** - y805*: Floppy support broken
** - y8805128r2/e: Firmware not working
** - cpg120: Remove ports
** - cpg120: Add V9958
**
** TODO:
** - Add T6950 support. T6950 is selectable between pal and ntsc by a pin.
**
** Possibly missing machines:
** - Sanyo MPC-1 (T6950)
** - Toshibo HX-51i (T7937)
** - Sony HB-101 (TMS9118)
************************************************************************

This following list is probably incomplete. Corrections are welcome.
Entries marked with * still need to be processed.

Al Alamiah AX-150 - MSX1 - ax150
Al Alamiah AX-170 - MSX1 - ax170
Al Alamiah AX-350II - MSX2 - ax350
Al Alamiah AX-370 - MSX2 - ax370
*Ascii MSXPLAYer 2003
*Ascii One Chip MSX
Canon V-8 - MSX1 - canonv8
Canon V-10 - MSX1 - canonv10
Canon V-20 - MSX1 - canonv20
Canon V-20E - MSX1 - canonv20e
Canon V-20F - MSX1 - canonv20f
Canon V-20G - MSX1 - canonv20g
Canon V-20S - MSX1 - canonv20s
Canon V-25 - MSX2 - canonv25
Canon V-30 - MSX2 - canonv30
Canon V-30F - MSX2 - canonv30f
Casio MX-10 - MSX1 - mx10
Casio MX-15 - MSX1 - mx15
Casio MX-101 - MSX1 - mx101
Casio PV-7 - MSX1 - pv7
*Casio PV-7 + KB-7
Casio PV-16 - MSX1 - pv16
Ciel Expert 3 IDE - MSX2+ - expert3i
Ciel Expert 3 Turbo = MSX2+ - expert3t
======================================
This one is a full motherboard by CIEL (not an upgrade kit), created to replace the motherboard of a Gradiente Expert (which means that only the case, the analog boards and the keyboard remains Gradiente). This new motherboard has the following built-in features:

1) MSX2+
2) Support either 3.57MHz or 7.14MHz natively, switched either by software (*1) or by a hardware-switch on the front panel. Turbo-led included.
3) Up to 4MB of Memory Mapper (1MB is the most common configuration)
4) MSX-Music
5) 4 expansion slots (two external on the front panel, two internal)
6) Stereo sound (YM2413 channels 0-6 on right, PSG+YM2413 channels 7-9 on left)
7) Support the V9938 instead of the V9958 by switching some jumpers
8) The main-ram can be placed on slot 2 or slot 3, using jumpers (slot 2 is the default)


*1: A routine hidden inside the BIOS frame-0 is used to switch the turbo.

Daewoo CPC-88 - MSX1 - cpc88
Daewoo CPC-300 - MSX2 - cpc300
Daewoo CPC-300E - MSX2 - cpc300e
Daewoo CPC-330K - MSX2 - cpc330k
Daewoo CPC-400 - MSX2 - cpc400
Daewoo CPC-400S - MSX2 cpc400s
Daewoo DPC-100 - MSX1 - dpc100
Daewoo DPC-180 - MSX1 - dpc180
Daewoo DPC-200 - MSX1 - dpc200
Daewoo DPC-200E - MSX1 - dpc200e
Daewoo Zemmix CPC-50A - MSX1 - cpc50a
Daewoo Zemmix CPC-50B - MSX1 - cpc50b
Daewoo Zemmix CPC-51 - MSX1 - cpc51
Daewoo Zemmix CPC-61 - MSX2 - cpc61
Daewoo Zemmix CPG-120 Normal (no clock chip, no printer port) - MSX2 - cpg120
*Daewoo Zemmix CPG-120 Turbo - MSX2 - cpg120t
Fenner DPC-200 - MSX1 - fdpc200
Fenner FPC-500 - MSX1 - fpc500
Fenner FPC-900 - MSX2 - fpc900
Fenner SPC-800 - MSX1 - fspc800
Frael Bruc 100-1 - MSX1 - bruc100
Fujitsu FM-X - MSX1 - fmx
Goldstar FC-80U - MSX1 - gsfc80u
Goldstar FC-200 - MSX1 - gsfc200
Goldstar GFC-1080 - MSX1 - gfc1080
Goldstar GFC-1080A - MSX1 - gfc1080a
Gradiente Expert 1.0 - MSX1 - expert10
Gradiente Expert 1.1 - MSX1 - expert11
Gradiente Expert 1.3 - MSX1 - expert13
Gradiente Expert 2.0 - MSX2 - expert20
Gradiente Expert AC88+ - MSX2+ - expertac
Gradiente Expert DDPlus - MSX1 - expertdp
Gradiente Expert DDX+ - MSX2+ - expertdx
Gradiente Expert Plus - MSX1 - expertpl
*Haesung Virtual Console
*Hitachi MB-H1
*Hitachi MB-H1E
Hitachi MB-H2 - MSX1 - mbh2
*Hitachi MB-H3 - MSX2 (64KB VRAM)
*Hitachi MB-H21 - MSX1
Hitachi MB-H25 - MSX1 - mbh25
Hitachi MB-H50 - MSX1 - mbh50
Hitachi MB-H70 - MSX2 - mbh70
*Hitachi MB-H80 - MSX1
JVC HC-7GB - MSX1 -jvchc7gb
Kawai KMC-5000 - MSX2 - kmc5000
Mitsubishi ML-F48 - MSX1 - mlf48
Mitsubishi ML-F80 - MSX1 - mlf80
Mitsubishi ML-F110 - MSX1 - mlf110
Mitsubishi ML-F120 - MSX1 - mlf120
Mitsubishi ML-FX1 - MSX1 - mlfx1
Mitsubishi ML-G1 - MSX2 - mlg1
Mitsubishi ML-G3 - MSX2 - mlg3
Mitsubishi ML-G10 - MSX2 - mlg10
Mitsubishi ML-G30 Model 1 - MSX2 - mlg30
Mitsubishi ML-G30 Model 2 - MSX2 - See Mitsubishi ML-G30 Model 1
National CF-1200 - MSX1 - cf1200
National CF-2000 - MSX1 - cf2000
National CF-2700 - MSX1 - cf2700
National CF-3000 - MSX1 - cf3000
National CF-3300 - MSX1 - cf3300
National FS-1300 - MSX1 - fs1300
National FS-4000 - MSX1 - fs4000
National FS-4000 (alt) - MSX1 - fs4000a
National FS-4500 - MSX2 - fs4500
National FS-4600 - MSX2 - fs4600
National FS-4700 - MSX2 - fs4700
National FS-5000F2 - MSX2 - fs5000
National FS-5500F1 - MSX2 - fs5500
National FS-5500F2 - MSX2 - fs5500
Olympia PHC-2 - MSX1 - phc2
Olympia PHC-28 - MSX1 - phc28
Panasonic CF-2700G - MSX1 - cf2700g
Panasonic FS-A1 - MSX2 - fsa1 / fsa1a
Panasonic FS-A1 MK2 - MSX2 - fsa1mk2
Panasonic FS-A1F - MSX2 - fsa1f
Panasonic FS-A1FM - MSX2 - fsa1fm
Panasonic FS-A1FX - MSX2+ - fsa1fx
Panasonic FS-A1GT - MSX Turbo-R - fsa1gt
Panasonic FS-A1ST - MSX Turbo-R - fsa1st
Panasonic FS-A1WSX - MSX2+ - fsa1wsx
Panasonic FS-A1WX - MSX2+ - fsa1wx / fsa1wxa
Perfect Perfect1 - MSX1 - perfect1
*Perfect Perfect2 - MSX2
Philips NMS-801 - MSX1 - nms801
Philips NMS-8220 - MSX2 - nms8220 / nms8220a
Philips NMS-8245 - MSX2 - nms8245
Philips NMS-8245F - MSX2 - nms8245f
Philips NMS-8250 - MSX2 - nms8250
Philips NMS-8250F - MSX2 - nms8250f
Philips NMS-8250J - MSX2 - nms8250j
Philips NMS-8255 - MSX2 - nms8255
Philips NMS-8255F - MSX2 - nms8255f
Philips NMS-8260 - MSX2 - nms8260
Philips NMS-8270 - MSX2 - nms8270 - not confirmed to exist yet
Philips NMS-8280 - MSX2 - nms8280
Philips NMS-8280F - MSX2 - nms8280f
Philips NMS-8280G - MSX2 - nms8280g
*Philips PTC MSX PC
Philips VG-8000 - MSX1 - vg8000
Philips VG-8010 - MSX1 - vg8010
Philips VG-8010F - MSX1 - vg8010f
Philips VG-8020-00 - MSX1 - vg802000
Philips VG-8020-20 - MSX1 - vg802020
*Philips VG-8020-40 - MSX1 -
Philips VG-8020F - MSX1 - vg8020f
Philips VG-8230 - MSX2 - vg8230
Philips VG-8230J - MSX2 - vg8230j
Philips VG-8235 - MSX2 - vg8235
Philips VG-8235F - MSX2 - vg8235f
Philips VG-8240 - MSX2 - vg8240
===============================

PCB Layout missing


Pioneer PX-7 - MSX1 - piopx7
============================

|---------------------------------------|
|  CN1     CN2                          |
|                                       |
|                                       |
|  IC33                                 |---------------------------------|
|                                                      CN3                |
|   IC32   IC34            IC38  IC40                                     |
|                                                               IC20      |
|   IC15   IC18  IC43      IC8   IC35   IC6     |----IC3---|              |
|                                               |----------|    IC21      |
|   IC16   IC19  |---IC13---|    IC7    IC10                              |
|                |----------|                   IC36  IC29      ---       |
|   IC17   IC14                                     X2          | |       |
|                |--IC12---|     |----IC1-----|       IC37      |I|       |
|   IC28   IC11  |---------|     |------------|   X1            |C|       |
|                                                               |2|       |
|  |----IC4----| |----IC5----|   IC39  IC9      IC42  IC44      | |       |
|  |-----------| |-----------|                                  ---       |
|                                                                         |
|       IC45   IC31    IC30      IC41                                     |
|                                                                         |
|  CN4 CN5  CN6  CN7                                  CN8                 |
|-------------------------------------------------------------------------|

Notes:
  X1 - 3.579MHz
  X2 - 500kHz
  IC1 - Sharp LH0080A Z80A-CPU-D
  IC2 - TMS91289NL
  IC3 - MB111S112  Z10 (500kHz)
  IC4  - M5L8255AP-5
  IC5  - YM2149F
  IC6,IC7,IC8,IC10,IC45 - SN74LS367AN
  IC9 - SN74LS245N
  IC11,IC34 - SN74LS139N
  IC12 - YM2301-23908 / 53 18 85 A (might indicate a version)
  IC13 - Pioneer PD5031 2364-213 514100 (M5L2764-213)
  IC14,IC17,IC30,IC31 - SN74LS157N
  IC15-IC19 - MB81416-12
  IC20,IC21 - TMS4416-I5NL
  IC28 - SN74LS153N
  IC29 - SN74LS02N
  IC32 - SN74LS374N
  IC33 - M5218P
  IC35 - SN74LS74AN
  IC36 - SN74LS30N
  IC37-IC39 - SN74LS04N
  IC40,IC41 - SN74LS05N
  IC42 - SN74LS08N
  IC43,IC44 - SN74LS32N
  CN1 - Printer
  CN2 - Cassette recorder
  CN3 - Expansion slot
  CN4 - Keyboard
  CN5 - Keyboard
  CN6 - Controller #1
  CN7 - Controller #2
  CN8 - Expansion slot


Pioneer PX-7UK - MSX1 - piopx7uk
Pioneer PX-V60 - MSX1 - piopxv60
Samsung SPC-800 MSX1 - spc800
Sanyo MPC-64 - MSX1 - mpc64
Sanyo MPC-100 - MSX1 - mpc100
Sanyo MPC-200 - MSX1 - mpc200
Sanyo MPC-200SP - MSX1 - mpc200sp
Sanyo MPC-2300 - MSX2 - mpc2300
Sanyo MPC-2500FD - MSX2 - mpc2500f
Sanyo PHC-28L - MSX1 - phc28l
Sanyo PHC-28S - MSX1 - phc28s
Sanyo Wavy MPC-10 - MSX1 - mpc10
Sanyo Wavy MPC-25FD - MSX2 - mpc25fd
Sanyo Wavy MPC-27 - MSX2 - mpc27
Sanyo Wavy PHC-23 - MSX2 - phc23
Sanyo Wavy PHC-35J - MSX2+ - phc35j
Sanyo Wavy PHC-55FD2 - MSX2 - phc55fd2
Sanyo Wavy PHC-70FD1 - MSX2+ - phc70fd
Sanyo Wavy PHC-70FD2 - MSX2+ - phc70fd2
Sanyo Wavy PHC-77 - MSX2 - phc77
Sharp Epcom HotBit 1.1 - MSX1 - hotbit11
Sharp Epcom HotBit 1.2 - MSX1 - hotbit12
Sharp Epcom HotBit 1.3b - MSX1 - hotbi13b
Sharp Epcom HotBit 1.3p - MSX1 - hotbi13p
Sharp Epcom HotBit 2.0 - MSX2 - hotbit20
Sony HB-10 - MSX1 - hb10
*Sony HB-10D - MSX1
Sony HB-10P - MSX1 - hb10p
Sony HB-20P - MSX1 - hb20p
Sony HB-55 Version 1 - MSX1 - hb55
Sony HB-55D - MSX1 - hb55d
Sony HB-55P - MSX1 - hb55p
Sony HB-75D - MSX1 - hb75d
Sony HB-75P - MSX1 - hb75p
*Sony HB-101 - MSX1
Sony HB-101P - MSX1 - hb101p
Sony HB-201 - MSX1 - hp201
Sony HB-201P - MSX1 - hb201p
Sony HB-501P - MSX1 - hb501p
Sony HB-701FD - MSX1 - hb701fd
Sony HB-F1 - MSX2 - hbf1
Sony HB-F1II - MSX2 - hbf12
Sony HB-F1XD - MSX2 - hbf1xd
Sony HB-F1XD MK2 - MSX2 - hbf1xdm2
Sony HB-F1XDJ - MSX2+ - hbf1xdj
Sony HB-F1XV - MSX2+ - hbf1xv
Sony HB-F5 - MSX2 - hbf5
Sony HB-F500 - MSX2 - hbf500
Sony HB-F500F - MSX2 - hbf500f
Sony HB-F500P - MSX2 - hbf500p
Sony HB-F700D - MSX2 - hbf700d
Sony HB-F700F - MSX2 - hbf700f
Sony HB-F700P - MSX2 - hbf700p
Sony HB-F700S - MSX2 - hbf700s
*Sony HB-F750+
Sony HB-F900 - MSX2 - hbf900 / hbf900a
Sony HB-F9P - MSX2 - hbf9p
Sony HB-F9P Russian - MSX2 - hbf9pr
Sony HB-F9S - MSX2 - hbf9s
Sony HB-F9S+ - MSX2+ - hbf9sp
Sony HB-G900AP - MSX2 - hbg900ap
Sony HB-G900P - MSX2 - hbg900p
*Sony HB-T7
Spectravideo SVI-728 - MSX1 - svi728
Spectravideo SVI-738 - MSX1 - svi738
Spectravideo SVI-738 Arabic - MSX1 - svi738ar
Spectravideo SVI-738 Danish - MSX1 - svi738dk
Spectravideo SVI-738 Polish - MSX1 - svi738pl
Spectravideo SVI-738 Spanish - MSX1 - svi738sp
Spectravideo SVI-738 Swedish - MSX1 - svi738sw
Talent DPC-200 - MSX1 - tadpc200
Talent DPC-200A - MSX1 - tadpc20a
Talent TPC-310 - MSX2 - tpc310
Talent TPP-311 - MSX2 - tpp311
Talent TPS-312 - MSX2 - tps312
==============================

PCB Layouts missing


Toshiba HX-10 - MSX1 - hx10
===========================

Code on PCB: MSX TUK
        |---------------------------|-------------------|-------------|
        |   CN1  CN2  CN3  CN4               CN5                      |
        |                        |---------------------------|        |
        |                        |---------------------------|        |
        |                                    CN6                      |
        |                        IC40                                 |
        |                                                         CN7 |
        |                      IC38     IC32     IC33     IC37        |
        |                                                             |
        |                      Q2       IC31     IC34     IC35        |
        |    Q1                                                   CN8 |
        |                                                 IC39        |
        |   |--IC15------| |--IC2----|   |----IC1-----|               |
        |   |------------| |---------|   |------------|               |
        |                                                 IC30        |
        |                 IC3    IC4                              CN9 |
        |                               |-----IC15-------|            |
        |  IC17   IC18    IC7    IC8    |----------------|            |
        |                                                 IC27        |
        |  IC19   IC20    IC9    IC10   |----IC25------|              |
|----|  |                               |--------------|  IC26        |
| Q  |  |  IC21   IC22    IC11   IC12                                 |
|    |  |                                                             |
| S  |  |  IC23   IC24    IC13   IC14    IC29             IC28        |
|  L |  |                                                             |
|    |  |                                 CN11   CN10                 |
|----|  |-------------------------------------------------------------|

Notes:
  Mainboard components:
   IC1               - Sharp LH0080A Z80A-CPU-D
   IC2               - MB83256
   IC3,IC4,IC27,IC28 - Texas Instruments SN74LS157N
   IC7-IC14          - HM4864AP
   IC15              - Toshiba TCX-1007 (64pin custom chip)
   IC16              - 40pin chip covered with some kind of heatsink(?), probably TMS9929A
   IC17-IC24         - 4116-3
   IC25              - AY-3-8910A
   IC26              - SN74LS09N
   IC29              - HD74LS145P
   IC30-IC34         - M74LS367AP
   IC35              - MB74LS74A
   IC37              - HD74LS373P
   IC38              - Toshiba TC74HCU04P
   IC39              - HD74LS08P
   IC40              - TA75559P
   Q1                - 10687.5
   Q2                - 3579545
   CN1               - Cassette connector
   CN2               - RF connector
   CN3               - Audio connector
   CN4               - Video connector
   CN5               - Expansion connector
   CN6               - Cartridge connector
   CN7               - Printer connector
   CN8               - Joystick 2 connector
   CN9               - Joystick 1 connector
   CN10              - Keyboard connector 1
   CN11              - Keyboard connector 2

  Extra pcb (video related?) components::
   Q - 4.433619
   S - 74LS04
   L - LVA510


Toshiba HX-10D - MSX1 - hx10d
Toshiba HX-10DP - MSX1 - hx10dp
Toshiba HX-10E - MSX1 - hx10e
Toshiba HX-10F - MSX1 - hx10f
Toshiba HX-10S - MSX1 - hx10s
Toshiba HX-10SA - MSX1 - hx10sa
Toshiba HX-20 - MSX1 - hx20
Toshiba HX-20I - MSX1 - hx20i
Toshiba HX-21 - MXS1 - -hx21
Toshiba HX-21I - MSX1 - hx21i
Toshiba HX-22 - MSX1 - hx22
Toshiba HX-22I - MSX1 - hx22i
Toshiba HX-23 - MSX2 - hx23
Toshiba HX-23F - MSX2 - hx23f
Toshiba HX-23I - HSX2 - hx23i
Toshiba HX-33 - MSX2 - hx33
Toshiba HX-34 - MSX2 - hx34
Toshiba HX-34I - MSX2 - hx34i
Toshiba FS-TM1 - MSX2 - fstm1
Victor HC-5 - MSX1 - hc5
Victor HC-6 - MSX1 - hc6
*Victor HC-6AV - MSX1
Victor HC-7 - MSX1 - hc7
*Victor HC-7E - MSX1
*Victor HC-7GB - MSX1
*Victor HC-9S - MSX2
*Victor HC-80 - MSX2
Victor HC-90 - MSX2 - victhc90
Victor HC-95 - MSX2 - victhc95
Victor HC-95A - MSX2 - victhc95a
*Victor HC-95T - MSX2
*Victor HC-95V - MSX2
Yamaha CX5F-1 - MSX1 - cx5f1
Yamaha CX5F-2 - MSX1 - cx5f
Yamaha CX5M - MSX1 - cx5m
Yamaha CX5MII-128 - MSX1 - cx5m128
Yamaha CX5MII - MSX1 - cx5m2
Yamaha CX7M - MSX2 - cx7m
Yamaha CX7M/128 - MSX2 - cx7m128
Yamaha YIS-303 - MSX1 - yis303
Yamaha YIS-503 - MSX1 - yis503
Yamaha YIS-503F - MSX1 - yis503f
Yamaha YIS-503II - MSX1 - yis503ii
Yamaha YIS-503IIR (Russian) - MSX1 - y503iir
Yamaha YIS-503IIR (Estonian) - MSX1 - y503iir2
Yamaha YIS-503M - MSX1 - yis503m
Yamaha YIS-503IIIR - MSX2 - y503iiir
Yamaha YIS-503IIIR Estonian - MSX2 - y503iiire
Yamaha YIS604 - MSX2 - yis60464
Yamaha YIS604-128 - MSX2 - yis604
Yamaha YIS805-128 - MSX2 - y805128
Yamaha YIS805-128R2 - MSX2 - y805128r2
Yamaha YIS805-128R2 Estonian - MSX2 - y805128r2e
Yamaha YIS805-256 - MSX2 - y805256
*Yamaha YIS805-256 2+ - MSX2+ - was this a real machine?
Yashica YC-64 - MSX1 - yc64
Yeno DPC-64 (same bios as Olympia PHC-2)
Yeno MX64 - MSX1 - mx64
=============

PCB Layouts missing


*/


#include "emu.h"

#include "bus/centronics/ctronics.h"
#include "bus/msx_slot/slot.h"
#include "bus/msx_slot/rom.h"
#include "bus/msx_slot/ram.h"
#include "bus/msx_slot/cartridge.h"
#include "bus/msx_slot/ram_mm.h"
#include "bus/msx_slot/disk.h"
#include "bus/msx_slot/music.h"
#include "bus/msx_slot/bunsetsu.h"
#include "bus/msx_slot/fs4600.h"
#include "bus/msx_slot/panasonic08.h"
#include "bus/msx_slot/sony08.h"
#include "cpu/z80/r800.h"
#include "cpu/z80/z80.h"
#include "formats/dsk_dsk.h"
#include "formats/dmk_dsk.h"
#include "formats/fmsx_cas.h"
#include "formats/msx_dsk.h"
#include "hashfile.h"
#include "imagedev/cassette.h"
#include "imagedev/floppy.h"
#include "machine/i8255.h"
#include "machine/rp5c01.h"
#include "machine/buffer.h"
#include "machine/input_merger.h"
#include "machine/wd_fdc.h"
#include "msx_matsushita.h"
#include "msx_s1985.h"
#include "msx_switched.h"
#include "msx_systemflags.h"
#include "screen.h"
#include "softlist_dev.h"
#include "sound/ay8910.h"
#include "sound/dac.h"
#include "sound/ymopl.h"
#include "speaker.h"
#include "video/v9938.h"
#include "video/tms9928a.h"


//#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"


namespace {

class msx_state : public driver_device
{
public:
	msx_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_cassette(*this, "cassette")
		, m_ay8910(*this, "ay8910")
		, m_dac(*this, "dac")
		, m_ppi(*this, "ppi8255")
		, m_tms9928a(*this, "tms9928a")
		, m_cent_status_in(*this, "cent_status_in")
		, m_cent_ctrl_out(*this, "cent_ctrl_out")
		, m_cent_data_out(*this, "cent_data_out")
		, m_centronics(*this, "centronics")
		, m_speaker(*this, "speaker")
		, m_mainirq(*this, "mainirq")
		, m_screen(*this, "screen")
		, m_region_maincpu(*this, "maincpu")
		, m_region_kanji(*this, "kanji")
		, m_io_joy(*this, "JOY%u", 0U)
		, m_io_dsw(*this, "DSW")
		, m_io_mouse(*this, "MOUSE%u", 0U)
		, m_io_key(*this, "KEY%u", 0U)
		, m_leds(*this, "led%u", 1U)
		, m_psg_b(0)
		, m_kanji_latch(0)
		, m_empty_slot(mconfig, *this)
		, m_primary_slot(0)
		, m_port_c_old(0)
		, m_keylatch(0)
	{
		for (int prim = 0; prim < 4; prim++)
		{
			m_slot_expanded[prim] = false;
			m_secondary_slot[prim] = 0;
			for (int sec = 0; sec < 4; sec++)
			{
				for (int page = 0; page < 4; page++)
				{
					m_all_slots[prim][sec][page] = nullptr;
				}
			}
		}
		m_mouse[0] = m_mouse[1] = 0;
		m_mouse_stat[0] = m_mouse_stat[1] = 0;
		m_empty_slot.set_memory_space(m_maincpu, AS_PROGRAM);
		m_empty_slot.set_io_space(m_maincpu, AS_IO);
	}

	void hc6(machine_config &config);
	void hb75d(machine_config &config);
	void dpc100(machine_config &config);
	void hb55p(machine_config &config);
	void hotbi13p(machine_config &config);
	void fspc800(machine_config &config);
	void mpc200(machine_config &config);
	void hb201p(machine_config &config);
	void hx22i(machine_config &config);
	void svi738dk(machine_config &config);
	void gfc1080a(machine_config &config);
	void svi738pl(machine_config &config);
	void hx10dp(machine_config &config);
	void perfect1(machine_config &config);
	void mlf110(machine_config &config);
	void cf1200(machine_config &config);
	void hx20(machine_config &config);
	void cf3000(machine_config &config);
	void mpc100(machine_config &config);
	void vg8020f(machine_config &config);
	void hx10f(machine_config &config);
	void cf2000(machine_config &config);
	void expertpl(machine_config &config);
	void fs4000a(machine_config &config);
	void mpc10(machine_config &config);
	void pv16(machine_config &config);
	void jvchc7gb(machine_config &config);
	void phc28(machine_config &config);
	void hx10sa(machine_config &config);
	void cx5m128(machine_config &config);
	void expert10(machine_config &config);
	void mbh50(machine_config &config);
	void hc7(machine_config &config);
	void gfc1080(machine_config &config);
	void cpc51(machine_config &config);
	void ax150(machine_config &config);
	void phc28s(machine_config &config);
	void mlfx1(machine_config &config);
	void tadpc200(machine_config &config);
	void gsfc200(machine_config &config);
	void hx10d(machine_config &config);
	void expertdp(machine_config &config);
	void yis303(machine_config &config);
	void canonv25(machine_config &config);
	void svi738sp(machine_config &config);
	void fmx(machine_config &config);
	void phc2(machine_config &config);
	void pv7(machine_config &config);
	void hx10(machine_config &config);
	void mlf48(machine_config &config);
	void cpc50b(machine_config &config);
	void hb10p(machine_config &config);
	void hx20i(machine_config &config);
	void mx10(machine_config &config);
	void mx15(machine_config &config);
	void expert13(machine_config &config);
	void bruc100(machine_config &config);
	void hx21(machine_config &config);
	void cf3300(machine_config &config);
	void cx5f1(machine_config &config);
	void hx10e(machine_config &config);
	void dpc200(machine_config &config);
	void svi738(machine_config &config);
	void dpc200e(machine_config &config);
	void canonv10(machine_config &config);
	void yis503(machine_config &config);
	void mpc200sp(machine_config &config);
	void svi738sw(machine_config &config);
	void vg8010f(machine_config &config);
	void dpc180(machine_config &config);
	void mlf120(machine_config &config);
	void hb201(machine_config &config);
	void piopxv60(machine_config &config);
	void hb10(machine_config &config);
	void hb501p(machine_config &config);
	void cx5m(machine_config &config);
	void mx101(machine_config &config);
	void mx64(machine_config &config);
	void hb55d(machine_config &config);
	void nms801(machine_config &config);
	void svi728(machine_config &config);
	void hotbi13b(machine_config &config);
	void hotbit12(machine_config &config);
	void hotbit11(machine_config &config);
	void vg8010(machine_config &config);
	void cf2700(machine_config &config);
	void hx21i(machine_config &config);
	void mbh2(machine_config &config);
	void cx5f(machine_config &config);
	void mpc64(machine_config &config);
	void yc64(machine_config &config);
	void yis503m(machine_config &config);
	void gsfc80u(machine_config &config);
	void cf2700g(machine_config &config);
	void ax170(machine_config &config);
	void y503iir(machine_config &config);
	void svi738ar(machine_config &config);
	void yis503ii(machine_config &config);
	void yis503f(machine_config &config);
	void cx5m2(machine_config &config);
	void spc800(machine_config &config);
	void canonv20(machine_config &config);
	void hb20p(machine_config &config);
	void mbh25(machine_config &config);
	void fs4000(machine_config &config);
	void hx10s(machine_config &config);
	void piopx7uk(machine_config &config);
	void hc5(machine_config &config);
	void dgnmsx(machine_config &config);
	void fdpc200(machine_config &config);
	void hx22(machine_config &config);
	void fs1300(machine_config &config);
	void phc28l(machine_config &config);
	void hb101p(machine_config &config);
	void expert11(machine_config &config);
	void vg802020(machine_config &config);
	void tadpc20a(machine_config &config);
	void hb75p(machine_config &config);
	void piopx7(machine_config &config);
	void canonv8(machine_config &config);
	void cpc88(machine_config &config);
	void vg802000(machine_config &config);
	void mlf80(machine_config &config);
	void cpc50a(machine_config &config);
	void hb701fd(machine_config &config);
	void vg8000(machine_config &config);
	void hb55(machine_config &config);
	void y503iir2(machine_config &config);
	void fpc500(machine_config &config);

protected:
	void msx_base(machine_config &config, XTAL xtal, int cpu_divider);
	template<typename VDPType> void msx1(VDPType &vdp_type, machine_config &config, bool has_cartlist = true);

	void msx1_floplist(machine_config &config);
	void msx_fd1793(machine_config &config);
	void msx_wd2793_force_ready(machine_config &config);
	void msx_wd2793(machine_config &config);
	void msx_mb8877a(machine_config &config);
	void msx_tc8566af(machine_config &config);
	void msx_microsol(machine_config &config);
	void msx_1_35_ssdd_drive(machine_config &config);
	void msx_1_35_dd_drive(machine_config &config);
	void msx_2_35_dd_drive(machine_config &config);
	template <uint8_t Game_port>
	uint8_t game_port_r();

	// configuration helpers
	void install_slot_pages(uint8_t prim, uint8_t sec, uint8_t page, uint8_t numpages, msx_internal_slot_interface &device);
	template <typename T, typename U>
	auto &add_internal_slot(machine_config &config, T &&type, U &&tag, uint8_t prim, uint8_t sec, uint8_t page, uint8_t numpages)
	{
		auto &device(std::forward<T>(type)(config, std::forward<U>(tag), 0U));
		device.set_memory_space(m_maincpu, AS_PROGRAM);
		device.set_io_space(m_maincpu, AS_IO);
		device.set_start_address(page * 0x4000);
		device.set_size(numpages * 0x4000);
		install_slot_pages(prim, sec, page, numpages, device);
		return device;
	}
	template <typename T, typename U>
	auto &add_internal_slot(machine_config &config, T &&type, U &&tag, uint8_t prim, uint8_t sec, uint8_t page, uint8_t numpages, const char *region, uint32_t offset)
	{
		auto &device(std::forward<T>(type)(config, std::forward<U>(tag), 0U));
		device.set_memory_space(m_maincpu, AS_PROGRAM);
		device.set_io_space(m_maincpu, AS_IO);
		device.set_start_address(page * 0x4000);
		device.set_size(numpages * 0x4000);
		device.set_rom_start(region, offset);
		install_slot_pages(prim, sec, page, numpages, device);
		return device;
	}
	template <typename T, typename U>
	auto &add_internal_slot_mirrored(machine_config &config, T &&type, U &&tag, uint8_t prim, uint8_t sec, uint8_t page, uint8_t numpages, const char *region, uint32_t offset)
	{
		// Memory mapped FDC registers are also accessible through page 2
		auto &device(type(config, std::forward<U>(tag), 0U));
		device.set_memory_space(m_maincpu, AS_PROGRAM);
		device.set_io_space(m_maincpu, AS_IO);
		device.set_start_address(page * 0x4000);
		device.set_size(0x4000);
		device.set_rom_start(region, offset);
		install_slot_pages(prim, sec, page, numpages, device);
		return device;
	}
	template <int N, typename T, typename U, typename V>
	auto &add_cartridge_slot(machine_config &config, T &&type, U &&tag, uint8_t prim, uint8_t sec, V &&intf, const char *deft)
	{
		auto &device(type(config, std::forward<U>(tag), 0U));
		device.set_memory_space(m_maincpu, AS_PROGRAM);
		device.set_io_space(m_maincpu, AS_IO);
		device.option_reset();
		intf(device);
		device.set_default_option(deft);
		device.set_fixed(false);
		device.irq_handler().set("mainirq", FUNC(input_merger_device::in_w<N>));
		install_slot_pages(prim, sec, 0, 4, device);
		return device;
	}

	virtual void driver_start() override;
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void device_post_load() override;

	void sec_slot_w(uint8_t data);
	uint8_t sec_slot_r();
	uint8_t kanji_r(offs_t offset);
	void kanji_w(offs_t offset, uint8_t data);
	void ppi_port_a_w(uint8_t data);
	void ppi_port_c_w(uint8_t data);
	uint8_t ppi_port_b_r();
	uint8_t mem_read(offs_t offset);
	void mem_write(offs_t offset, uint8_t data);
	uint8_t psg_port_a_r();
	uint8_t psg_port_b_r();
	void psg_port_a_w(uint8_t data);
	void psg_port_b_w(uint8_t data);

	void msx_io_map(address_map &map);
	void memory_map(address_map &map);

	required_device<z80_device> m_maincpu;
	required_device<cassette_image_device> m_cassette;
	required_device<ay8910_device> m_ay8910;
	required_device<dac_bit_interface> m_dac;
	required_device<i8255_device> m_ppi;
	optional_device<tms9928a_device> m_tms9928a;
	required_device<input_buffer_device> m_cent_status_in;
	required_device<output_latch_device> m_cent_ctrl_out;
	required_device<output_latch_device> m_cent_data_out;
	required_device<centronics_device> m_centronics;
	required_device<speaker_device> m_speaker;
	required_device<input_merger_any_high_device> m_mainirq;
	required_device<screen_device> m_screen;
	required_memory_region m_region_maincpu;
	optional_memory_region m_region_kanji;
	required_ioport_array<2> m_io_joy;
	required_ioport m_io_dsw;
	required_ioport_array<2> m_io_mouse;
	required_ioport_array<6> m_io_key;
	output_finder<2> m_leds;

	static constexpr bool HAS_CARTLIST = true;
	static constexpr bool NO_CARTLIST = false;

private:
	void memory_map_all();
	void memory_map_page(uint8_t page);
	void memory_reset();
	void memory_init();

	static void floppy_formats(format_registration &fr);

	INTERRUPT_GEN_MEMBER(msx_interrupt);

	// PSG
	uint8_t m_psg_b = 0;
	// mouse
	uint16_t m_mouse[2]{};
	int8_t m_mouse_stat[2]{};
	// kanji
	uint32_t m_kanji_latch = 0;
	// memory
	msx_internal_slot_interface m_empty_slot;
	msx_internal_slot_interface *m_all_slots[4][4][4]{};
	msx_internal_slot_interface *m_current_page[4]{};
	bool m_slot_expanded[4]{};
	uint8_t m_primary_slot = 0;
	uint8_t m_secondary_slot[4]{};
	uint8_t m_port_c_old = 0;
	uint8_t m_keylatch = 0;
};


class msx2_state : public msx_state
{
public:
	msx2_state(const machine_config &mconfig, device_type type, const char *tag)
		: msx_state(mconfig, type, tag)
		, m_v9938(*this, "v9938")
		, m_v9958(*this, "v9958")
		, m_rtc(*this, "rtc")
		, m_rtc_latch(0)
	{
	}

	void ax350(machine_config &config);
	void ax370(machine_config &config);
	void canonv25(machine_config &config);
	void canonv30(machine_config &config);
	void canonv30f(machine_config &config);
	void cpc300(machine_config &config);
	void cpc300e(machine_config &config);
	void cpc330k(machine_config &config);
	void cpc400(machine_config &config);
	void cpc400s(machine_config &config);
	void cpc61(machine_config &config);
	void cpg120(machine_config &config);
	void fpc900(machine_config &config);
	void expert20(machine_config &config);
	void mbh70(machine_config &config);
	void kmc5000(machine_config &config);
	void mlg1(machine_config &config);
	void mlg3(machine_config &config);
	void mlg10(machine_config &config);
	void mlg30(machine_config &config);
	void fs4500(machine_config &config);
	void fs4600(machine_config &config);
	void fs4700(machine_config &config);
	void fs5000(machine_config &config);
	void fs5500f1(machine_config &config);
	void fs5500f2(machine_config &config);
	void fsa1(machine_config &config);
	void fsa1a(machine_config &config);
	void fsa1f(machine_config &config);
	void fsa1fm(machine_config &config);
	void fsa1mk2(machine_config &config);
	void nms8220(machine_config &config);
	void nms8220a(machine_config &config);
	void nms8245(machine_config &config);
	void nms8245f(machine_config &config);
	void nms8250(machine_config &config);
	void nms8250f(machine_config &config);
	void nms8250j(machine_config &config);
	void nms8255(machine_config &config);
	void nms8255f(machine_config &config);
	void nms8260(machine_config &config);
	void nms8280(machine_config &config);
	void nms8280f(machine_config &config);
	void nms8280g(machine_config &config);
	void vg8230(machine_config &config);
	void vg8230j(machine_config &config);
	void vg8235(machine_config &config);
	void vg8235f(machine_config &config);
	void vg8240(machine_config &config);
	void mpc2300(machine_config &config);
	void mpc2500f(machine_config &config);
	void mpc25fd(machine_config &config);
	void mpc27(machine_config &config);
	void phc23(machine_config &config);
	void phc55fd2(machine_config &config);
	void phc77(machine_config &config);
	void hotbit20(machine_config &config);
	void hbf1(machine_config &config);
	void hbf12(machine_config &config);
	void hbf1xd(machine_config &config);
	void hbf1xdm2(machine_config &config);
	void hbf5(machine_config &config);
	void hbf500(machine_config &config);
	void hbf500f(machine_config &config);
	void hbf500p(machine_config &config);
	void hbf700d(machine_config &config);
	void hbf700f(machine_config &config);
	void hbf700p(machine_config &config);
	void hbf700s(machine_config &config);
	void hbf900(machine_config &config);
	void hbf900a(machine_config &config);
	void hbf9p(machine_config &config);
	void hbf9pr(machine_config &config);
	void hbf9s(machine_config &config);
	void hbg900ap(machine_config &config);
	void hbg900p(machine_config &config);
	void tpc310(machine_config &config);
	void tpp311(machine_config &config);
	void tps312(machine_config &config);
	void hx23(machine_config &config);
	void hx23f(machine_config &config);
	void hx23i(machine_config &config);
	void hx33(machine_config &config);
	void hx34(machine_config &config);
	void hx34i(machine_config &config);
	void fstm1(machine_config &config);
	void victhc90(machine_config &config);
	void victhc95(machine_config &config);
	void victhc95a(machine_config &config);
	void cx7m(machine_config &config);
	void cx7m128(machine_config &config);
	void y503iiir(machine_config &config);
	void y503iiire(machine_config &config);
	void yis60464(machine_config &config);
	void yis604(machine_config &config);
	void y805128(machine_config &config);
	void y805128r2(machine_config &config);
	void y805128r2e(machine_config &config);
	void y805256(machine_config &config);
	void expert3i(machine_config &config);
	void expert3t(machine_config &config);
	void expertac(machine_config &config);
	void expertdx(machine_config &config);
	void fsa1fx(machine_config &config);
	void fsa1wsx(machine_config &config);
	void fsa1wx(machine_config &config);
	void fsa1wxa(machine_config &config);
	void phc35j(machine_config &config);
	void phc70fd(machine_config &config);
	void phc70fd2(machine_config &config);
	void hbf1xdj(machine_config &config);
	void hbf1xv(machine_config &config);
	void hbf9sp(machine_config &config);
	void fsa1gt(machine_config &config);
	void fsa1st(machine_config &config);

protected:
	virtual void machine_start() override;

private:
	void msx2_base(machine_config &config, bool has_cartlist = true);
	void msx2(machine_config &config, bool has_cartlist = true);
	void msx2_pal(machine_config &config, bool has_cartlist = true);
	void msx2plus(machine_config &config);
	void turbor(machine_config &config);

	void msx2_floplist(machine_config &config);
	void msx2plus_floplist(machine_config &config);
	void msxr_floplist(machine_config &config);
	void msx_ym2413(machine_config &config);
	void msx2_64kb_vram(machine_config &config);

	uint8_t rtc_reg_r();
	void rtc_reg_w(uint8_t data);
	void rtc_latch_w(uint8_t data);
	uint8_t switched_r(offs_t offset);
	void switched_w(offs_t offset, uint8_t data);
	DECLARE_WRITE_LINE_MEMBER(turbo_w);

	void msx2_io_map(address_map &map);
	void msx2plus_io_map(address_map &map);

	std::vector<msx_switched_interface *> m_switched;

	optional_device<v9938_device> m_v9938;
	optional_device<v9958_device> m_v9958;
	required_device<rp5c01_device> m_rtc;

	// rtc
	uint8_t m_rtc_latch = 0;
};


void msx_state::memory_map(address_map &map)
{
	map(0x0000, 0xfffe).rw(FUNC(msx_state::mem_read), FUNC(msx_state::mem_write));
	map(0xffff, 0xffff).rw(FUNC(msx_state::sec_slot_r), FUNC(msx_state::sec_slot_w));
}


void msx_state::msx_io_map(address_map &map)
{
	map.unmap_value_high();
	map.global_mask(0xff);
	// 0x7c - 0x7d : MSX-MUSIC/FM-PAC write port. Handlers will be installed if MSX-MUSIC is present in a system
	map(0x90, 0x90).r(m_cent_status_in, FUNC(input_buffer_device::read));
	map(0x90, 0x90).w(m_cent_ctrl_out, FUNC(output_latch_device::write));
	map(0x91, 0x91).w(m_cent_data_out, FUNC(output_latch_device::write));
	map(0xa0, 0xa7).rw(m_ay8910, FUNC(ay8910_device::data_r), FUNC(ay8910_device::address_data_w));
	map(0xa8, 0xab).rw(m_ppi, FUNC(i8255_device::read), FUNC(i8255_device::write));
	map(0x98, 0x99).rw(m_tms9928a, FUNC(tms9928a_device::read), FUNC(tms9928a_device::write));
	map(0xd8, 0xd9).w(FUNC(msx_state::kanji_w));
	map(0xd9, 0xd9).r(FUNC(msx_state::kanji_r));
	// 0xfc - 0xff : Memory mapper I/O ports. I/O handlers will be installed if a memory mapper is present in a system
}


void msx2_state::msx2_io_map(address_map &map)
{
	map.unmap_value_high();
	map.global_mask(0xff);
	map(0x40, 0x4f).rw(FUNC(msx2_state::switched_r), FUNC(msx2_state::switched_w));
	// 0x7c - 0x7d : MSX-MUSIC/FM-PAC write port. Handlers will be installed if MSX-MUSIC is present in a system
	map(0x90, 0x90).r(m_cent_status_in, FUNC(input_buffer_device::read));
	map(0x90, 0x90).w(m_cent_ctrl_out, FUNC(output_latch_device::write));
	map(0x91, 0x91).w(m_cent_data_out, FUNC(output_latch_device::write));
	map(0xa0, 0xa7).rw(m_ay8910, FUNC(ay8910_device::data_r), FUNC(ay8910_device::address_data_w));
	map(0xa8, 0xab).rw(m_ppi, FUNC(i8255_device::read), FUNC(i8255_device::write));
	map(0x98, 0x9b).rw(m_v9938, FUNC(v9938_device::read), FUNC(v9938_device::write));
	map(0xb4, 0xb4).w(FUNC(msx2_state::rtc_latch_w));
	map(0xb5, 0xb5).rw(FUNC(msx2_state::rtc_reg_r), FUNC(msx2_state::rtc_reg_w));
	map(0xd8, 0xd9).w(FUNC(msx2_state::kanji_w));
	map(0xd9, 0xd9).r(FUNC(msx2_state::kanji_r));
	// 0xfc - 0xff : Memory mapper I/O ports. I/O handlers will be installed if a memory mapper is present in a system
}


void msx2_state::msx2plus_io_map(address_map &map)
{
	map.unmap_value_high();
	map.global_mask(0xff);
	map(0x40, 0x4f).rw(FUNC(msx2_state::switched_r), FUNC(msx2_state::switched_w));
	// 0x7c - 0x7d : MSX-MUSIC/FM-PAC write port. Handlers will be installed if MSX-MUSIC is present in a system
	map(0x90, 0x90).r(m_cent_status_in, FUNC(input_buffer_device::read));
	map(0x90, 0x90).w(m_cent_ctrl_out, FUNC(output_latch_device::write));
	map(0x91, 0x91).w(m_cent_data_out, FUNC(output_latch_device::write));
	map(0xa0, 0xa7).rw(m_ay8910, FUNC(ay8910_device::data_r), FUNC(ay8910_device::address_data_w));
	map(0xa8, 0xab).rw(m_ppi, FUNC(i8255_device::read), FUNC(i8255_device::write));
	map(0x98, 0x9b).rw(m_v9958, FUNC(v9958_device::read), FUNC(v9958_device::write));
	map(0xb4, 0xb4).w(FUNC(msx2_state::rtc_latch_w));
	map(0xb5, 0xb5).rw(FUNC(msx2_state::rtc_reg_r), FUNC(msx2_state::rtc_reg_w));
	map(0xd8, 0xd9).w(FUNC(msx2_state::kanji_w));
	map(0xd9, 0xd9).r(FUNC(msx2_state::kanji_r));
	// 0xfc - 0xff : Memory mapper I/O ports. I/O handlers will be installed if a memory mapper is present in a system
}


void msx_state::machine_reset()
{
	memory_reset();
	memory_map_all();
}


void msx_state::machine_start()
{
	m_leds.resolve();
	m_port_c_old = 0xff;
}


void msx2_state::machine_start()
{
	msx_state::machine_start();

	for (msx_switched_interface &switched : device_interface_enumerator<msx_switched_interface>(*this))
		m_switched.push_back(&switched);

	save_item(NAME(m_rtc_latch));
}

/* A hack to add 1 wait cycle in each opcode fetch.
   Possibly worth not to use custom table at all but adjust desired icount
   directly in m_opcodes.read_byte handler. */
static const uint8_t cc_op[0x100] = {
	4+1,10+1, 7+1, 6+1, 4+1, 4+1, 7+1, 4+1, 4+1,11+1, 7+1, 6+1, 4+1, 4+1, 7+1, 4+1,
	8+1,10+1, 7+1, 6+1, 4+1, 4+1, 7+1, 4+1,12+1,11+1, 7+1, 6+1, 4+1, 4+1, 7+1, 4+1,
	7+1,10+1,16+1, 6+1, 4+1, 4+1, 7+1, 4+1, 7+1,11+1,16+1, 6+1, 4+1, 4+1, 7+1, 4+1,
	7+1,10+1,13+1, 6+1,11+1,11+1,10+1, 4+1, 7+1,11+1,13+1, 6+1, 4+1, 4+1, 7+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1,
	7+1, 7+1, 7+1, 7+1, 7+1, 7+1, 4+1, 7+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 7+1, 4+1,
	5+1,10+1,10+1,10+1,10+1,11+1, 7+1,11+1, 5+1,10+1,10+1, 4+1,10+1,17+1, 7+1,11+1,
	5+1,10+1,10+1,11+1,10+1,11+1, 7+1,11+1, 5+1, 4+1,10+1,11+1,10+1, 4+1, 7+1,11+1,
	5+1,10+1,10+1,19+1,10+1,11+1, 7+1,11+1, 5+1, 4+1,10+1, 4+1,10+1, 4+1, 7+1,11+1,
	5+1,10+1,10+1, 4+1,10+1,11+1, 7+1,11+1, 5+1, 6+1,10+1, 4+1,10+1, 4+1, 7+1,11+1
};

static const uint8_t cc_cb[0x100] = {
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 8+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 8+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 8+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 8+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 8+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 8+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 8+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 8+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1,
	4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,11+1, 4+1
};

static const uint8_t cc_ed[0x100] = {
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,
	 8+1, 8+1,15+2,16+1, 4+1,14+2, 4+1, 5+1, 8+1, 8+1,15+2,16+1, 4+1,14+2, 4+1, 5+1,
	 8+1, 8+1,15+2,16+1, 4+1,14+2, 4+1, 5+1, 8+1, 8+1,15+2,16+1, 4+1,14+2, 4+1, 5+1,
	 8+1, 8+1,15+2,16+1, 4+1,14+2, 4+1,14+1, 8+1, 8+1,15+2,16+1, 4+1,14+2, 4+1,14+1,
	 8+1, 8+1,15+2,16+1, 4+1,14+2, 4+1, 4+1, 8+1, 8+1,15+2,16+1, 4+1,14+2, 4+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,
	12+1,12+1,12+1,12+1, 4+1, 4+1, 4+1, 4+1,12+1,12+1,12+1,12+1, 4+1, 4+1, 4+1, 4+1,
	12+1,12+1,12+1,12+1, 4+1, 4+1, 4+1, 4+1,12+1,12+1,12+1,12+1, 4+1, 4+1, 4+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1
};

static const uint8_t cc_xy[0x100] = {
	 4+1,10+1, 7+1, 6+1, 4+1, 4+1, 7+1, 4+1, 4+1,11+1, 7+1, 6+1, 4+1, 4+1, 7+1, 4+1,
	 8+1,10+1, 7+1, 6+1, 4+1, 4+1, 7+1, 4+1,12+1,11+1, 7+1, 6+1, 4+1, 4+1, 7+1, 4+1,
	 7+1,10+1,16+1, 6+1, 4+1, 4+1, 7+1, 4+1, 7+1,11+1,16+1, 6+1, 4+1, 4+1, 7+1, 4+1,
	 7+1,10+1,13+1, 6+1,19+1,19+1,15+1, 4+1, 7+1,11+1,13+1, 6+1, 4+1, 4+1, 7+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1,
	15+1,15+1,15+1,15+1,15+1,15+1, 4+1,15+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1,
	 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1, 4+1,15+1, 4+1,
	 5+1,10+1,10+1,10+1,10+1,11+1, 7+1,11+1, 5+1,10+1,10+1, 7+1,10+1,17+1, 7+1,11+1,
	 5+1,10+1,10+1,11+1,10+1,11+1, 7+1,11+1, 5+1, 4+1,10+1,11+1,10+1, 4+1, 7+1,11+1,
	 5+1,10+1,10+1,19+1,10+1,11+1, 7+1,11+1, 5+1, 4+1,10+1, 4+1,10+1, 4+1, 7+1,11+1,
	 5+1,10+1,10+1, 4+1,10+1,11+1, 7+1,11+1, 5+1, 6+1,10+1, 4+1,10+1, 4+1, 7+1,11+1
};

/* extra cycles if jr/jp/call taken and 'interrupt latency' on rst 0-7 */
static const uint8_t cc_ex[0x100] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* DJNZ */
	5, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, /* JR NZ/JR Z */
	5, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, /* JR NC/JR C */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	5, 5, 5, 5, 0, 0, 0, 0, 5, 5, 5, 5, 0, 0, 0, 0, /* LDIR/CPIR/INIR/OTIR LDDR/CPDR/INDR/OTDR */
	6, 0, 0, 0, 7, 0, 0, 2, 6, 0, 0, 0, 7, 0, 0, 2,
	6, 0, 0, 0, 7, 0, 0, 2, 6, 0, 0, 0, 7, 0, 0, 2,
	6, 0, 0, 0, 7, 0, 0, 2, 6, 0, 0, 0, 7, 0, 0, 2,
	6, 0, 0, 0, 7, 0, 0, 2, 6, 0, 0, 0, 7, 0, 0, 2+1
};

void msx_state::driver_start()
{
	m_maincpu->set_input_line_vector(0, 0xff); // Z80

	memory_init();

	m_maincpu->z80_set_cycle_tables(cc_op, cc_cb, cc_ed, cc_xy, nullptr, cc_ex);

	save_item(NAME(m_psg_b));
	save_item(NAME(m_mouse));
	save_item(NAME(m_mouse_stat));
	save_item(NAME(m_kanji_latch));
	save_item(NAME(m_slot_expanded));
	save_item(NAME(m_primary_slot));
	save_item(NAME(m_secondary_slot));
	save_item(NAME(m_port_c_old));
	save_item(NAME(m_keylatch));
}

void msx_state::device_post_load()
{
	for (int page = 0; page < 4; page++)
	{
		int slot_primary = (m_primary_slot >> (page * 2)) & 3;
		int slot_secondary = (m_secondary_slot[slot_primary] >> (page * 2)) & 3;

		m_current_page[page] = m_all_slots[slot_primary][slot_secondary][page];
	}
}

INTERRUPT_GEN_MEMBER(msx_state::msx_interrupt)
{
	m_mouse[0] = m_io_mouse[0]->read();
	m_mouse_stat[0] = -1;
	m_mouse[1] = m_io_mouse[1]->read();
	m_mouse_stat[1] = -1;
}

/*
** The I/O functions
*/


template <uint8_t Game_port>
uint8_t msx_state::game_port_r()
{
	uint8_t inp = m_io_joy[Game_port]->read();
	if (!(inp & 0x80))
	{
		// joystick
		return (inp & 0x7f);
	}
	else
	{
		// mouse
		uint8_t data = (inp & 0x70);
		if (m_mouse_stat[Game_port] < 0)
			data |= 0xf;
		else
			data |= ~(m_mouse[Game_port] >> (4 * m_mouse_stat[Game_port])) & 0x0f;
		return data;
	}
}

uint8_t msx_state::psg_port_a_r()
{
	uint8_t data = (m_cassette->input() > 0.0038 ? 0x80 : 0);

	if ((m_psg_b ^ m_io_dsw->read()) & 0x40)
	{
		// game port 2
		data |= game_port_r<1>();
	}
	else
	{
		// game port 1
		data |= game_port_r<0>();
	}

	return data;
}

uint8_t msx_state::psg_port_b_r()
{
	return m_psg_b;
}

void msx_state::psg_port_a_w(uint8_t data)
{
}

void msx_state::psg_port_b_w(uint8_t data)
{
	// Arabic or kana mode led
	if ((data ^ m_psg_b) & 0x80)
		m_leds[1] = BIT(~data, 7);

	if ((m_psg_b ^ data) & 0x10)
	{
		if (++m_mouse_stat[0] > 3)
			m_mouse_stat[0] = -1;
	}
	if ((m_psg_b ^ data) & 0x20)
	{
		if (++m_mouse_stat[1] > 3)
			m_mouse_stat[1] = -1;
	}

	m_psg_b = data;
}


/*
** RTC functions
*/

void msx2_state::rtc_latch_w(uint8_t data)
{
	m_rtc_latch = data & 15;
}

void msx2_state::rtc_reg_w(uint8_t data)
{
	m_rtc->write(m_rtc_latch, data);
}

uint8_t msx2_state::rtc_reg_r()
{
	return m_rtc->read(m_rtc_latch);
}


/*
** The PPI functions
*/

void msx_state::ppi_port_a_w(uint8_t data)
{
	m_primary_slot = data;

	LOG("write to primary slot select: %02x\n", m_primary_slot);
	memory_map_all();
}

void msx_state::ppi_port_c_w(uint8_t data)
{
	m_keylatch = data & 0x0f;

	// caps lock
	if (BIT(m_port_c_old ^ data, 6))
		m_leds[0] = BIT(~data, 6);

	// key click
	if (BIT(m_port_c_old ^ data, 7))
		m_dac->write(BIT(data, 7));

	// cassette motor on/off
	if (BIT(m_port_c_old ^ data, 4))
		m_cassette->change_state(BIT(data, 4) ? CASSETTE_MOTOR_DISABLED : CASSETTE_MOTOR_ENABLED, CASSETTE_MASK_MOTOR);

	// cassette signal write
	if (BIT(m_port_c_old ^ data, 5))
		m_cassette->output(BIT(data, 5) ? -1.0 : 1.0);

	m_port_c_old = data;
}

uint8_t msx_state::ppi_port_b_r()
{
	uint8_t result = 0xff;

	if (m_keylatch <= 10)
	{
		uint16_t data = m_io_key[m_keylatch / 2]->read();

		if (BIT(m_keylatch, 0))
			data >>= 8;
		result = data & 0xff;
	}
	return result;
}

/************************************************************************
 *
 * New memory emulation !!
 *
 ***********************************************************************/

void msx_state::install_slot_pages(uint8_t prim, uint8_t sec, uint8_t page, uint8_t numpages, msx_internal_slot_interface &device)
{
	for (int i = page; i < std::min(page + numpages, 4); i++)
	{
		m_all_slots[prim][sec][i] = &device;
	}
	if (sec)
	{
		m_slot_expanded[prim] = true;
	}
}

void msx_state::memory_init()
{
	int count_populated_pages = 0;

	// Populate all unpopulated slots with the dummy interface
	for (auto & elem : m_all_slots)
	{
		for (int sec = 0; sec < 4; sec++)
		{
			for (int page = 0; page < 4; page++)
			{
				if (elem[sec][page] == nullptr)
				{
					elem[sec][page] = &m_empty_slot;
				}
				else
				{
					count_populated_pages++;
				}
			}
		}
	}

	if (count_populated_pages == 0) {
		fatalerror("No msx slot layout defined for this system!\n");
	}
}

void msx_state::memory_reset()
{
	m_primary_slot = 0;

	for (auto & elem : m_secondary_slot)
	{
		elem = 0;
	}
}

void msx_state::memory_map_page(uint8_t page)
{
	int slot_primary = (m_primary_slot >> (page * 2)) & 3;
	int slot_secondary = (m_secondary_slot[slot_primary] >> (page * 2)) & 3;

	m_current_page[page] = m_all_slots[slot_primary][slot_secondary][page];
}

void msx_state::memory_map_all()
{
	for (uint8_t i = 0; i < 4; i++)
		memory_map_page(i);
}

uint8_t msx_state::mem_read(offs_t offset)
{
	return m_current_page[offset >> 14]->read(offset);
}

void msx_state::mem_write(offs_t offset, uint8_t data)
{
	m_current_page[offset >> 14]->write(offset, data);
}

void msx_state::sec_slot_w(uint8_t data)
{
	int slot = m_primary_slot >> 6;
	if (m_slot_expanded[slot])
	{
		LOG("write to secondary slot %d select: %02x\n", slot, data);

		m_secondary_slot[slot] = data;
		memory_map_all();
	}
	else
		m_current_page[3]->write(0xffff, data);
}

uint8_t msx_state::sec_slot_r()
{
	int slot = m_primary_slot >> 6;

	if (m_slot_expanded[slot])
	{
		return ~m_secondary_slot[slot];
	}
	else
	{
		return m_current_page[3]->read(0xffff);
	}
}

uint8_t msx_state::kanji_r(offs_t offset)
{
	uint8_t result = 0xff;

	if (m_region_kanji)
	{
		uint32_t latch = m_kanji_latch;
		result = m_region_kanji->as_u8(latch++);

		m_kanji_latch &= ~0x1f;
		m_kanji_latch |= latch & 0x1f;
	}
	return result;
}

void msx_state::kanji_w(offs_t offset, uint8_t data)
{
	if (offset)
		m_kanji_latch = (m_kanji_latch & 0x007E0) | ((data & 0x3f) << 11);
	else
		m_kanji_latch = (m_kanji_latch & 0x1f800) | ((data & 0x3f) << 5);
}

uint8_t msx2_state::switched_r(offs_t offset)
{
	uint8_t data = 0xff;

	for (int i = 0; i < m_switched.size(); i++)
	{
		data &= m_switched[i]->switched_read(offset);
	}

	return data;
}

void msx2_state::switched_w(offs_t offset, uint8_t data)
{
	for (int i = 0; i < m_switched.size(); i++)
	{
		m_switched[i]->switched_write(offset, data);
	}
}


static INPUT_PORTS_START(msx_dips)
	PORT_START("JOY0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_BUTTON1)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_BUTTON2)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_DIPNAME(0x80, 0, "Game port 1")
	PORT_DIPSETTING(0x00, DEF_STR(Joystick))
	PORT_DIPSETTING(0x80, "Mouse")

	PORT_START("JOY1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_PLAYER(2)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_PLAYER(2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_PLAYER(2)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_PLAYER(2)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_BUTTON1)        PORT_PLAYER(2)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_BUTTON2)        PORT_PLAYER(2)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_DIPNAME(0x80, 0, "Game port 2")
	PORT_DIPSETTING(0x00, DEF_STR(Joystick))
	PORT_DIPSETTING(0x80, "Mouse")

	PORT_START("DSW")
	PORT_DIPNAME(0x40, 0, "Swap game port 1 and 2")
	PORT_DIPSETTING(0, DEF_STR(No))
	PORT_DIPSETTING(0x40, DEF_STR(Yes))

	PORT_START("MOUSE0")
	PORT_BIT(0xff00, 0x00, IPT_TRACKBALL_X) PORT_SENSITIVITY(100) PORT_KEYDELTA(0) PORT_PLAYER(1)
	PORT_BIT(0x00ff, 0x00, IPT_TRACKBALL_Y) PORT_SENSITIVITY(100) PORT_KEYDELTA(0) PORT_PLAYER(1)

	PORT_START("MOUSE1")
	PORT_BIT(0xff00, 0x00, IPT_TRACKBALL_X) PORT_SENSITIVITY(100) PORT_KEYDELTA(0) PORT_PLAYER(2)
	PORT_BIT(0x00ff, 0x00, IPT_TRACKBALL_Y) PORT_SENSITIVITY(100) PORT_KEYDELTA(0) PORT_PLAYER(2)
INPUT_PORTS_END


/* 2008-05 FP: About keyboards

Even if some later Philips (and maybe others) models started to use a layout similar to current
PC keyboards, common MSX keyboards have a couple of keys which do not fit usual mapping
- the key in the 1st row before 'Backspace', 3rd key from '0', here re-mapped to KEYCODE_BACKSLASH2
- the last key in the 4th row, 4th key from 'M' (not counting Shift), here re-mapped to KEYCODE_TILDE

These keys corresponds to the following symbols

    input_port  | msx   | msxuk | msxjp | msxkr |hotbit |expert |
    -------------------------------------------------------------
    BACKSLASH2  |  \ |  |  \ |  |  ? |  | won | |  \ ^  |  { }  |
    -------------------------------------------------------------
    TILDE       |  DK*  |  DK*  |  _    |  _    |  < >  |  / ?  |

* DK = "Dead Key"
Notice that 'expert' input_ports covers both versions 1.0 and 1.1.
msx2 input_ports have the same symbols as their msx counterparts.

TO DO:
- check Expert 1.0 layout with the real thing
- check Korean layout
- fix natural support in systems using msx inputs but with different mapping
(these systems could have different uses for keys mapped at the following
locations: COLON, QUOTE, BACKSLASH, OPENBRACE, CLOSEBRACE, BACKSLASH2, TILDE.
The corresponding symbols would not work properly in -natural mode).

Additional note about natural keyboard support: currently,
- "Graph" is mapped to 'F6' (this key could be labeled "L Graph")
- "Code" is mapped to 'F7' (this key could be labeled "R Graph", "Kana" or "Hangul")
- "Stop" is mapped to 'F8'
- "Select" is mapped to 'F9'
*/

#define KEYB_ROW0   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_CHAR('0') PORT_CHAR(')') \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!') \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('@') \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#') \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$') \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%') \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('^') \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('&')

#define KEYB_EXPERT11_ROW0   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_CHAR('0') PORT_CHAR(')') \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!') \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('"') \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#') \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$') \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%') \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('^') \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('&')

#define KEYB_HOTBIT_ROW0     \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_CHAR('0') PORT_CHAR(')') \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!') \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('@') \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#') \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$') \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%') \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('"') \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('&')

#define KEYB_ROW1   \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)          PORT_CHAR('8')  PORT_CHAR('*') \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)          PORT_CHAR('9')  PORT_CHAR('(') \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)      PORT_CHAR('-')  PORT_CHAR('_') \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS)     PORT_CHAR('=')  PORT_CHAR('+') \
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH2) PORT_CHAR('\\') PORT_CHAR('|') \
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('[')  PORT_CHAR('{') \
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(']')  PORT_CHAR('}') \
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON)      PORT_CHAR(';')  PORT_CHAR(':')

#define KEYB_HOTBIT_ROW1     \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)          PORT_CHAR('8')  PORT_CHAR('*')  \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)          PORT_CHAR('9')  PORT_CHAR('(')  \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)      PORT_CHAR('-')  PORT_CHAR('_')  \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS)     PORT_CHAR('=')  PORT_CHAR('+')  \
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH2) PORT_CHAR('\\') PORT_CHAR('^')  \
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('\'') PORT_CHAR('`')  \
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR('"')  PORT_CHAR('`')  \
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON)      PORT_CHAR(0xC7) PORT_CHAR(0xE7)

#define KEYB_EXPERT11_ROW1   \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)          PORT_CHAR('8')  PORT_CHAR('\'') \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)          PORT_CHAR('9')  PORT_CHAR('(')  \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)      PORT_CHAR('-')  PORT_CHAR('_')  \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS)     PORT_CHAR('=')  PORT_CHAR('+')  \
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH2) PORT_CHAR('{')  PORT_CHAR('}')  \
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('\'') PORT_CHAR('`')  \
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR('[')  PORT_CHAR(']')  \
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE)      PORT_CHAR('~')  PORT_CHAR('^')

#define KEYB_ROW2   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE)     PORT_CHAR('\'')          PORT_CHAR('"') \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('`')           PORT_CHAR('~') \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)     PORT_CHAR(',')           PORT_CHAR('<') \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)      PORT_CHAR('.')           PORT_CHAR('>') \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)     PORT_CHAR('/')           PORT_CHAR('?') \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Dead Key")        PORT_CODE(KEYCODE_TILDE)                \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)         PORT_CHAR('a')           PORT_CHAR('A') \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)         PORT_CHAR('b')           PORT_CHAR('B')

#define KEYB_HOTBIT_ROW2     \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE)     PORT_CHAR('~') PORT_CHAR('^') \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('[') PORT_CHAR(']') \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)     PORT_CHAR(',') PORT_CHAR(';') \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)      PORT_CHAR('.') PORT_CHAR(':') \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)     PORT_CHAR('/') PORT_CHAR('?') \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_TILDE)     PORT_CHAR('<') PORT_CHAR('>') \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)         PORT_CHAR('a') PORT_CHAR('A') \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)         PORT_CHAR('b') PORT_CHAR('B')

#define KEYB_EXPERT10_ROW2   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE)     PORT_CHAR('\'') PORT_CHAR('"')  \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR(0xC7) PORT_CHAR(0xE7) \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)     PORT_CHAR(',') PORT_CHAR('<')   \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)      PORT_CHAR('.') PORT_CHAR('>')   \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)     PORT_CHAR('/') PORT_CHAR('?')   \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Dead Key")        PORT_CODE(KEYCODE_TILDE)        \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)         PORT_CHAR('a') PORT_CHAR('A')   \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)         PORT_CHAR('b') PORT_CHAR('B')

#define KEYB_EXPERT11_ROW2   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('*')  PORT_CHAR('@')  \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON)     PORT_CHAR(0xC7) PORT_CHAR(0xE7) \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)     PORT_CHAR(',')  PORT_CHAR('<')  \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)      PORT_CHAR('.')  PORT_CHAR('>')  \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_TILDE)     PORT_CHAR('/')  PORT_CHAR('?')  \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)     PORT_CHAR(';')  PORT_CHAR(':')  \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)         PORT_CHAR('a')  PORT_CHAR('A')  \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)         PORT_CHAR('b')  PORT_CHAR('B')

#define KEYB_ROW3   \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C') \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D') \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E') \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F') \
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G') \
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H') \
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I') \
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J')

#define KEYB_ROW4   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K') \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L') \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M') \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N') \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_O) PORT_CHAR('o') PORT_CHAR('O') \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P') \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q) PORT_CHAR('q') PORT_CHAR('Q') \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R')

#define KEYB_ROW5   \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_S) PORT_CHAR('s') PORT_CHAR('S') \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T') \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U') \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V') \
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W') \
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_X) PORT_CHAR('x') PORT_CHAR('X') \
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y) PORT_CHAR('y') PORT_CHAR('Y') \
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z) PORT_CHAR('z') PORT_CHAR('Z')

#define KEYB_ROW6   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)           \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)           \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("GRAPH")  PORT_CODE(KEYCODE_PGUP)     PORT_CHAR(UCHAR_MAMEKEY(F6))       \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CAPS")   PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK)) \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CODE")   PORT_CODE(KEYCODE_PGDN)     PORT_CHAR(UCHAR_MAMEKEY(F7))       \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1  F6") PORT_CODE(KEYCODE_F1)       PORT_CHAR(UCHAR_MAMEKEY(F1))       \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2  F7") PORT_CODE(KEYCODE_F2)       PORT_CHAR(UCHAR_MAMEKEY(F2))       \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3  F8") PORT_CODE(KEYCODE_F3)       PORT_CHAR(UCHAR_MAMEKEY(F3))

#define KEYB_EXPERT11_ROW6   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD)                        PORT_CODE(KEYCODE_LSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)           \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CONTROL")   PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)           \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("L GRA")     PORT_CODE(KEYCODE_PGUP)     PORT_CHAR(UCHAR_MAMEKEY(F6))       \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CAPS LOCK") PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK)) \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("R GRA")     PORT_CODE(KEYCODE_PGDN)     PORT_CHAR(UCHAR_MAMEKEY(F7))       \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1  F6")    PORT_CODE(KEYCODE_F1)       PORT_CHAR(UCHAR_MAMEKEY(F1))       \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2  F7")    PORT_CODE(KEYCODE_F2)       PORT_CHAR(UCHAR_MAMEKEY(F2))       \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3  F8")    PORT_CODE(KEYCODE_F3)       PORT_CHAR(UCHAR_MAMEKEY(F3))

#define KEYB_ROW7   \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F4  F9")  PORT_CODE(KEYCODE_F4)        PORT_CHAR(UCHAR_MAMEKEY(F4))  \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F5  F10") PORT_CODE(KEYCODE_F5)        PORT_CHAR(UCHAR_MAMEKEY(F5))  \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD)                      PORT_CODE(KEYCODE_ESC)       PORT_CHAR(UCHAR_MAMEKEY(ESC)) \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD)                      PORT_CODE(KEYCODE_TAB)       PORT_CHAR('\t')               \
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("STOP")    PORT_CODE(KEYCODE_RCONTROL)  PORT_CHAR(UCHAR_MAMEKEY(F8))  \
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYBOARD)                      PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8)                  \
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SELECT")  PORT_CODE(KEYCODE_END)       PORT_CHAR(UCHAR_MAMEKEY(F9))  \
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_KEYBOARD)                      PORT_CODE(KEYCODE_ENTER)     PORT_CHAR(13)

#define KEYB_ROW8   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD)                  PORT_CODE(KEYCODE_SPACE)  PORT_CHAR(' ')                   \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD)                  PORT_CODE(KEYCODE_HOME)   PORT_CHAR(UCHAR_MAMEKEY(HOME))   \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD)                  PORT_CODE(KEYCODE_INSERT) PORT_CHAR(UCHAR_MAMEKEY(INSERT)) \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DEL") PORT_CODE(KEYCODE_DEL)    PORT_CHAR(UCHAR_MAMEKEY(DEL))    \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD)                  PORT_CODE(KEYCODE_LEFT)   PORT_CHAR(UCHAR_MAMEKEY(LEFT))   \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD)                  PORT_CODE(KEYCODE_UP)     PORT_CHAR(UCHAR_MAMEKEY(UP))     \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD)                  PORT_CODE(KEYCODE_DOWN)   PORT_CHAR(UCHAR_MAMEKEY(DOWN))   \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD)                  PORT_CODE(KEYCODE_RIGHT)  PORT_CHAR(UCHAR_MAMEKEY(RIGHT))

#define KEYB_ROW9   \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ASTERISK)  PORT_CHAR(UCHAR_MAMEKEY(ASTERISK))  \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_PLUS_PAD)  PORT_CHAR(UCHAR_MAMEKEY(PLUS_PAD))  \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH_PAD) PORT_CHAR(UCHAR_MAMEKEY(SLASH_PAD)) \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_0_PAD)     PORT_CHAR(UCHAR_MAMEKEY(0_PAD))     \
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_1_PAD)     PORT_CHAR(UCHAR_MAMEKEY(1_PAD))     \
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_2_PAD)     PORT_CHAR(UCHAR_MAMEKEY(2_PAD))     \
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_3_PAD)     PORT_CHAR(UCHAR_MAMEKEY(3_PAD))     \
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_4_PAD)     PORT_CHAR(UCHAR_MAMEKEY(4_PAD))

#define KEYB_ROW10  \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_5_PAD)     PORT_CHAR(UCHAR_MAMEKEY(5_PAD))     \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6_PAD)     PORT_CHAR(UCHAR_MAMEKEY(6_PAD))     \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_7_PAD)     PORT_CHAR(UCHAR_MAMEKEY(7_PAD))     \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_8_PAD)     PORT_CHAR(UCHAR_MAMEKEY(8_PAD))     \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_9_PAD)     PORT_CHAR(UCHAR_MAMEKEY(9_PAD))     \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS_PAD) PORT_CHAR(UCHAR_MAMEKEY(MINUS_PAD)) \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER_PAD) PORT_CHAR(UCHAR_MAMEKEY(COMMA_PAD)) \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_DEL_PAD)   PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD))

static INPUT_PORTS_START(msx)
	PORT_START("KEY0")
	KEYB_ROW0
	KEYB_ROW1

	PORT_START("KEY1")
	KEYB_ROW2
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	KEYB_ROW6
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	PORT_BIT(0xff00, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEY5")
	PORT_BIT(0xffff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END

#ifdef UNREFERENCED_CODE
static INPUT_PORTS_START(msxuk)
	PORT_START("KEY0")
	KEYB_ROW0
	KEYB_ROW1

	PORT_START("KEY1")
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD)                       PORT_CODE(KEYCODE_QUOTE)     PORT_CHAR('\'') PORT_CHAR('"')
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD)                       PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR(0xA3) PORT_CHAR('~')
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD)                       PORT_CODE(KEYCODE_COMMA)     PORT_CHAR(',')  PORT_CHAR('<')
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD)                       PORT_CODE(KEYCODE_STOP)      PORT_CHAR('.')  PORT_CHAR('>')
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD)                       PORT_CODE(KEYCODE_SLASH)     PORT_CHAR('/')  PORT_CHAR('?')
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Dead Key") PORT_CODE(KEYCODE_TILDE)
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD)                       PORT_CODE(KEYCODE_A)         PORT_CHAR('a')  PORT_CHAR('A')
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD)                       PORT_CODE(KEYCODE_B)         PORT_CHAR('b')  PORT_CHAR('B')
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	KEYB_ROW6
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	PORT_BIT(0xff00, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEY5")
	PORT_BIT(0xffff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END
#endif

#define KEYB_JP_ROW0   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_CHAR('0')                 \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!')  \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('"')  \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#')  \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$')  \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%')  \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('&')  \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('\'')

#define KEYB_JP_ROW1   \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)          PORT_CHAR('8')  PORT_CHAR('(') \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)          PORT_CHAR('9')  PORT_CHAR(')') \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)      PORT_CHAR('-')  PORT_CHAR('=') \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS)     PORT_CHAR('^')  PORT_CHAR('~') \
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH2) PORT_CHAR(0xA5) PORT_CHAR('|') \
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('@')  PORT_CHAR('`') \
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR('[')  PORT_CHAR('{') \
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON)      PORT_CHAR(';')  PORT_CHAR('+')

#define KEYB_KR_ROW1   \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD)                             PORT_CODE(KEYCODE_8)          PORT_CHAR('8')    PORT_CHAR('(') \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD)                             PORT_CODE(KEYCODE_9)          PORT_CHAR('9')    PORT_CHAR(')') \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD)                             PORT_CODE(KEYCODE_MINUS)      PORT_CHAR('-')    PORT_CHAR('=') \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD)                             PORT_CODE(KEYCODE_EQUALS)     PORT_CHAR('^')    PORT_CHAR('~') \
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("\xE2\x82\xA9 |") PORT_CODE(KEYCODE_BACKSLASH2) PORT_CHAR(0xffe6) PORT_CHAR('|') \
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYBOARD)                             PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('@')    PORT_CHAR('`') \
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_KEYBOARD)                             PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR('[')    PORT_CHAR('{') \
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_KEYBOARD)                             PORT_CODE(KEYCODE_COLON)      PORT_CHAR(';')    PORT_CHAR('+')

#define KEYB_JP_ROW2   \
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD)                PORT_CODE(KEYCODE_QUOTE)     PORT_CHAR(':') PORT_CHAR('*') \
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD)                PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR(']') PORT_CHAR('}') \
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD)                PORT_CODE(KEYCODE_COMMA)     PORT_CHAR(',') PORT_CHAR('<') \
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD)                PORT_CODE(KEYCODE_STOP)      PORT_CHAR('.') PORT_CHAR('>') \
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD)                PORT_CODE(KEYCODE_SLASH)     PORT_CHAR('/') PORT_CHAR('?') \
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("_") PORT_CODE(KEYCODE_TILDE)     PORT_CHAR('_')                \
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD)                PORT_CODE(KEYCODE_A)         PORT_CHAR('a') PORT_CHAR('A') \
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD)                PORT_CODE(KEYCODE_B)         PORT_CHAR('b') PORT_CHAR('B')

static INPUT_PORTS_START(msxjp)
	PORT_START("KEY0")
	KEYB_JP_ROW0
	KEYB_JP_ROW1

	PORT_START("KEY1")
	KEYB_JP_ROW2
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("GRAPH")  PORT_CODE(KEYCODE_PGUP)     PORT_CHAR(UCHAR_MAMEKEY(F6))
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CAPS")   PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("KANA")   PORT_CODE(KEYCODE_PGDN)     PORT_CHAR(UCHAR_MAMEKEY(F7))
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1  F6") PORT_CODE(KEYCODE_F1)       PORT_CHAR(UCHAR_MAMEKEY(F1))
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2  F7") PORT_CODE(KEYCODE_F2)       PORT_CHAR(UCHAR_MAMEKEY(F2))
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3  F8") PORT_CODE(KEYCODE_F3)       PORT_CHAR(UCHAR_MAMEKEY(F3))
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	PORT_BIT(0xff00, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEY5")
	PORT_BIT(0xffff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END

static INPUT_PORTS_START(msxkr)
	PORT_START("KEY0")
	KEYB_JP_ROW0
	KEYB_KR_ROW1

	PORT_START("KEY1")
	KEYB_JP_ROW2
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("GRAPH")  PORT_CODE(KEYCODE_PGUP)     PORT_CHAR(UCHAR_MAMEKEY(F6))
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CAPS")   PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Hangul") PORT_CODE(KEYCODE_PGDN)     PORT_CHAR(UCHAR_MAMEKEY(F7))
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1  F6") PORT_CODE(KEYCODE_F1)       PORT_CHAR(UCHAR_MAMEKEY(F1))
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2  F7") PORT_CODE(KEYCODE_F2)       PORT_CHAR(UCHAR_MAMEKEY(F2))
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3  F8") PORT_CODE(KEYCODE_F3)       PORT_CHAR(UCHAR_MAMEKEY(F3))
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	PORT_BIT(0xff00, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEY5")
	PORT_BIT(0xffff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END

static INPUT_PORTS_START(hotbit)
	PORT_START("KEY0")
	KEYB_HOTBIT_ROW0
	KEYB_HOTBIT_ROW1

	PORT_START("KEY1")
	KEYB_HOTBIT_ROW2
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	KEYB_ROW6
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	PORT_BIT(0xff00, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEY5")
	PORT_BIT(0xffff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END

/* 2008-05 FP: I guess these belong to the keypad */
#define KEYB_EXPERT11_ROW9  \
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_PLUS_PAD)  PORT_CHAR(UCHAR_MAMEKEY(PLUS_PAD))  \
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS_PAD) PORT_CHAR(UCHAR_MAMEKEY(MINUS_PAD)) \
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ASTERISK)  PORT_CHAR(UCHAR_MAMEKEY(ASTERISK))  \
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH_PAD) PORT_CHAR(UCHAR_MAMEKEY(SLASH_PAD)) \
	PORT_BIT(0xf000, IP_ACTIVE_LOW, IPT_UNUSED)

static INPUT_PORTS_START(expert11)
	PORT_START("KEY0")
	KEYB_EXPERT11_ROW0
	KEYB_EXPERT11_ROW1

	PORT_START("KEY1")
	KEYB_EXPERT11_ROW2
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	KEYB_EXPERT11_ROW6
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	KEYB_EXPERT11_ROW9

	PORT_START("KEY5")
	PORT_BIT(0xffff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END

static INPUT_PORTS_START(expert10)
	PORT_START("KEY0")
	KEYB_ROW0
	KEYB_ROW1

	PORT_START("KEY1")
	KEYB_EXPERT10_ROW2
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	KEYB_EXPERT11_ROW6
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	KEYB_EXPERT11_ROW9

	PORT_START("KEY5")
	PORT_BIT(0xffff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END

static INPUT_PORTS_START(msx2)
	PORT_START("KEY0")
	KEYB_ROW0
	KEYB_ROW1

	PORT_START("KEY1")
	KEYB_ROW2
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	KEYB_ROW6
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	KEYB_ROW9

	PORT_START("KEY5")
	KEYB_ROW10

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END

static INPUT_PORTS_START(msx2jp)
	PORT_START("KEY0")
	KEYB_JP_ROW0
	KEYB_JP_ROW1

	PORT_START("KEY1")
	KEYB_JP_ROW2
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("GRAPH")  PORT_CODE(KEYCODE_PGUP)     PORT_CHAR(UCHAR_MAMEKEY(F6))
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CAPS")   PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("KANA")   PORT_CODE(KEYCODE_PGDN)     PORT_CHAR(UCHAR_MAMEKEY(F7))
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1  F6") PORT_CODE(KEYCODE_F1)       PORT_CHAR(UCHAR_MAMEKEY(F1))
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2  F7") PORT_CODE(KEYCODE_F2)       PORT_CHAR(UCHAR_MAMEKEY(F2))
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3  F8") PORT_CODE(KEYCODE_F3)       PORT_CHAR(UCHAR_MAMEKEY(F3))
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	KEYB_ROW9

	PORT_START("KEY5")
	KEYB_ROW10

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END

static INPUT_PORTS_START(msx2kr)
	PORT_START("KEY0")
	KEYB_JP_ROW0
	KEYB_KR_ROW1

	PORT_START("KEY1")
	KEYB_JP_ROW2
	KEYB_ROW3

	PORT_START("KEY2")
	KEYB_ROW4
	KEYB_ROW5

	PORT_START("KEY3")
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYBOARD)                     PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("GRAPH")  PORT_CODE(KEYCODE_PGUP)     PORT_CHAR(UCHAR_MAMEKEY(F6))
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CAPS")   PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Hangul") PORT_CODE(KEYCODE_PGDN)     PORT_CHAR(UCHAR_MAMEKEY(F7))
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1  F6") PORT_CODE(KEYCODE_F1)       PORT_CHAR(UCHAR_MAMEKEY(F1))
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2  F7") PORT_CODE(KEYCODE_F2)       PORT_CHAR(UCHAR_MAMEKEY(F2))
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3  F8") PORT_CODE(KEYCODE_F3)       PORT_CHAR(UCHAR_MAMEKEY(F3))
	KEYB_ROW7

	PORT_START("KEY4")
	KEYB_ROW8
	PORT_BIT(0xff00, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEY5")
	PORT_BIT(0xffff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_INCLUDE(msx_dips)
INPUT_PORTS_END


// Some MSX2+ can switch the z80 clock between 3.5 and 5.3 MHz
WRITE_LINE_MEMBER(msx2_state::turbo_w)
{
	// 0 - 5.369317 MHz
	// 1 - 3.579545 MHz
	m_maincpu->set_unscaled_clock(21.477272_MHz_XTAL / (state ? 6 : 4));
}


#define MSX_XBORDER_PIXELS          15
#define MSX_YBORDER_PIXELS          27
#define MSX_TOTAL_XRES_PIXELS       256 + (MSX_XBORDER_PIXELS * 2)
#define MSX_TOTAL_YRES_PIXELS       192 + (MSX_YBORDER_PIXELS * 2)
#define MSX_VISIBLE_XBORDER_PIXELS  8
#define MSX_VISIBLE_YBORDER_PIXELS  24


void msx_state::msx1_floplist(machine_config &config)
{
	SOFTWARE_LIST(config, "flop_list").set_original("msx1_flop");
}

void msx2_state::msx2_floplist(machine_config &config)
{
	SOFTWARE_LIST(config, "flop_list").set_original("msx2_flop");
	SOFTWARE_LIST(config, "msx1_flp_l").set_compatible("msx1_flop");
}

void msx2_state::msx2plus_floplist(machine_config &config)
{
	SOFTWARE_LIST(config, "flop_list").set_original("msx2p_flop");
	SOFTWARE_LIST(config, "msx2_flp_l").set_compatible("msx2_flop");
	SOFTWARE_LIST(config, "msx1_flp_l").set_compatible("msx1_flop");    // maybe not?
}

void msx2_state::msxr_floplist(machine_config &config)
{
	SOFTWARE_LIST(config, "flop_list").set_original("msxr_flop");
	SOFTWARE_LIST(config, "msx2p_flp_l").set_compatible("msx2p_flop");
	SOFTWARE_LIST(config, "msx2_flp_l").set_compatible("msx2_flop");    // maybe not?
	SOFTWARE_LIST(config, "msx1_flp_l").set_compatible("msx1_flop");    // maybe not?
}

void msx_state::floppy_formats(format_registration &fr)
{
	fr.add_mfm_containers();
	fr.add(FLOPPY_MSX_FORMAT);
	fr.add(FLOPPY_DMK_FORMAT);
}

static void msx_floppies(device_slot_interface &device)
{
	device.option_add("35dd", FLOPPY_35_DD);
	device.option_add("35ssdd", FLOPPY_35_SSDD);
}

void msx_state::msx_fd1793(machine_config &config)
{
	fd1793_device& fdc(FD1793(config, "fdc", 4_MHz_XTAL / 4));
	fdc.set_force_ready(true);
}

void msx_state::msx_wd2793_force_ready(machine_config &config)
{
	// From NMS8245 schematics:
	// READY + HLT - pulled high
	// SSO/-ENMF + -DDEN + ENP + -5/8 - pulled low
	wd2793_device& fdc(WD2793(config, "fdc", 4_MHz_XTAL / 4));
	fdc.set_force_ready(true);
}

void msx_state::msx_wd2793(machine_config &config)
{
	WD2793(config, "fdc", 4_MHz_XTAL / 4);
}

void msx_state::msx_mb8877a(machine_config & config)
{
	// From CF-3300 FDC schematic:
	// READY + HLT - pulled high
	// -DDEN - pulled low
	mb8877_device& fdc(MB8877(config, "fdc", 4_MHz_XTAL / 4));
	fdc.set_force_ready(true);
}

void msx_state::msx_tc8566af(machine_config &config)
{
	TC8566AF(config, "fdc", 16'000'000);
}

void msx_state::msx_microsol(machine_config &config)
{
	wd2793_device& fdc(WD2793(config, "fdc", 4_MHz_XTAL / 4));
	fdc.set_force_ready(true);
}

void msx_state::msx_1_35_ssdd_drive(machine_config &config)
{
	FLOPPY_CONNECTOR(config, "fdc:0", msx_floppies, "35ssdd", msx_state::floppy_formats);
}

void msx_state::msx_1_35_dd_drive(machine_config &config)
{
	FLOPPY_CONNECTOR(config, "fdc:0", msx_floppies, "35dd", msx_state::floppy_formats);
}

void msx_state::msx_2_35_dd_drive(machine_config &config)
{
	FLOPPY_CONNECTOR(config, "fdc:0", msx_floppies, "35dd", msx_state::floppy_formats);
	FLOPPY_CONNECTOR(config, "fdc:1", msx_floppies, "35dd", msx_state::floppy_formats);
}

void msx2_state::msx_ym2413(machine_config &config)
{
	YM2413(config, "ym2413", 21.477272_MHz_XTAL / 6).add_route(ALL_OUTPUTS, m_speaker, 0.4);
}

void msx2_state::msx2_64kb_vram(machine_config &config)
{
	m_v9938->set_vram_size(0x10000);
}

void msx_state::msx_base(machine_config &config, XTAL xtal, int cpu_divider)
{
	// basic machine hardware
	Z80(config, m_maincpu, xtal / cpu_divider);         // 3.579545 MHz
	m_maincpu->set_addrmap(AS_PROGRAM, &msx_state::memory_map);
	config.set_maximum_quantum(attotime::from_hz(60));

	INPUT_MERGER_ANY_HIGH(config, m_mainirq).output_handler().set_inputline("maincpu", INPUT_LINE_IRQ0);

	I8255(config, m_ppi);
	m_ppi->out_pa_callback().set(FUNC(msx_state::ppi_port_a_w));
	m_ppi->in_pb_callback().set(FUNC(msx_state::ppi_port_b_r));
	m_ppi->out_pc_callback().set(FUNC(msx_state::ppi_port_c_w));

	// Video hardware
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);

	// sound hardware
	SPEAKER(config, m_speaker).front_center();
	DAC_1BIT(config, m_dac, 0).add_route(ALL_OUTPUTS, m_speaker, 0.1);

	AY8910(config, m_ay8910, xtal / cpu_divider / 2);
	m_ay8910->set_flags(AY8910_SINGLE_OUTPUT);
	m_ay8910->port_a_read_callback().set(FUNC(msx2_state::psg_port_a_r));
	m_ay8910->port_b_read_callback().set(FUNC(msx2_state::psg_port_b_r));
	m_ay8910->port_a_write_callback().set(FUNC(msx2_state::psg_port_a_w));
	m_ay8910->port_b_write_callback().set(FUNC(msx2_state::psg_port_b_w));
	m_ay8910->add_route(ALL_OUTPUTS, m_speaker, 0.3);

	// printer
	CENTRONICS(config, m_centronics, centronics_devices, "printer");
	m_centronics->busy_handler().set(m_cent_status_in, FUNC(input_buffer_device::write_bit1));

	OUTPUT_LATCH(config, m_cent_data_out);
	m_centronics->set_output_latch(*m_cent_data_out);
	INPUT_BUFFER(config, m_cent_status_in);

	OUTPUT_LATCH(config, m_cent_ctrl_out);
	m_cent_ctrl_out->bit_handler<1>().set(m_centronics, FUNC(centronics_device::write_strobe));

	// cassette
	CASSETTE(config, m_cassette);
	m_cassette->set_formats(fmsx_cassette_formats);
	m_cassette->set_default_state(CASSETTE_PLAY);
	m_cassette->add_route(ALL_OUTPUTS, m_speaker, 0.05);
	m_cassette->set_interface("msx_cass");
}

template<typename VDPType>
void msx_state::msx1(VDPType &vdp_type, machine_config &config, bool has_cartlist)
{
	msx_base(config, 10.738635_MHz_XTAL, 3);

	m_maincpu->set_addrmap(AS_IO, &msx_state::msx_io_map);
	m_maincpu->set_vblank_int("screen", FUNC(msx_state::msx_interrupt)); /* Needed for mouse updates */

	// Software lists
	SOFTWARE_LIST(config, "cass_list").set_original("msx1_cass");

	vdp_type(config, m_tms9928a, 10.738635_MHz_XTAL);
	m_tms9928a->set_screen(m_screen);
	m_tms9928a->set_vram_size(0x4000);
	m_tms9928a->int_callback().set(m_mainirq, FUNC(input_merger_device::in_w<0>));

	if (has_cartlist)
		SOFTWARE_LIST(config, "cart_list").set_original("msx1_cart");
}


void msx2_state::msx2_base(machine_config &config, bool has_cartlist)
{
	msx_base(config, 21.477272_MHz_XTAL, 6);

	// real time clock
	RP5C01(config, m_rtc, 32.768_kHz_XTAL);

	// Software lists
	SOFTWARE_LIST(config, "cass_list").set_original("msx2_cass");
	SOFTWARE_LIST(config, "msx1_cas_l").set_compatible("msx1_cass");

	if (has_cartlist)
	{
		SOFTWARE_LIST(config, "cart_list").set_original("msx2_cart");
		SOFTWARE_LIST(config, "msx1_crt_l").set_compatible("msx1_cart");
	}
}

void msx2_state::msx2(machine_config &config, bool has_cartlist)
{
	msx2_base(config, has_cartlist);

	m_maincpu->set_addrmap(AS_IO, &msx2_state::msx2_io_map);

	// video hardware
	V9938(config, m_v9938, 21.477272_MHz_XTAL);
	m_v9938->set_screen_ntsc(m_screen);
	m_v9938->set_vram_size(0x20000);
	m_v9938->int_cb().set(m_mainirq, FUNC(input_merger_device::in_w<0>));
}


void msx2_state::msx2_pal(machine_config &config, bool has_cartlist)
{
	msx2(config, has_cartlist);
	m_v9938->set_screen_pal(m_screen);
}


void msx2_state::msx2plus(machine_config &config)
{
	msx2_base(config, true);

	m_maincpu->set_addrmap(AS_IO, &msx2_state::msx2plus_io_map);

	// video hardware
	V9958(config, m_v9958, 21.477272_MHz_XTAL);
	m_v9958->set_screen_ntsc(m_screen);
	m_v9958->set_vram_size(0x20000);
	m_v9958->int_cb().set(m_mainirq, FUNC(input_merger_device::in_w<0>));
}

void msx2_state::turbor(machine_config &config)
{
	msx2plus(config);

	R800(config.replace(), m_maincpu, 28.636363_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &msx2_state::memory_map);
	m_maincpu->set_addrmap(AS_IO, &msx2_state::msx2plus_io_map);
}


/***************************************************************************

  Game driver(s)

***************************************************************************/

/********************************  MSX 1 **********************************/

/* MSX - Al Alamiah AX-150 */

ROM_START(ax150)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("ax150bios.rom", 0x0000, 0x8000, CRC(bd95c436) SHA1(5e094fca95ab8e91873ee372a3f1239b9a48a48d))
	ROM_LOAD("ax150arab.rom", 0x8000, 0x8000, CRC(339cd1aa) SHA1(0287b2ec897b9196788cd9f10c99e1487d7adbbb))
ROM_END

void msx_state::ax150(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "arab", 1, 0, 1, 2, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 2, 2); /* 32KB RAM */
}

/* MSX - Al Alamiah AX-170 */

ROM_START (ax170)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("ax170bios.rom", 0x0000, 0x8000, CRC(bd95c436) SHA1(5e094fca95ab8e91873ee372a3f1239b9a48a48d))
	ROM_LOAD("ax170arab.rom", 0x8000, 0x8000, CRC(339cd1aa) SHA1(0287b2ec897b9196788cd9f10c99e1487d7adbbb))
ROM_END


void msx_state::ax170(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// T7937 (in ax170mk2)

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "arab", 1, 0, 1, 2, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4); /* 64KB RAM */
}

/* MSX - Canon V-8 */

ROM_START(canonv8)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("v8bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::canonv8(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149??
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 3, 1).force_start_address(0xe000);   /* 8KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Canon V-10 */

ROM_START(canonv10)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("v10bios.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::canonv10(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 3, 1);   /* 16KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Canon V-20 */

ROM_START(canonv20)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("v20bios.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::canonv20(machine_config &config)
{
	msx1(TMS9929A, config);
	// XTAL: 1431818(Z80/PSG) + 10.6875(VDP)
	// YM2149
	// TMS9929ANL
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Canon V-20E */

ROM_START(canonv20e)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("v20ebios.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

/* MSX - Canon V-20F */

ROM_START(canonv20f)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("v20fbios.rom", 0x0000, 0x8000, CRC(e0e894b7) SHA1(d99eebded5db5fce1e072d08e642c0909bc7efdd))
ROM_END

/* MSX - Canon V-20G */

ROM_START(canonv20g)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("v20gbios.rom", 0x0000, 0x8000, CRC(d6e704ad) SHA1(d67be6d7d56d7229418f4e122f2ec27990db7d19))
ROM_END

/* MSX - Canon V-20S */

ROM_START(canonv20s)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("v20sbios.rom", 0x0000, 0x8000, CRC(c72b186e) SHA1(9fb289ea5c11d497ee00703f64e82575d1c59923))
ROM_END

/* MSX - Casio MX-10 */

ROM_START(mx10)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD( "mx10bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::mx10(machine_config &config)
{
	msx1(TMS9118, config);
	// FDC: None, 0 drives
	// 2? Cartridge slots
	// Z80: uPD780C-1

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 3, 1); // 16KB RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
}

/* MSX - Casio MX-15 */

ROM_START(mx15)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mx15bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::mx15(machine_config &config)
{
	msx1(TMS9928A, config);
	// FDC: None, 0 drives
	// 3 Cartridge slots
	// T6950

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 3, 1); // 16KB RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<3>(config, MSX_SLOT_CARTRIDGE, "cartslot3", 3, 0, msx_cart, nullptr);
}

/* MSX - Casio MX-101 */

ROM_START(mx101)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mx101bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::mx101(machine_config &config)
{
	msx1(TMS9928A, config);
	// FDC: None, 0 drives
	// 2? Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 3, 1); // 16KB RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
}

/* MSX - Casio PV-7 */

ROM_START(pv7)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("pv7bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::pv7(machine_config &config)
{
	msx1(TMS9118, config);
	// AY8910?
	// FDC: None, 0 drives
	// 1 Cartridge slot + expansion slot, or 2 cartridge slots?
	// By adding a Casio KB-7 2 additional cartridge slots become available and 8KB extra RAM?
	// No cassette port
	// No printer port
	// Z80: uPD780C-1

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 3, 1).force_start_address(0xe000);   /* 8KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
}

/* MSX - Casio PV-16 */

ROM_START(pv16)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("pv16.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::pv16(machine_config &config)
{
	msx1(TMS9118, config);
	// AY8910
	// FDC: None, 0 drives
	// 1 Cartridge slot
	// No printer port

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 3, 1);   /* 16KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
}

/* MSX - Daewoo CPC-88 */

ROM_START(cpc88)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("88bios.rom", 0x0000, 0x8000, CRC(3ab0cd3b) SHA1(171b587bd5a947a13f3114120b6e7baca3b57d78))
	ROM_LOAD("88han.rom",  0x8000, 0x2000, CRC(938db440) SHA1(d41676fde0a3047792f93c4a41509b8749e55e74))
	ROM_RELOAD(0xa000, 0x2000)
ROM_END

void msx_state::cpc88(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2? Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);   /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Daewoo DPC-100 */

ROM_START(dpc100)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("100bios.rom", 0x0000, 0x8000, CRC(3ab0cd3b) SHA1(171b587bd5a947a13f3114120b6e7baca3b57d78))
	ROM_LOAD("100han.rom",  0x8000, 0x4000, CRC(97478efb) SHA1(4421fa2504cbce18f7c84b5ea97f04e017007f07))
ROM_END

void msx_state::dpc100(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 3, 1);   /* 16KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Daewoo DPC-180 */

ROM_START(dpc180)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("180bios.rom", 0x0000, 0x8000, CRC(3ab0cd3b) SHA1(171b587bd5a947a13f3114120b6e7baca3b57d78))
	ROM_LOAD("180han.rom",  0x8000, 0x4000, CRC(97478efb) SHA1(4421fa2504cbce18f7c84b5ea97f04e017007f07))
ROM_END

void msx_state::dpc180(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Daewoo DPC-200 */

ROM_START(dpc200)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("200bios.rom", 0x0000, 0x8000, CRC(3ab0cd3b) SHA1(171b587bd5a947a13f3114120b6e7baca3b57d78))
	ROM_LOAD("200han.rom",  0x8000, 0x4000, CRC(97478efb) SHA1(4421fa2504cbce18f7c84b5ea97f04e017007f07))
ROM_END

void msx_state::dpc200(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Daewoo DPC-200E */

ROM_START(dpc200e)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("dpc200ebios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
ROM_END

void msx_state::dpc200e(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Daewoo Zemmix CPC-50A */

ROM_START(cpc50a)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("50abios.rom", 0x0000, 0x8000, CRC(c3a868ef) SHA1(a08a940aa87313509e00bc5ac7494d53d8e03492))
ROM_END

void msx_state::cpc50a(machine_config &config)
{
	msx1(TMS9118, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 1? Cartridge slot
	// No keyboard?
	// No cassette port?
	// No printer port?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 3, 1).force_start_address(0xe000);  /* 8KB RAM */
}

/* MSX - Daewoo Zemmix CPC-50B */

ROM_START(cpc50b)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("50bbios.rom", 0x0000, 0x8000, CRC(c3a868ef) SHA1(a08a940aa87313509e00bc5ac7494d53d8e03492))
ROM_END

void msx_state::cpc50b(machine_config &config)
{
	msx1(TMS9118, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 1? Cartridge slot
	// No keyboard?
	// No cassette port?
	// No printer port?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 3, 1);  /* 16KB RAM */
}

/* MSX - Daewoo Zemmix CPC-51 */

ROM_START(cpc51)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("51bios.rom", 0x0000, 0x8000, CRC(c3a868ef) SHA1(a08a940aa87313509e00bc5ac7494d53d8e03492))
ROM_END

void msx_state::cpc51(machine_config &config)
{
	msx1(TMS9118, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 1 Cartridge slot
	// No keyboard, just a keyboard connector
	// No cassette port
	// No printer port

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Dragon MSX-64 */

ROM_START(dgnmsx)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("uk1msx048.ic37", 0x0000, 0x4000, CRC(24c198be) SHA1(7f8c94cb8913db32a696dec80ffc78e46693f1b7))
	ROM_LOAD("uk2msx058.ic6",  0x4000, 0x4000, CRC(e516e7e5) SHA1(05fedd4b9bfcf4949020c79d32c4c3f03a54fb62))
ROM_END

void msx_state::dgnmsx(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Fenner DPC-200 */

ROM_START(fdpc200)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("dpc200bios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
ROM_END

void msx_state::fdpc200(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Fenner FPC-500 */

ROM_START(fpc500)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("fpc500bios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
ROM_END

void msx_state::fpc500(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Fenner SPC-800 */

ROM_START(fspc800)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("spc800bios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
ROM_END

void msx_state::fspc800(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// Z80: GSS Z8400APS

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Frael Bruc 100-1 */

ROM_START(bruc100)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("bruc100-1bios.rom", 0x0000, 0x8000, CRC(c7bc4298) SHA1(3abca440cba16ac5e162b602557d30169f77adab))
ROM_END

void msx_state::bruc100(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000);   /* 64KB RAM */
}

/* MSX - Fujitsu FM-X */

ROM_START(fmx)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("fmxbios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::fmx(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 1 Cartridge slot, 2 "Fujitsu expansion slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 3, 1); // 16KB RAM
	// Fujitsu expansion slot #1 in slot 1
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	// Fijutsu expansion slot #2 in slot 3
}

/* MSX - Goldstar FC-80U */

ROM_START(gsfc80u)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("fc80ubios.rom", 0x0000, 0x8000, CRC(3ab0cd3b) SHA1(171b587bd5a947a13f3114120b6e7baca3b57d78))
	ROM_LOAD("fc80uhan.rom",  0x8000, 0x2000, CRC(0cdb8501) SHA1(58dbe73ae80c2c409e766c3ace730ecd7bec89d0))
	ROM_RELOAD(0xa000, 0x2000)
ROM_END

void msx_state::gsfc80u(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Goldstar FC-200 */

ROM_START(gsfc200)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("fc200bios.rom.u5a", 0x0000, 0x4000, CRC(61f473fb) SHA1(c425750bbb2ae1d278216b45029d303e37d8df2f))
	ROM_LOAD("fc200bios.rom.u5b", 0x4000, 0x4000, CRC(1a99b1a1) SHA1(e18f72271b64693a2a2bc226e1b9ebd0448e07c0))
ROM_END

void msx_state::gsfc200(machine_config &config)
{
	msx1(TMS9129, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "rom", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Goldstar GFC-1080 */

ROM_START(gfc1080)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("gfc1080bios.rom",     0x0000, 0x8000, CRC(d9cdd4a6) SHA1(6b0be712b9c95c1e912252ab5703e1c0bc457d9e))
	ROM_LOAD("gfc1080han.rom",      0x8000, 0x4000, CRC(f209448c) SHA1(141b44212ba28e7d03e0b54126fedd9e0807dc42))
	ROM_LOAD("gfc1080pasocalc.rom", 0xc000, 0x4000, CRC(4014f7ea) SHA1(a5581fa3ce10f90f15ba3dc53d57b02d6e4af172))
ROM_END

void msx_state::gfc1080(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "rom", 0, 0, 0, 4, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4); // 64KB RAM
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Goldstar GFC-1080A */

ROM_START(gfc1080a)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("gfc1080abios.rom", 0x0000, 0x8000, CRC(3ab0cd3b) SHA1(171b587bd5a947a13f3114120b6e7baca3b57d78))
	ROM_LOAD("gfc1080ahan.rom",  0x8000, 0x2000, CRC(0cdb8501) SHA1(58dbe73ae80c2c409e766c3ace730ecd7bec89d0))
	ROM_RELOAD(0xa000, 0x2000)
ROM_END

void msx_state::gfc1080a(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "rom", 0, 0, 0, 3, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4); // 64KB RAM
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Gradiente Expert 1.0 */

ROM_START(expert10)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("expbios.rom", 0x0000, 0x8000, CRC(07610d77) SHA1(ef3e010eb57e4476700a3bbff9d2119ab3acdf62))
ROM_END

void msx_state::expert10(machine_config &config)
{
	msx1(TMS9128, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Gradiente Expert 1.1 */
ROM_START(expert11)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("expbios11.rom", 0x0000, 0x8000, CRC(efb4b972) SHA1(d6720845928ee848cfa88a86accb067397685f02))
ROM_END

void msx_state::expert11(machine_config &config)
{
	msx1(TMS9128, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Gradiente Expert 1.3 */
ROM_START(expert13)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("expbios13.rom", 0x0000, 0x8000, CRC(5638bc38) SHA1(605f5af3f358c6811f54e0173bad908614a198c0))
ROM_END

void msx_state::expert13(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 2, 0, 0, 4).set_total_size(0x10000);   /* 64KB Mapper RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Gradiente Expert DDPlus */
ROM_START(expertdp)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("eddpbios.rom", 0x0000, 0x8000, CRC(efb4b972) SHA1(d6720845928ee848cfa88a86accb067397685f02))
	ROM_LOAD("eddpdisk.rom", 0x8000, 0x4000, CRC(549f1d90) SHA1(f1525de4e0b60a6687156c2a96f8a8b2044b6c56))
ROM_END

void msx_state::expertdp(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: mb8877a, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// MSX Engine T7937A (also VDP)

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 3, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_mb8877a(config);
	msx_1_35_dd_drive(config);
	msx1_floplist(config);
}

/* MSX - Gradiente Expert Plus */

ROM_START(expertpl)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("exppbios.rom", 0x0000, 0x8000, CRC(efb4b972) SHA1(d6720845928ee848cfa88a86accb067397685f02))
	ROM_LOAD("exppdemo.rom", 0x8000, 0x4000, CRC(a9bbef64) SHA1(d4cea8c815f3eeabe0c6a1c845f902ec4318bf6b))
ROM_END

void msx_state::expertpl(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// MSX Engine T7937A

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "demo", 3, 3, 2, 1, "maincpu", 0x8000);
}

/* MSX - Hitachi MB-H2 */

ROM_START(mbh2)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("mbh2bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
	ROM_LOAD("mbh2firm.rom", 0x8000, 0x4000, CRC(4f03c947) SHA1(e2140fa2e8e59090ecccf55b62323ea9dcc66d0b))
ROM_END

void msx_state::mbh2(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// Builtin cassette player
	// Speed controller (normal, slow 1, slow 2)

	add_internal_slot(config, MSX_SLOT_ROM, "rom", 0, 0, 0, 3, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4); // 64KB RAM
}

/* MSX - Hitachi MB-H25 */

ROM_START(mbh25)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mbh25bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::mbh25(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "rom", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 2, 2); // 32KB RAM
}

/* MSX - Hitachi MB-H50 */

ROM_START(mbh50)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mbh50bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::mbh50(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// T6950

	add_internal_slot(config, MSX_SLOT_ROM, "rom", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4); // 64KB RAM
}

/* MSX - JVC HC-7GB */

ROM_START(jvchc7gb)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hc7gbbios.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::jvchc7gb(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "rom", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Mitsubishi ML-F48 */

ROM_START(mlf48)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mlf48bios.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::mlf48(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 2, 2); // 32KB RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Mitsubishi ML-F80 */

ROM_START(mlf80)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mlf80bios.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::mlf80(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Mitsubishi ML-F110 */

ROM_START(mlf110)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mlf110bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::mlf110(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 3, 1); // 16KB RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Mitsubishi ML-F120 */

ROM_START(mlf120)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("mlf120bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
	ROM_LOAD("mlf120firm.rom", 0x8000, 0x4000, CRC(4b5f3173) SHA1(21a9f60cb6370d0617ce54c42bb7d8e40a4ab560))
ROM_END

void msx_state::mlf120(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2? Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "firm", 1, 0, 1, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 2, 2); // 32KB RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Mitsubishi ML-FX1 */

ROM_START(mlfx1)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mlfx1bios.rom", 0x0000, 0x8000, CRC(62867dce) SHA1(0cbe0df4af45e8f531e9c761403ac9e71808f20c))
ROM_END

void msx_state::mlfx1(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */
}

/* MSX - National CF-1200 */

ROM_START(cf1200)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("1200bios.rom", 0x0000, 0x8000, CRC(5ad03407) SHA1(c7a2c5baee6a9f0e1c6ee7d76944c0ab1886796c))
ROM_END

void msx_state::cf1200(machine_config &config)
{
	msx1(TMS9918A, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 3, 1);   /* 16KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
}

/* MSX - National CF-2000 */

ROM_START(cf2000)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("2000bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::cf2000(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 3, 1);   /* 16KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
}

/* MSX - National CF-2700 */
ROM_START(cf2700)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("2700bios.rom.ic32", 0x0000, 0x8000, CRC(5ad03407) SHA1(c7a2c5baee6a9f0e1c6ee7d76944c0ab1886796c))
ROM_END

void msx_state::cf2700(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
}

/* MSX - National CF-3000 */

ROM_START(cf3000)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("3000bios.rom", 0x0000, 0x8000, CRC(5ad03407) SHA1(c7a2c5baee6a9f0e1c6ee7d76944c0ab1886796c))
ROM_END

void msx_state::cf3000(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - National CF-3300 */
ROM_START(cf3300)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("3300bios.rom", 0x0000, 0x8000, CRC(5ad03407) SHA1(c7a2c5baee6a9f0e1c6ee7d76944c0ab1886796c))
	ROM_LOAD("3300disk.rom", 0x8000, 0x4000, CRC(549f1d90) SHA1(f1525de4e0b60a6687156c2a96f8a8b2044b6c56))
ROM_END

void msx_state::cf3300(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: mb8877a, 1 3.5" SSDD drive
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_mb8877a(config);
	msx_1_35_ssdd_drive(config);
	msx1_floplist(config);
}

/* MSX - National FS-1300 */

ROM_START(fs1300)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("1300bios.rom", 0x0000, 0x8000, CRC(5ad03407) SHA1(c7a2c5baee6a9f0e1c6ee7d76944c0ab1886796c))
ROM_END

void msx_state::fs1300(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - National FS-4000 */

ROM_START(fs4000)
	ROM_REGION(0x18000 ,"maincpu", 0)
	ROM_LOAD("4000bios.rom",  0x0000, 0x8000, CRC(071135e0) SHA1(df48902f5f12af8867ae1a87f255145f0e5e0774))
	ROM_LOAD("4000word.rom",  0x8000, 0x8000, CRC(950b6c87) SHA1(931d6318774bd495a32ec3dabf8d0edfc9913324))
	ROM_LOAD("4000kdr.rom",  0x10000, 0x8000, CRC(ebaa5a1e) SHA1(77bd67d5d10d459d343e79eafcd8e17eb0f209dd))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("4000kfn.rom", 0, 0x20000, CRC(956dc96d) SHA1(9ed3ab6d893632b9246e91b412cd5db519e7586b))
ROM_END

void msx_state::fs4000(machine_config &config)
{
	msx1(TMS9128, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 0, 0, 2, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */
}

/* MSX - National FS-4000 (Alt) */

ROM_START(fs4000a)
	ROM_REGION(0x18000 ,"maincpu", 0)
	ROM_LOAD("4000bios.rom",  0x0000, 0x8000, CRC(071135e0) SHA1(df48902f5f12af8867ae1a87f255145f0e5e0774))
	ROM_LOAD("4000wora.rom",  0x8000, 0x8000, CRC(52f4cdf7) SHA1(acbac3cb5b700254bed2cacc19fa54f1950f371d))
	ROM_LOAD("4000kdra.rom", 0x10000, 0x8000, CRC(b2db6bf5) SHA1(3a9a942ed888dd641cddf8deada1879c454df3c6))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("4000kfn.rom", 0, 0x20000, CRC(956dc96d) SHA1(9ed3ab6d893632b9246e91b412cd5db519e7586b))
ROM_END

void msx_state::fs4000a(machine_config &config)
{
	msx1(TMS9128, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 0, 0, 2, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */
}

/*MSX - Olympia PHC-2*/

ROM_START(phc2)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("phc2bios.rom", 0x0000, 0x8000, CRC(4f7bb04b) SHA1(ab0177624d46dd77ab4f50ffcb983c3ba88223f4))
ROM_END

void msx_state::phc2(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Olympia PHC-28 */

ROM_START(phc28)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("phc28bios.rom", 0x0000, 0x8000, CRC(eceb2802) SHA1(195950173701abeb460a1a070d83466f3f53b337))
ROM_END

void msx_state::phc28(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 2, 2);   /* 32KB RAM */
}

/* MSX - Panasonic CF-2700G */

ROM_START(cf2700g)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("cf2700g.rom", 0x0000, 0x8000, CRC(4aa194f4) SHA1(69bf27b610e11437dad1f7a1c37a63179a293d12))
ROM_END

void msx_state::cf2700g(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB?? RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Perfect Perfect1 */

ROM_START(perfect1)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("perfect1bios.rom", 0x0000, 0x8000, CRC(a317e6b4) SHA1(e998f0c441f4f1800ef44e42cd1659150206cf79))
	ROM_LOAD("perfect1arab.rom", 0x8000, 0x8000, CRC(6db04a4d) SHA1(01012a0e2738708861f66b6921b2e2108f2edb54))
	ROM_RELOAD(0x10000, 0x8000)
ROM_END

void msx_state::perfect1(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 dribes
	// 1 Cartridge slot

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "arab", 0, 1, 0, 4, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 2, 0, 4); // 64KB RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
}

/* MSX - Philips NMS-801 */

ROM_START(nms801)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("801bios.rom", 0x0000, 0x8000, CRC(fa089461) SHA1(21329398c0f350e330b353f45f21aa7ba338fc8d))
ROM_END

void msx_state::nms801(machine_config &config)
{
	msx1(TMS9929A, config, NO_CARTLIST);
	// AY8910
	// FDC: None, 0 drives
	// 0 Cartridge slots
	// No printer port

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Philips VG-8000 */

ROM_START(vg8000)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("8000bios.rom", 0x0000, 0x8000, CRC(efd970b0) SHA1(42252cf87deeb58181a7bfec7c874190a1351779))
ROM_END

void msx_state::vg8000(machine_config &config)
{
	msx1(TMS9129, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// No printer port

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 3, 1);   /* 16KB RAM */
}

/* MSX - Philips VG-8010 */

ROM_START(vg8010)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("8010bios.rom", 0x0000, 0x8000, CRC(efd970b0) SHA1(42252cf87deeb58181a7bfec7c874190a1351779))
ROM_END

void msx_state::vg8010(machine_config &config)
{
	msx1(TMS9129, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// No printer port

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 2, 2);   /* 32KB RAM */
}

/* MSX - Philips VG-8010F */

ROM_START(vg8010f)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("8010fbios.rom", 0x0000, 0x8000, CRC(df57c9ca) SHA1(898630ad1497dc9a329580c682ee55c4bcb9c30c))
ROM_END

void msx_state::vg8010f(machine_config &config)
{
	msx1(TMS9129, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// No printer port

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 2, 2);   /* 32KB RAM */
}

/* MSX - Philips VG-8020-00 */

ROM_START(vg802000)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("8020-00bios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
ROM_END

void msx_state::vg802000(machine_config &config)
{
	msx1(TMS9929A, config);
	// YM2149
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Philips VG-8020-20 */

ROM_START(vg802020)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("8020-20bios.rom", 0x0000, 0x8000, CRC(a317e6b4) SHA1(e998f0c441f4f1800ef44e42cd1659150206cf79))
ROM_END

void msx_state::vg802020(machine_config &config)
{
	msx1(TMS9129, config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000);   /* 64KB Mapper RAM */
}

/* MSX - Philips VG-8020F */

ROM_START(vg8020f)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("vg8020f.rom", 0x0000, 0x8000, CRC(6e692fa1) SHA1(9eaad185efc8e224368d1db4949eb9659c26fb2c))
ROM_END

void msx_state::vg8020f(machine_config &config)
{
	msx1(TMS9929A, config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB?? RAM */
}

/* MSX - Pioneer PX-7 */

ROM_START(piopx7)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("ym2301.ic12", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
	ROM_LOAD("pd5031.ic13", 0x8000, 0x2000, CRC(91e0df72) SHA1(4f0102cdc27216fd9bcdb9663db728d2ccd8ca6d))
	ROM_FILL(0xa000, 0x2000, 0x6e)
ROM_END

void msx_state::piopx7(machine_config &config)
{
	msx1(TMS9929A, config);
	// TMS9129NL VDP with sync/overlay interface
	// AY-3-8910 PSG
	// Pioneer System Remote (SR) system control interface
	// FDC: None, 0 drives
	// 2 Cartridge slots

	// Line-level stereo audio input can be mixed with sound output, balance controlled with slider on front panel
	// Front-panel switch allows audio input to be passed through bypassing the mixing circuit
	// Line input can be muted under software control, e.g. when loading data from Laserdisc
	// Right channel of line input is additionally routed via some signal processing to the cassette input for loading data from Laserdisc

	// PSG port B bits 0-5 can be used to drive controller pins 1-6, 1-7, 2-6, 2-7, 1-8 and 2-8 low if 0 is written

	// Slot #2 7FFE is the SR control register LCON
	// Bit 7 R = /ACK (significant with acknowledge 1->0 with respect to remote control signal transmission)
	// Bit 0 R = RMCLK (clock produced by dividing CLK1/CLK2 frequency by 128)
	// Bit 0 W = /REM (high output with bit serial data output generated in synchronisation with RMCLK)

	// Slot #2 7FFF is the video overlay control register VCON
	// Bit 7 R = /EXTV (low when external video input available; high when not available)
	// Bit 7 W = Mute (line input signal muting)
	// Bit 0 R = INTEXV (interrupt available when external video signal OFF, reset on read)
	// Bit 0 W = /OVERLAY (0 = superimpose, 1 = non-superimpose)

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "rom2", 2, 0, 1, 1, "maincpu", 0x8000);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Pioneer PX-7UK */

ROM_START(piopx7uk)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("px7ukbios.rom",   0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
	ROM_LOAD("px7ukpbasic.rom", 0x8000, 0x2000, CRC(91e0df72) SHA1(4f0102cdc27216fd9bcdb9663db728d2ccd8ca6d))
	ROM_FILL(0xa000, 0x2000, 0x6e)
	ROM_LOAD("videoart.rom",    0xc000, 0x8000, CRC(0ba148dc) SHA1(b7b4e4cd40a856bb071976e6cf0f5e546fc86a78))
ROM_END

void msx_state::piopx7uk(machine_config &config)
{
	msx1(TMS9129, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "rom2", 2, 0, 1, 1, "maincpu", 0x8000);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Pioneer PX-V60 */

ROM_START(piopxv60)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("pxv60bios.rom",   0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
	ROM_LOAD("pxv60pbasic.rom", 0x8000, 0x2000, CRC(91e0df72) SHA1(4f0102cdc27216fd9bcdb9663db728d2ccd8ca6d))
	ROM_FILL(0xa000, 0x2000, 0x6E)
ROM_END

void msx_state::piopxv60(machine_config &config)
{
	msx1(TMS9128, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "rom2", 2, 0, 1, 1, "maincpu", 0x8000);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Samsung SPC-800 */

ROM_START(spc800)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("spc800bios.rom", 0x0000, 0x8000, CRC(3ab0cd3b) SHA1(171b587bd5a947a13f3114120b6e7baca3b57d78))
	ROM_LOAD("spc800han.rom",  0x8000, 0x4000, CRC(5ae2b013) SHA1(1e7616261a203580c1044205ad8766d104f1d874))
ROM_END

void msx_state::spc800(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 0, 4, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB?? RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Sanyo MPC-64 */

ROM_START(mpc64)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mpc64bios.rom", 0x0000, 0x8000, CRC(d6e704ad) SHA1(d67be6d7d56d7229418f4e122f2ec27990db7d19))
ROM_END

void msx_state::mpc64(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sanyo MPC-100 */

ROM_START(mpc100)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mpc100bios.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::mpc100(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sanyo MPC-200 */

ROM_START(mpc200)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mpc200bios.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::mpc200(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2? Cartridge slots
	// T6950
	// T7775 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4); // 64KB RAM
}

/* MSX - Sanyo MPC-200SP */

ROM_START(mpc200sp)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mpcsp200bios.rom", 0x0000, 0x8000, CRC(bcd79900) SHA1(fc8c2b69351e60dc902add232032c2d69f00e41e))
ROM_END

void msx_state::mpc200sp(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2? Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4); // 64KB RAM
}

/* MSX - Sanyo PHC-28L */

ROM_START(phc28l)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("28lbios.rom", 0x0000, 0x8000, CRC(d2110d66) SHA1(d3af963e2529662eae63f04a2530454685a1989f))
ROM_END

void msx_state::phc28l(machine_config &config)
{
	msx1(TMS9929A, config);
	// YM2149
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sanyo PHC-28S */

ROM_START(phc28s)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("28sbios.rom", 0x0000, 0x8000, CRC(e5cf6b3c) SHA1(b1cce60ef61c058f5e42ef7ac635018d1a431168))
ROM_END

void msx_state::phc28s(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 2, 2);   /* 32KB RAM */
}

/* MSX - Sanyo Wavy MPC-10 */

ROM_START(mpc10)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mpc10.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::mpc10(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 2, 2);   /* 32KB RAM */
}

/* MSX - Sharp Epcom HotBit 1.1 */

ROM_START(hotbit11)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hotbit11.rom", 0x0000, 0x8000, CRC(b6942694) SHA1(663f8c512d04d213fa616b0db5eefe3774012a4b))
ROM_END

void msx_state::hotbit11(machine_config &config)
{
	msx1(TMS9128, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sharp Epcom HotBit 1.2 */

ROM_START(hotbit12)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hotbit12.rom", 0x0000, 0x8000, CRC(f59a4a0c) SHA1(9425815446d468058705bae545ffa13646744a87))
ROM_END

void msx_state::hotbit12(machine_config &config)
{
	msx1(TMS9128, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sharp Epcom HotBit 1.3b */

ROM_START(hotbi13b)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hotbit13b.rom", 0x0000, 0x8000, CRC(7a19820e) SHA1(e0c2bfb078562d15acabc5831020a2370ea87052))
ROM_END

void msx_state::hotbi13b(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000);   /* 64KB Mapper RAM */
}

/* MSX - Sharp Epcom HotBit 1.3p */

ROM_START(hotbi13p)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hotbit13p.rom", 0x0000, 0x8000, CRC(150e239c) SHA1(942f9507d206cd8156f15601fe8032fcf0e3875b))
ROM_END

void msx_state::hotbi13p(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000);   /* 64KB Mapper RAM */
}

/* MSX - Sony HB-10 */

ROM_START(hb10)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hb10bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::hb10(machine_config &config)
{
	msx1(TMS9928A, config);
	// YM2149 (in S-1985 MSX-Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX-Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 3, 1);  /* 16KB? RAM */

	MSX_S1985(config, "s1985", 0);
}

/* MSX - Sony HB-10P */

ROM_START(hb10p)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("10pbios.rom", 0x0000, 0x8000, CRC(0f488dd8) SHA1(5e7c8eab238712d1e18b0219c0f4d4dae180420d))
ROM_END

void msx_state::hb10p(machine_config &config)
{
	msx1(TMS9929A, config);
	// XTAL: 3.579545 + 22.168(VDP)
	// YM2149 (in S3527 MSX-Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// T6950

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sony HB-20P */

ROM_START(hb20p)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("20pbios.rom", 0x0000, 0x8000, CRC(21af423f) SHA1(365c93d7652c9f727221689bcc348652832a7b7a))
ROM_END

void msx_state::hb20p(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// T6950

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sony HB-201 */

ROM_START(hb201)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("201bios.rom.ic9", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
	ROM_LOAD("201note.rom.ic8", 0x8000, 0x4000, CRC(74567244) SHA1(0f4f09f1a6ef7535b243afabfb44a3a0eb0498d9))
	ROM_FILL(0xc000, 0x4000, 0xff)
ROM_END

void msx_state::hb201(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "note", 0, 0, 2, 2, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sony HB-201P */

ROM_START(hb201p)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("201pbios.rom.ic9", 0x0000, 0x8000, CRC(0f488dd8) SHA1(5e7c8eab238712d1e18b0219c0f4d4dae180420d))
	ROM_LOAD("201pnote.rom.ic8", 0x8000, 0x4000, CRC(1ff9b6ec) SHA1(e84d3ec7a595ee36b50e979683c84105c1871857))
ROM_END

void msx_state::hb201p(machine_config &config)
{
	msx1(TMS9929A, config);
	// YM2149
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "note", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sony HB-501P */

ROM_START(hb501p)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("501pbios.rom", 0x0000, 0x8000, CRC(0f488dd8) SHA1(5e7c8eab238712d1e18b0219c0f4d4dae180420d))
ROM_END

void msx_state::hb501p(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Sony HB-55 (Version 1) */

ROM_START(hb55)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("hb55bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
	ROM_LOAD("hb55note.rom", 0x8000, 0x2000, CRC(5743ab55) SHA1(b9179db93608c4da649532e704f072e0a3ea1b22))
ROM_END

void msx_state::hb55(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "note", 0, 0, 2, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 3, 1);   /* 16KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Sony HB-55D */

ROM_START(hb55d)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("55dbios.rom", 0x0000, 0x8000, CRC(7e2b32dd) SHA1(38a645febd0e0fe86d594f27c2d14be995acc730))
	ROM_LOAD("55dnote.rom", 0x8000, 0x4000, CRC(8aae0494) SHA1(97ce59892573cac3c440efff6d74c8a1c29a5ad3))
ROM_END

void msx_state::hb55d(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "note", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 3, 1);   /* 16KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Sony HB-55P */

ROM_START(hb55p)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("55pbios.ic42", 0x0000, 0x4000, CRC(24c198be) SHA1(7f8c94cb8913db32a696dec80ffc78e46693f1b7))
	ROM_LOAD("55pbios.ic43", 0x4000, 0x4000, CRC(e516e7e5) SHA1(05fedd4b9bfcf4949020c79d32c4c3f03a54fb62))
	ROM_LOAD("55pnote.ic44", 0x8000, 0x4000, CRC(492b12f8) SHA1(b262aedc71b445303f84efe5e865cbb71fd7d952))
ROM_END

void msx_state::hb55p(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "note", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 3, 1);   /* 16KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Sony HB-75D */

ROM_START(hb75d)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("75dbios.rom", 0x0000, 0x8000, CRC(7e2b32dd) SHA1(38a645febd0e0fe86d594f27c2d14be995acc730))
	ROM_LOAD("75dnote.rom", 0x8000, 0x4000, CRC(8aae0494) SHA1(97ce59892573cac3c440efff6d74c8a1c29a5ad3))
ROM_END

void msx_state::hb75d(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "note", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Sony HB-75P */

ROM_START(hb75p)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("75pbios.ic42", 0x0000, 0x4000, CRC(24c198be) SHA1(7f8c94cb8913db32a696dec80ffc78e46693f1b7))
	ROM_LOAD("75pbios.ic43", 0x4000, 0x4000, CRC(e516e7e5) SHA1(05fedd4b9bfcf4949020c79d32c4c3f03a54fb62))
	ROM_LOAD("75pnote.ic44", 0x8000, 0x4000, CRC(492b12f8) SHA1(b262aedc71b445303f84efe5e865cbb71fd7d952))
ROM_END

void msx_state::hb75p(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "note", 0, 0, 2, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Sony HB-101P */

ROM_START(hb101p)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("101pbios.rom", 0x0000, 0x8000, CRC(0f488dd8) SHA1(5e7c8eab238712d1e18b0219c0f4d4dae180420d))
	ROM_LOAD("101pnote.rom", 0x8000, 0x4000, CRC(525017c2) SHA1(8ffc24677fd9d2606a79718764261cdf02434f0a))
ROM_END

void msx_state::hb101p(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "note", 3, 0, 1, 1, "maincpu", 0x8000);
}

/* MSX - Sony HB-701FD */

ROM_START(hb701fd)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("hb701fdbios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
	ROM_LOAD("hb701fddisk.rom", 0x8000, 0x4000, CRC(71961d9d) SHA1(2144036d6573d666143e890e5413956bfe8f66c5))
ROM_END

void msx_state::hb701fd(machine_config &config)
{
	msx1(TMS9928A, config);
	// YM2149 (in S-1985)
	// FDC: WD2793?, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1"); // Is this correct??

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx1_floplist(config);
}

/* MSX - Spectravideo SVI-728 */

ROM_START(svi728)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("728bios.rom", 0x0000, 0x8000, CRC(1ce9246c) SHA1(ea6a82cf8c6e65eb30b98755c8577cde8d9186c0))
//  ROM_LOAD("707disk.rom", 0x8000, 0x4000, CRC(f9978853) SHA1(6aa856cc56eb98863c9da7a566571605682b5c6b))
ROM_END

void msx_state::svi728(machine_config &config)
{
	msx1(TMS9129, config);
	// AY8910
	// FDC: None, 0 drives
	// 1 Cartridge slots, 1 Expansion slot (eg for SVI-707)

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot", 2, 0, msx_cart, nullptr);
//  MSX_LAYOUT_SLOT (3, 0, 1, 1, DISK_ROM2, 0x4000, 0x8000)
//  MSX_LAYOUT_SLOT (3, 1, 0, 4, CARTRIDGE2, 0x0000, 0x0000)
}

/* MSX - Spectravideo SVI-738 */

ROM_START(svi738)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("738bios.rom", 0x0000, 0x8000, CRC(ad007d62) SHA1(c53b3f2c00f31683914f7452f3f4d94ae2929c0d))
	ROM_LOAD("738disk.rom", 0x8000, 0x4000, CRC(acd27a36) SHA1(99a40266bc296cef1d432cb0caa8df1a7e570be4))
	ROM_LOAD("738232c.rom", 0xc000, 0x2000, CRC(3353dcc6) SHA1(4e9384c9d137f0ab65ffc5a78f04cd8c9df6c8b7))
	ROM_FILL(0xe000, 0x2000, 0xff)
ROM_END

void msx_state::svi738(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: wd1793, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// builtin 80 columns card (V9938)
	// RS-232C interface

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "rs232", 3, 0, 1, 1, "maincpu", 0xc000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 2, msx_cart, nullptr);

	msx_fd1793(config);
	msx_1_35_ssdd_drive(config);
	msx1_floplist(config);
}

/* MSX - Spectravideo SVI-738 Arabic */

ROM_START(svi738ar)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("738arbios.rom", 0x0000, 0x8000, CRC(ad007d62) SHA1(c53b3f2c00f31683914f7452f3f4d94ae2929c0d))
	ROM_LOAD("738ardisk.rom", 0x8000, 0x4000, CRC(acd27a36) SHA1(99a40266bc296cef1d432cb0caa8df1a7e570be4))
	ROM_LOAD("738ar232c.rom", 0xc000, 0x2000, CRC(3353dcc6) SHA1(4e9384c9d137f0ab65ffc5a78f04cd8c9df6c8b7))
	ROM_FILL(0xe000, 0x2000, 0xff)
	ROM_LOAD("738arab.rom",  0x10000, 0x8000, CRC(339cd1aa) SHA1(0287b2ec897b9196788cd9f10c99e1487d7adbbb))
ROM_END

void msx_state::svi738ar(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: wd2793, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// builtin 80 columns card (V9938)
	// RS-232C interface

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "rs232", 3, 0, 1, 1, "maincpu", 0xc000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 2, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "arab", 3, 3, 1, 2, "maincpu", 0x10000);

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx1_floplist(config);
}

/* MSX - Spectravideo SVI-738 Danish */

ROM_START(svi738dk)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("738dkbios.rom", 0x0000, 0x8000, CRC(88720320) SHA1(1bda5af20cb86565bdc1ebd1e59a691fed7f9256))
	ROM_LOAD("738dkdisk.rom", 0x8000, 0x4000, CRC(fb884df4) SHA1(6d3a530ae822ec91f6444c681c9b08b9efadc7e7))
	ROM_LOAD("738dk232c.rom", 0xc000, 0x2000, CRC(3353dcc6) SHA1(4e9384c9d137f0ab65ffc5a78f04cd8c9df6c8b7))
	ROM_FILL(0xe000, 0x2000, 0xff)
ROM_END

void msx_state::svi738dk(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: wd2793, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// builtin 80 columns card (V9938)
	// RS-232C interface

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "rs232", 3, 0, 1, 1, "maincpu", 0xc000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 2, msx_cart, nullptr);

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx1_floplist(config);
}

/* MSX - Spectravideo SVI-738 Spanish */

ROM_START(svi738sp)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("738spbios.rom", 0x0000, 0x8000, CRC(f0c0cbb9) SHA1(5f04d5799ed72ea4993e7c4302a1dd55ac1ea8cd))
	ROM_LOAD("738spdisk.rom", 0x8000, 0x4000, CRC(fb884df4) SHA1(6d3a530ae822ec91f6444c681c9b08b9efadc7e7))
	ROM_LOAD("738sp232c.rom", 0xc000, 0x2000, CRC(3353dcc6) SHA1(4e9384c9d137f0ab65ffc5a78f04cd8c9df6c8b7))
	ROM_FILL(0xe000, 0x2000, 0xff)
ROM_END

void msx_state::svi738sp(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: wd2793, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// builtin 80 columns card (V9938)
	// RS-232C interface

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "rs232", 3, 0, 1, 1, "maincpu", 0xc000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 2, msx_cart, nullptr);

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx1_floplist(config);
}

/* MSX - Spectravideo SVI-738 Swedish */

ROM_START(svi738sw)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("738sebios.rom", 0x0000, 0x8000, CRC(c8ccdaa0) SHA1(87f4d0fa58cfe9cef818a3185df2735e6da6168c))
	ROM_LOAD("738sedisk.rom", 0x8000, 0x4000, CRC(fb884df4) SHA1(6d3a530ae822ec91f6444c681c9b08b9efadc7e7))
	ROM_LOAD("738se232c.rom", 0xc000, 0x2000, CRC(3353dcc6) SHA1(4e9384c9d137f0ab65ffc5a78f04cd8c9df6c8b7))
	ROM_FILL(0xe000, 0x2000, 0xff)
ROM_END

void msx_state::svi738sw(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: wd2793, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// builtin 80 columns card (V9938)
	// RS-232C interface

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "rs232", 3, 0, 1, 1, "maincpu", 0xc000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 2, msx_cart, nullptr);

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx1_floplist(config);
}

/* MSX - Spectravideo SVI-738 Poland*/

ROM_START(svi738pl)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("738plbios.rom", 0x0000, 0x8000, CRC(431b8bf5) SHA1(c90077ed84133a947841e07856e71133ba779da6)) // IC51 on board
	ROM_LOAD("738disk.rom",   0x8000, 0x4000, CRC(acd27a36) SHA1(99a40266bc296cef1d432cb0caa8df1a7e570be4))
	ROM_LOAD("738232c.rom",   0xc000, 0x2000, CRC(3353dcc6) SHA1(4e9384c9d137f0ab65ffc5a78f04cd8c9df6c8b7))
	ROM_FILL(0xe000, 0x2000, 0xff)
ROM_END

void msx_state::svi738pl(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: wd2793, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// builtin 80 columns card (V9938)
	// RS-232C interface

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "rs232", 3, 0, 1, 1, "maincpu", 0xc000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 2, msx_cart, nullptr);

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx1_floplist(config);
}

/* MSX - Talent DPC-200 */

ROM_START(tadpc200)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("dpc200bios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
ROM_END

void msx_state::tadpc200(machine_config &config)
{
	msx1(TMS9129, config);
	// AY8910
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Talent DPC-200A */

ROM_START(tadpc20a)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("dpc200abios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
ROM_END

void msx_state::tadpc20a(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 1, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Toshiba HX-10 */
/* The BIOS on the Toshiba HX-10 is inside a big 64pin Toshiba chip label TCX-1007 */

ROM_START(hx10)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("tcx-1007.ic15", 0x0000, 0x8000, CRC(5486b711) SHA1(4dad9de7c28b452351cc12910849b51bd9a37ab3))
ROM_END

void msx_state::hx10(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910
	// FDC: None, 0 drives
	// 1 Cartridge slot, 1 Toshiba Expension slot

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	//MSX_LAYOUT_SLOT (3, 0, 0, 4, CARTRIDGE2, 0x0000, 0x0000)    // Expansion slot
}

/* MSX - Toshiba HX-10D */

ROM_START(hx10d)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hx10dbios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::hx10d(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);   /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Toshiba HX-10DP */

ROM_START(hx10dp)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hx10dpbios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::hx10dp(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);   /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Toshiba HX-10E */

ROM_START(hx10e)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hx10ebios.rom", 0x0000, 0x8000, CRC(5486b711) SHA1(4dad9de7c28b452351cc12910849b51bd9a37ab3))
ROM_END

void msx_state::hx10e(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);   /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Toshiba HX-10F */

ROM_START(hx10f)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hx10fbios.rom", 0x0000, 0x8000, CRC(e0e894b7) SHA1(d99eebded5db5fce1e072d08e642c0909bc7efdd))
ROM_END

void msx_state::hx10f(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);   /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Toshiba HX-10S */

ROM_START(hx10s)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hx10sbios.rom", 0x0000, 0x8000, CRC(5486b711) SHA1(4dad9de7c28b452351cc12910849b51bd9a37ab3))
ROM_END

void msx_state::hx10s(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 3, 1);   /* 16KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Toshiba HX-10SA */

ROM_START(hx10sa)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hx10sabios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::hx10sa(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);   /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Toshiba HX-20 */

ROM_START(hx20)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("hx20bios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
	ROM_LOAD("hx20word.rom", 0x8000, 0x8000, CRC(39b3e1c0) SHA1(9f7cfa932bd7dfd0d9ecaadc51655fb557c2e125))
ROM_END

void msx_state::hx20(machine_config &config)
{
	msx1(TMS9129, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// T6950

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram1", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 3, 0, 2, 2);   /* 32KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 3, 1, 2, "maincpu", 0x8000);
}

/* MSX - Toshiba HX-20I */

ROM_START(hx20i)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("hx20ibios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
	ROM_LOAD("hx20iword.rom", 0x8000, 0x8000, CRC(39b3e1c0) SHA1(9f7cfa932bd7dfd0d9ecaadc51655fb557c2e125))
ROM_END

void msx_state::hx20i(machine_config &config)
{
	msx1(TMS9129, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// T6950 VDP instead of TMS9928A

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram1", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 3, 0, 2, 2);   /* 32KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 3, 1, 2, "maincpu", 0x8000);
}

/* MSX - Toshiba HX-21 */

ROM_START(hx21)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("hx21bios.rom", 0x0000, 0x8000, CRC(83ba6fde) SHA1(01600d06d83072d4e757b29728555efde2c79705))
	ROM_LOAD("hx21word.rom", 0x8000, 0x8000, CRC(87508e78) SHA1(4e2ec9c0294a18a3ab463f182f9333d2af68cdca))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("hx21kfn.rom", 0x0000, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx_state::hx21(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 3, 0, 0, 4);   /* 64KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 3, 1, 2, "maincpu", 0x8000);
}

/* MSX - Toshiba HX-21I */

ROM_START(hx21i)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("hx21ibios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
	ROM_LOAD("hx21iword.rom", 0x8000, 0x8000, CRC(f9e29c66) SHA1(3289336b2c12161fd926a7e5ce865770ae7038af))
ROM_END

void msx_state::hx21i(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram1", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 3, 0, 2, 2);   /* 32KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 3, 1, 2, "maincpu", 0x8000);
}

/* MSX - Toshiba HX-22 */

ROM_START(hx22)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("hx22bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
	ROM_LOAD("hx22word.rom", 0x8000, 0x8000, CRC(87508e78) SHA1(4e2ec9c0294a18a3ab463f182f9333d2af68cdca))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("hx22kfn.rom", 0x0000, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx_state::hx22(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// RS232C builtin?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 3, 0, 0, 4);   /* 64KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 3, 1, 2, "maincpu", 0x8000);
}

/* MSX - Toshiba HX-22I */

ROM_START(hx22i)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("hx22ibios.rom", 0x0000, 0x8000, CRC(8205795e) SHA1(829c00c3114f25b3dae5157c0a238b52a3ac37db))
	ROM_LOAD("hx22iword.rom", 0x8000, 0x8000, CRC(f9e29c66) SHA1(3289336b2c12161fd926a7e5ce865770ae7038af))
ROM_END

void msx_state::hx22i(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// RS232C builtin?
	// Z80: LH0080A

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram1", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 3, 0, 2, 2);   /* 32KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 3, 1, 2, "maincpu", 0x8000);
}

/* MSX - Victor HC-5 */

ROM_START(hc5)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hc5bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::hc5(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives,
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 3, 1); // 16KB or 32KB RAM ?
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Victor HC-6 */

ROM_START(hc6)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hc6bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::hc6(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives,
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 2, 2); // 32KB RAM
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Victor HC-7 */

ROM_START(hc7)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("hc7bios.rom", 0x0000, 0x8000, CRC(ee229390) SHA1(302afb5d8be26c758309ca3df611ae69cced2821))
ROM_END

void msx_state::hc7(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives,
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4); // 64KB RAM
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX - Yamaha CX5F (with SFG01) */

ROM_START(cx5f1)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("cx5fbios.rom", 0x0000, 0x8000, CRC(dc662057) SHA1(36d77d357a5fd15af2ab266ee66e5091ba4770a3))
ROM_END

void msx_state::cx5f1(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 1 Cartridge slot?
	// 1 Yamaha expansion slot?
	// S-5327 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 2, 2); // 32KB RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 2, 0, msx_yamaha_60pin, "sfg01");
}

/* MSX - Yamaha CX5F (with SFG05) */

ROM_START(cx5f)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("cx5fbios.rom", 0x0000, 0x8000, CRC(dc662057) SHA1(36d77d357a5fd15af2ab266ee66e5091ba4770a3))
ROM_END

void msx_state::cx5f(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 1 Cartridge slot?
	// 1 Yamaha expansion slot?
	// S-5327 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 0, msx_yamaha_60pin, "sfg05");
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 2, 2); // 32KB RAM
}

/* MSX - Yamaha CX5M / Yamaha CX5M-2 */

ROM_START(cx5m)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("cx5mbios.rom", 0x0000, 0x8000, CRC(e2242b53) SHA1(706dd67036baeec7127e4ccd8c8db8f6ce7d0e4c))
ROM_END

void msx_state::cx5m(machine_config &config)
{
	msx1(TMS9929A, config);
	// YM2149
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// 1 Expansion slot (60 pins interface instead of regular 50 pin cartridge interface)

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 0, msx_yamaha_60pin, "sfg01");
}

/* MSX - Yamaha CX5M-128 */

ROM_START(cx5m128)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("cx5m128bios.rom", 0x0000, 0x8000, CRC(507b2caa) SHA1(0dde59e8d98fa524961cd37b0e100dbfb42cf576))
	ROM_LOAD("cx5m128ext.rom",  0x8000, 0x4000, CRC(feada82e) SHA1(48b0c2ff1f1e407cc44394219f7b3878efaa919f))
	ROM_LOAD("yrm502.rom",     0x14000, 0x4000, CRC(5330fe21) SHA1(7b1798561ee1844a7d6432924fbee9b4fc591c19))
ROM_END

void msx_state::cx5m128(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 1, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "yrm", 3, 1, 1, 1, "maincpu", 0x14000); /* YRM-502 */
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000);   /* 128KB Mapper RAM */
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 3, msx_yamaha_60pin, "sfg05");
}

/* MSX - Yamaha CX5MII */

ROM_START(cx5m2)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("cx5m2bios.rom", 0x0000, 0x8000, CRC(507b2caa) SHA1(0dde59e8d98fa524961cd37b0e100dbfb42cf576))
	ROM_LOAD("cx5m2ext.rom",  0x8000, 0x4000, CRC(feada82e) SHA1(48b0c2ff1f1e407cc44394219f7b3878efaa919f))
ROM_END

void msx_state::cx5m2(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 1, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000);   /* 64KB Mapper RAM */
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 3, msx_yamaha_60pin, "sfg05");
}

/* MSX - Yamaha YIS303 */

ROM_START(yis303)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("yis303bios.rom", 0x0000, 0x8000, CRC(e2242b53) SHA1(706dd67036baeec7127e4ccd8c8db8f6ce7d0e4c))
	ROM_FILL(0x8000, 0xc000, 0xff)
ROM_END

void msx_state::yis303(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "fillff", 3, 0, 0, 3, "maincpu", 0x0000);   /* Fill FF */
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 3, 1);   /* 16KB RAM */
}

/* MSX - Yamaha YIS503 */

ROM_START(yis503)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("yis503bios.rom", 0x0000, 0x8000, CRC(e2242b53) SHA1(706dd67036baeec7127e4ccd8c8db8f6ce7d0e4c))
	ROM_FILL(0x8000, 0xc000, 0xff)
ROM_END

void msx_state::yis503(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "fillff", 3, 0, 0, 3, "maincpu", 0x0000);   /* Fill FF */
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 2, 2);   /* 32KB RAM */
}

/* MSX - Yamaha YIS503F */

ROM_START(yis503f)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("yis503f.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::yis503f(machine_config &config)
{
	msx1(TMS9929A, config);
	// YM2149
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB?? RAM */
}

/* MSX - Yamaha YIS503II */

ROM_START(yis503ii)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("yis503iibios.rom", 0x0000, 0x8000, CRC(e2242b53) SHA1(706dd67036baeec7127e4ccd8c8db8f6ce7d0e4c))
ROM_END

void msx_state::yis503ii(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Yamaha YIS503IIR Russian */

ROM_START(y503iir)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("yis503iirbios.rom", 0x0000, 0x8000, CRC(225a4f9e) SHA1(5173ac403e26c462f904f85c9ef5e7b1e19253e7))
	ROM_LOAD("yis503iirdisk.rom", 0x8000, 0x4000, CRC(9eb7e24d) SHA1(3a481c7b7e4f0406a55952bc5b9f8cf9d699376c))
	ROM_LOAD("yis503iirnet.rom",  0xc000, 0x2000, CRC(0731db3f) SHA1(264fbb2de69fdb03f87dc5413428f6aa19511a7f))
ROM_END

void msx_state::y503iir(machine_config &config)
{
	msx1(TMS9929A, config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793/mb8877?, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine
	// RTC
	// V9938 VDP

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1"); /* National disk */
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "net", 3, 3, 1, 1, "maincpu", 0xc000);   /* Net */

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx1_floplist(config);
}

/* MSX - Yamaha YIS503IIR Estonian */

ROM_START(y503iir2)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("yis503ii2bios.rom", 0x0000, 0x8000, CRC(1548cee3) SHA1(42c7fff25b1bd90776ac0aea971241aedce8947d))
	ROM_LOAD("yis503iirdisk.rom", 0x8000, 0x4000, CRC(9eb7e24d) SHA1(3a481c7b7e4f0406a55952bc5b9f8cf9d699376c))
	ROM_LOAD("yis503iirnet.rom",  0xc000, 0x2000, CRC(0731db3f) SHA1(264fbb2de69fdb03f87dc5413428f6aa19511a7f))
ROM_END

void msx_state::y503iir2(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: wd2793/mb8877?, 1 3.5" DSDD drive?
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1"); /* National disk */
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "net", 3, 3, 1, 1, "maincpu", 0xc000);   /* Net */

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx1_floplist(config);
}

/* MSX - Yamaha YIS503M */

ROM_START(yis503m)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("yis503mbios.rom", 0x0000, 0x8000, CRC(e2242b53) SHA1(706dd67036baeec7127e4ccd8c8db8f6ce7d0e4c))
ROM_END

void msx_state::yis503m(machine_config &config)
{
	msx1(TMS9929A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 0, msx_yamaha_60pin, "sfg05");
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 2, 2);   /* 32KB RAM */
}

/* MSX - Yashica YC-64 */

ROM_START(yc64)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("yc64bios.rom", 0x0000, 0x8000, CRC(e9ccd789) SHA1(8963fc041975f31dc2ab1019cfdd4967999de53e))
ROM_END

void msx_state::yc64(machine_config &config)
{
	msx1(TMS9929A, config);
	// YM2149
	// FDC: None, 0 drives
	// 1 Cartridge slot (slot 1)

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
}

/* MSX - Yeno MX64 */

ROM_START(mx64)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("mx64bios.rom", 0x0000, 0x8000, CRC(e0e894b7) SHA1(d99eebded5db5fce1e072d08e642c0909bc7efdd))
ROM_END

void msx_state::mx64(machine_config &config)
{
	msx1(TMS9928A, config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 2, 0, 0, 4);  /* 64KB RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}


/********************************  MSX 2 **********************************/

/* MSX2 - Al Alamiah AX-350 */

ROM_START(ax350)
	ROM_REGION(0x30000, "maincpu", 0)
	ROM_LOAD("ax350bios.rom",  0x00000,  0x8000, CRC(ea306155) SHA1(35195ab67c289a0b470883464df66bc6ea5b00d3))
	ROM_LOAD("ax350ext.rom",   0x08000,  0x4000, CRC(7c7540b7) SHA1(ebb76f9061e875365023523607db610f2eda1d26))
	ROM_LOAD("ax350arab.rom",  0x0c000,  0x8000, CRC(c0d8fc85) SHA1(2c9600c6e0025fee10d249e97448ecaa37e38c42))
	ROM_LOAD("ax350swp.rom",   0x14000,  0x8000, CRC(076f40fc) SHA1(4b4508131dca6d811694ae6379f41364c477de58))
	ROM_LOAD("ax350paint.rom", 0x1c000, 0x10000, CRC(18956e3a) SHA1(ace202e87337fbc54fea21e22c0b3af0abe6f4ae))
	ROM_LOAD("ax350disk.rom",  0x2c000,  0x4000, CRC(1e7d6512) SHA1(78cd7f847e77fd8cd51a647efb2725ba93f4c471))
ROM_END

void msx2_state::ax350(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793/tc8566af?, 1 3.5" DSDD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);  /* Bios */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);  /* Ext */
	add_internal_slot(config, MSX_SLOT_ROM, "arab", 0, 1, 1, 2, "maincpu", 0xc000);  /* Arab */
	add_internal_slot(config, MSX_SLOT_ROM, "swp", 0, 2, 1, 2, "maincpu", 0x14000); /* SWP */
	add_internal_slot(config, MSX_SLOT_ROM, "paint", 0, 3, 0, 4, "maincpu", 0x1c000);  /* Paint */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 1, 1, 2, "maincpu", 0x2c000).set_tags("fdc", "fdc:0", "fdc:1"); /* Disk */
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Al Alamiah AX-370 */

ROM_START(ax370)
	ROM_REGION(0x30000, "maincpu", 0)
	ROM_LOAD("ax370bios.rom",   0x0000,  0x8000, CRC(ea306155) SHA1(35195ab67c289a0b470883464df66bc6ea5b00d3))
	ROM_LOAD("ax370ext.rom",    0x8000,  0x4000, CRC(7c7540b7) SHA1(ebb76f9061e875365023523607db610f2eda1d26))
	ROM_LOAD("ax370arab.rom",   0xc000,  0x8000, CRC(c0d8fc85) SHA1(2c9600c6e0025fee10d249e97448ecaa37e38c42))
	ROM_LOAD("ax370swp.rom",   0x14000,  0x8000, CRC(076f40fc) SHA1(4b4508131dca6d811694ae6379f41364c477de58))
	ROM_LOAD("ax370paint.rom", 0x1c000, 0x10000, CRC(18956e3a) SHA1(ace202e87337fbc54fea21e22c0b3af0abe6f4ae))
	ROM_LOAD("ax370disk.rom",  0x2c000,  0x4000, CRC(60f8baba) SHA1(95de8809d2758fc0a743390ea5085b602e59e101))
ROM_END

void msx2_state::ax370(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);  /* Bios */
	add_internal_slot(config, MSX_SLOT_ROM, "swp", 0, 2, 1, 2, "maincpu", 0x14000); /* SWP */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);  /* Ext */
	add_internal_slot(config, MSX_SLOT_ROM, "arab", 3, 1, 1, 2, "maincpu", 0xc000);  /* Arab */
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0x2c000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "paint", 3, 3, 0, 4, "maincpu", 0x1c000);  /* Paint */

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Canon V-25 */

ROM_START(canonv25)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("v25bios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("v25ext.rom", 0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
ROM_END

void msx2_state::canonv25(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX Engine
	// 64KB VRAM

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000); // BIOS
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000); // EXT
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM

	MSX_S1985(config, "s1985", 0);

	msx2_64kb_vram(config);
}

/* MSX2 - Canon V-30 */

ROM_START(canonv30)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("v30bios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("v30ext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("v30disk.rom", 0xc000, 0x4000, CRC(54c73ad6) SHA1(12f2cc79b3d09723840bae774be48c0d721ec1c6))
ROM_END

void msx2_state::canonv30(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: ??, 2 3.5" DSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000); // BIOS
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000); // EXT
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1"); // DISK
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000); // 64KB?? Mapper RAM

	MSX_S1985(config, "s1985", 0);

	msx_wd2793(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Canon V-30F */

ROM_START(canonv30f)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("v30fbios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("v30fext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("v30fdisk.rom", 0xc000, 0x4000, CRC(54c73ad6) SHA1(12f2cc79b3d09723840bae774be48c0d721ec1c6))
ROM_END

void msx2_state::canonv30f(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: ??, 2 3.5" DSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000); // BIOS
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000); // EXT
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1"); // DISK
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 128KB Mapper RAM

	MSX_S1985(config, "s1985", 0);

	msx_wd2793(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Daewoo CPC-300 */

ROM_START(cpc300)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("300bios.rom", 0x0000, 0x8000, CRC(53850907) SHA1(affa3c5cd8db79a1450ad8a7f405a425b251653d))
	ROM_LOAD("300ext.rom",  0x8000, 0x8000, CRC(d64da39c) SHA1(fb51c505adfbc174df94289fa894ef969f5357bc))
	ROM_LOAD("300han.rom", 0x10000, 0x8000, CRC(e78cd87f) SHA1(47a9d9a24e4fc6f9467c6e7d61a02d45f5a753ef))
ROM_END

void msx2_state::cpc300(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 3, 0, 2, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Daewoo CPC-300E */

// These roms apparently came from a hacked set, see: http://www.vik.cc/bluemsx/blueforum/viewtopic.php?t=1569&start=75
ROM_START(cpc300e)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("300ebios.rom", 0x0000, 0x8000, BAD_DUMP CRC(53850907) SHA1(affa3c5cd8db79a1450ad8a7f405a425b251653d))
	ROM_LOAD("300eext.rom",  0x8000, 0x8000, BAD_DUMP CRC(d64da39c) SHA1(fb51c505adfbc174df94289fa894ef969f5357bc))
	ROM_LOAD("300ehan.rom", 0x10000, 0x4000, BAD_DUMP CRC(5afea78d) SHA1(f08c91f8c78d681e1f02eaaaaafb87ad81112b60))
ROM_END

void msx2_state::cpc300e(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?
	// No clockchip
	// No joystick port??

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 1, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 3, 0, 2, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
}

/* MSX2 - Daewoo CPC-330K */

ROM_START(cpc330k)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("330kbios.rom", 0x0000, 0x8000, CRC(53850907) SHA1(affa3c5cd8db79a1450ad8a7f405a425b251653d))
	ROM_LOAD("330kext.rom",  0x8000, 0x8000, CRC(5d685cca) SHA1(97afbadd8fe34ab658cce8222a27cdbe19bcef39))
	ROM_LOAD("330khan.rom", 0x10000, 0x4000, CRC(3d6dd335) SHA1(d2b058989a700ca772b9591f42c01ed0f45f74d6))
ROM_END

void msx2_state::cpc330k(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX Engine
	// Ergonomic keyboard, 2 builtin game controllers
	// builtin FM?? builtin SCC??

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 1, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x20000);   /* 128KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 3, 0, 2, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Daewoo CPC-400 */

ROM_START(cpc400)
	ROM_REGION(0x1c000, "maincpu", 0)
	ROM_LOAD("400bios.rom", 0x0000, 0x8000, CRC(53850907) SHA1(affa3c5cd8db79a1450ad8a7f405a425b251653d))
	ROM_LOAD("400disk.rom", 0x8000, 0x4000, CRC(5fa517df) SHA1(914f6ccb25d78621186001f2f5e2aaa2d628cd0c))
	ROM_LOAD("400ext.rom",  0xc000, 0x8000, CRC(2ba104a3) SHA1(b6d3649a6647fa9f6bd61efc317485a20901128f))
	ROM_LOAD("400han.rom", 0x14000, 0x8000, CRC(a8ead5e3) SHA1(87936f808423dddfd00629056d6807b4be1dc63e))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("400kfn.rom", 0, 0x20000, CRC(b663c605) SHA1(965f4982790f1817bcbabbb38c8777183b231a55))
ROM_END

void msx2_state::cpc400(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: mb8877a, 1 3.5" DS?DD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 1, 1, 2, "maincpu", 0x14000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 3, 0, 2, "maincpu", 0xc000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 2, 0, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);

	msx_mb8877a(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Daewoo CPC-400S */

ROM_START(cpc400s)
	ROM_REGION(0x1c000, "maincpu", 0)
	ROM_LOAD("400sbios.rom", 0x0000, 0x8000, CRC(53850907) SHA1(affa3c5cd8db79a1450ad8a7f405a425b251653d))
	ROM_LOAD("400sdisk.rom", 0x8000, 0x4000, CRC(5fa517df) SHA1(914f6ccb25d78621186001f2f5e2aaa2d628cd0c))
	ROM_LOAD("400sext.rom",  0xc000, 0x8000, CRC(2ba104a3) SHA1(b6d3649a6647fa9f6bd61efc317485a20901128f))
	ROM_LOAD("400shan.rom", 0x14000, 0x8000, CRC(975e7a31) SHA1(6a50295ea35e720ba6f4ba5616c3441128b384ed))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("400skfn.rom", 0, 0x20000, CRC(fa85368c) SHA1(30fff22e3e3d464993707488442721a5e56a9707))
ROM_END

void msx2_state::cpc400s(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: mb8877a, 1 3.5" DS?DD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "han", 0, 1, 1, 2, "maincpu", 0x14000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 3, 0, 2, "maincpu", 0xc000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 2, 0, 1, 2, "maincpu", 0x8000).set_tags("fdc", "fdc:0", "fdc:1");
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);

	MSX_S1985(config, "s1985", 0);

	msx_mb8877a(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Daewoo Zemmix CPC-61 */

ROM_START(cpc61)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("61bios.rom", 0x0000, 0x8000, CRC(b80c8e45) SHA1(310a02a9746bc062834e0cf2fabf7f3e0f7e829e))
	ROM_LOAD("61ext.rom",  0x8000, 0x8000, CRC(b3d740b4) SHA1(7121c3c5ee6e4931fceda14a06f4c0e3b8eda437))

	ROM_REGION(0x40000, "kanji", 0)
	ROM_LOAD("61kfn.rom", 0x00000, 0x40000, CRC(9a3cf67c) SHA1(22d5f4e522250dc5eb17e15b68f4a51bb752cba1))
ROM_END

void msx2_state::cpc61(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: None, 0 drives
	// 1 Cartridge slot
	// S-1985 MSX Engine
	// No clock chip
	// No keyboard, but a keyboard connector
	// No printer port
	// No cassette port?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM?
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 3, 0, 2, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Daewoo Zemmix CPG-120 Normal */

ROM_START(cpg120)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("cpg120bios.rom",   0x0000, 0x8000, CRC(b80c8e45) SHA1(310a02a9746bc062834e0cf2fabf7f3e0f7e829e))
	ROM_LOAD("cpg120ext.rom",    0x8000, 0x8000, CRC(b3d740b4) SHA1(7121c3c5ee6e4931fceda14a06f4c0e3b8eda437))
	ROM_LOAD("cpg128music.rom", 0x10000, 0x4000, CRC(73491999) SHA1(b9ee4f30a36e283a2b1b9a28a70ab9b9831570c6))

	ROM_REGION(0x40000, "kanji", 0)
	ROM_LOAD("cpg120kfn.rom", 0x0000, 0x40000, CRC(9a3cf67c) SHA1(22d5f4e522250dc5eb17e15b68f4a51bb752cba1))
ROM_END

void msx2_state::cpg120(machine_config &config)
{
	msx2(config);
	// YM2149 (in S1985)
	// FDC: None, 0 drives
	// 2 Cartridge slots?
	// S-1985 MSX Engine
	// V9958 VDP
	// FM built in
	// No keyboard, bot a keyboard connector?
	// No clock chip
	// No printer port

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x20000); // 128KB Mapper RAM
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 3, 0, 2, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 2, 0, 1, 1, "maincpu", 0x10000).set_ym2413_tag("ym2413");
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);

	MSX_S1985(config, "s1985", 0);

	msx_ym2413(config);
}

/* MSX2 - Daewoo Zemmic CPG-120 Turbo */
/* Same as normal CPG-120 but with CPU running at 5.369317 MHz */

/* MSX2 - Fenner FPC-900 */

ROM_START(fpc900)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("fpc900bios.rom", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("fpc900ext.rom",  0x8000, 0x4000, CRC(66237ecf) SHA1(5c1f9c7fb655e43d38e5dd1fcc6b942b2ff68b02))
	ROM_LOAD("fpc900disk.rom", 0xc000, 0x4000, CRC(ca3307d3) SHA1(c3efedda7ab947a06d9345f7b8261076fa7ceeef))
ROM_END

void msx2_state::fpc900(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: WD2793?, 1 3.5" DSDD drive
	// 2? Cartridge slots
	// S-3527 MSX Engine
	// 256KB?? VRAM

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x40000); // 256KB? Mapper RAM
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Gradiente Expert 2.0 */

ROM_START(expert20)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("exp20bios.rom",     0x0000, 0x8000, CRC(6bacdce4) SHA1(9c43106dba3ae2829e9a11dffa9d000ed6d6454c))
	ROM_LOAD("exp20ext.rom",      0x8000, 0x4000, CRC(08ced880) SHA1(4f2a7e0172f0214f025f23845f6e053d0ffd28e8))
	ROM_LOAD("xbasic2.rom",       0xc000, 0x4000, CRC(2825b1a0) SHA1(47370bec7ca1f0615a54eda548b07fbc0c7ef398))
	ROM_LOAD("microsoldisk.rom", 0x10000, 0x4000, CRC(6704ef81) SHA1(a3028515ed829e900cc8deb403e17b09a38bf9b0))
ROM_END

void msx2_state::expert20(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: microsol, 1? 3.5"? DS?DD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 1, 1, 0, 1, "maincpu", 0x8000); /* EXT */
	add_internal_slot(config, MSX_SLOT_ROM, "xbasic", 1, 1, 1, 1, "maincpu", 0xc000); /* XBASIC */
	add_internal_slot(config, MSX_SLOT_DISK5, "disk", 1, 3, 1, 1, "maincpu", 0x10000).set_tags("fdc", "fdc:0", "fdc:1", "fdc:2", "fdc:3"); /* Microsol controller */
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 2, 0, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);

	msx_microsol(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Hitachi MB-H70 */

ROM_START(mbh70)
	ROM_REGION(0x110000, "maincpu", 0)
	ROM_LOAD("mbh70bios.rom"    , 0x0000,   0x8000, CRC(a27c563d) SHA1(c1e46c00f1e38fc9e0ab487bf0513bd93ce61f3f))
	ROM_LOAD("mbh70ext.rom",      0x8000,   0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("mbh70disk.rom",     0xc000,   0x4000, CRC(05661a3f) SHA1(e695fc0c917577a3183901a08ca9e5f9c60b8317))
	ROM_LOAD("mbh70halnote.rom", 0x10000, 0x100000, CRC(40313fec) SHA1(1af617bfd11b10a71936c606174a80019762ea71))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("mbh70kfn.rom", 0x0000, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx2_state::mbh70(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985)
	// FDC: WD2793?, 1? 3.5" DSDD drive
	// S-1985 MSX Engine
	// 2? Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_SONY08, "firm", 0, 3, 0, 4, "maincpu", 0x10000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 128KB Mapper RAM

	MSX_S1985(config, "s1985", 0);

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Kawai KMC-5000 */

ROM_START(kmc5000)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("kmc5000bios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("kmc5000ext.rom",   0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("kmc5000disk.rom",  0xc000, 0x4000, CRC(e25cacca) SHA1(607cfca605eaf82e3efa33459d6583efb7ecc13b))
	ROM_LOAD("kmc5000kdr.rom",  0x10000, 0x8000, CRC(2dbea5ec) SHA1(ea35cc2cad9cfdf56cae224d8ee41579de37f000))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("kmc5000kfn.rom", 0, 0x20000, CRC(c61ddc5d) SHA1(5e872d5853698731a0ed22fb72dbcdfd59cd19c3))
ROM_END

void msx2_state::kmc5000(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985)
	// FDC: TC8566AF?, 1? 3.5" DSDD drive
	// S-1985 MSX Engine
	// 2? Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Mitsubishi ML-G1 */

ROM_START(mlg1)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("mlg1bios.rom",  0x0000, 0x8000, CRC(0cc7f817) SHA1(e4fdf518a8b9c8ab4290c21b83be2c347965fc24))
	ROM_LOAD("mlg1ext.rom",   0x8000, 0x4000, CRC(dc0951bd) SHA1(1e9a955943aeea9b1807ddf1250ba6436d8dd276))
	ROM_LOAD("mlg1paint.rom", 0xc000, 0x8000, CRC(64df1750) SHA1(5cf0abca6dbcf940bc33c433ecb4e4ada02fbfe6))
ROM_END

void msx2_state::mlg1(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985)
	// FDC: None, 0 drives
	// S-1985 MSX Engine
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 64KB or 128KB Mapper RAM ?
	add_internal_slot(config, MSX_SLOT_ROM, "paint", 3, 3, 0, 2, "maincpu", 0xc000);

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Mitsubishi ML-G3 */

ROM_START(mlg3)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("mlg3bios.rom",    0x0000, 0x8000, CRC(0cc7f817) SHA1(e4fdf518a8b9c8ab4290c21b83be2c347965fc24))
	ROM_LOAD("mlg3ext.rom",     0x8000, 0x4000, CRC(dc0951bd) SHA1(1e9a955943aeea9b1807ddf1250ba6436d8dd276))
	ROM_LOAD("mlg3disk.rom",    0xc000, 0x4000, CRC(b94ebc7a) SHA1(30ba1144c872a0bb1c91768e75a2c28ab1f4e3c6))
	ROM_LOAD("mlg3rs232c.rom", 0x10000, 0x4000, CRC(90b8a114) SHA1(dc50f2c9db233e505b0981c244ff3de553ac9d68))
ROM_END

void msx2_state::mlg3(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985)
	// FDC: wd2793?, 1 3.5" DSDD drive
	// S-1985 MSX Engine
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 64KB or 128KB Mapper RAM?
	//add_internal_slot(config, MSX_SLOT_ROM, "rs232c", 3, 3, 1, 1, "maincpu", 0x10000);

	MSX_S1985(config, "s1985", 0);

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Mitsubishi ML-G10 */

ROM_START(mlg10)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("mlg10bios.rom", 0x0000, 0x8000, CRC(a27c563d) SHA1(c1e46c00f1e38fc9e0ab487bf0513bd93ce61f3f))
	ROM_LOAD("mlg10ext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("mlg10kfn.rom", 0, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx2_state::mlg10(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985)
	// FDC: None, 0 drives
	// S-1985 MSX Engine
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 64KB or 128KB Mapper RAM?

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Mitsubishi ML-G30 Model 1/Model 2 */

ROM_START(mlg30)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("g30bios.rom", 0x0000, 0x8000, CRC(a27c563d) SHA1(c1e46c00f1e38fc9e0ab487bf0513bd93ce61f3f))
	ROM_LOAD("g30ext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("g30disk.rom", 0xc000, 0x4000, CRC(05661a3f) SHA1(e695fc0c917577a3183901a08ca9e5f9c60b8317))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("g30kfn.rom", 0x0000, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx2_state::mlg30(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: wd2793/tc8566af?, 1 or 2? 3.5" DSDD drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);    /* Slot 2 subslot 0 */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000);   /* 64KB Mapper RAM */

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - National FS-4500 */

ROM_START(fs4500)
	ROM_REGION(0x40000, "maincpu", 0)
	ROM_LOAD("4500bios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("4500ext.rom",   0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("4500font.rom",  0xc000, 0x4000, CRC(4bd54f95) SHA1(3ce8e35790eb4689b21e14c7ecdd4b63943ee158))
	ROM_LOAD("4500buns.rom", 0x10000, 0x8000, CRC(c9398e11) SHA1(e89ea1e8e583392e2dd9debb8a4b6a162f58ba91))
	ROM_LOAD("4500jush.rom", 0x18000, 0x8000, CRC(4debfd2d) SHA1(6442c1c5cece64c6dae90cc6ae3675f070d93e06))
	ROM_LOAD("4500wor1.rom", 0x20000, 0xc000, CRC(0c8b5cfb) SHA1(3f047469b62d93904005a0ea29092e892724ce0b))
	ROM_LOAD("4500wor2.rom", 0x2c000, 0xc000, CRC(d9909451) SHA1(4c8ea05c09b40c41888fa18db065575a317fda16))
	ROM_LOAD("4500kdr1.rom", 0x38000, 0x4000, CRC(f8c7f0db) SHA1(df07e89fa0b1c7874f9cdf184c136f964fea4ff4))
	ROM_LOAD("4500kdr2.rom", 0x3c000, 0x4000, CRC(69e87c31) SHA1(c63db26660da96af56f8a7d3ea18544b9ae5a37c))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("4500kfn.rom", 0, 0x20000, CRC(956dc96d) SHA1(9ed3ab6d893632b9246e91b412cd5db519e7586b))

	/* Matsushita Bunsetsu Henkan ROM must be emulated */
	ROM_REGION(0x20000, "bunsetsu", 0)
	ROM_LOAD("4500budi.rom", 0, 0x20000, CRC(f94590f8) SHA1(1ebb06062428fcdc66808a03761818db2bba3c73))
ROM_END

void msx2_state::fs4500(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX Engine
	// Matsushita switched device

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "font", 0, 2, 0, 1, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_BUNSETSU, "buns", 0, 2, 1, 2, "maincpu", 0x10000).set_bunsetsu_region_tag("bunsetsu");
	add_internal_slot(config, MSX_SLOT_ROM, "jush", 0, 3, 1, 2, "maincpu", 0x18000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "wor1", 3, 0, 0, 3, "maincpu", 0x20000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr1", 3, 0, 3, 1, "maincpu", 0x38000);
	add_internal_slot(config, MSX_SLOT_ROM, "wor2", 3, 1, 0, 3, "maincpu", 0x2c000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr2", 3, 1, 3, 1, "maincpu", 0x3c000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */

	MSX_S1985(config, "s1985", 0);

	MSX_MATSUSHITA(config, "matsushita", 0);
}

/* MSX2 - National FS-4600 */

ROM_START(fs4600)
	ROM_REGION(0x120000, "maincpu", 0)
	ROM_LOAD("4600bios.rom",  0x0000,   0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("4600ext.rom",   0x8000,   0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("4600disk.rom",  0xc000,   0x4000, CRC(ae4e65b7) SHA1(073feb8bb645d935e099afaf61e6f04f52adee42))
	ROM_LOAD("4600fon1.rom", 0x10000,   0x4000, CRC(7391389b) SHA1(31292b9ca9fe7d1d8833530f44c0a5671bfefe4e))
	ROM_LOAD("4600fon2.rom", 0x14000,   0x4000, CRC(c3a6b445) SHA1(02155fc25c9bd23e1654fe81c74486351e1ecc28))
	ROM_LOAD("4600kdr.rom",  0x18000,   0x8000, CRC(b2db6bf5) SHA1(3a9a942ed888dd641cddf8deada1879c454df3c6))
	ROM_LOAD("4600firm.rom", 0x20000, 0x100000, CRC(1df57472) SHA1(005794c10a4237de3907ba4a44d436078d3c06c2))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("4600kfn.rom", 0, 0x20000, CRC(c61ddc5d) SHA1(5e872d5853698731a0ed22fb72dbcdfd59cd19c3))

	/* Matsushita 12 dots Kanji ROM must be emulated */
	ROM_REGION(0x20000, "kanji12", 0)
	ROM_LOAD("4600kf12.rom", 0, 0x20000, CRC(340d1ef7) SHA1(a7a23dc01314e88381eee88b4878b39931ab4818))
ROM_END

void msx2_state::fs4600(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: mb8877a, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "fon1", 0, 2, 0, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 0, 2, 1, 2, "maincpu", 0x18000);
	add_internal_slot(config, MSX_SLOT_ROM, "fon2", 0, 3, 0, 1, "maincpu", 0x14000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_FS4600, "firm", 3, 1, 0, 4, "maincpu", 0x20000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	msx_mb8877a(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - National FS-4700 */

ROM_START(fs4700)
	ROM_REGION(0x44000, "maincpu", 0)
	ROM_LOAD("4700bios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("4700ext.rom",   0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("4700disk.rom",  0xc000, 0x4000, CRC(1e7d6512) SHA1(78cd7f847e77fd8cd51a647efb2725ba93f4c471))
	ROM_LOAD("4700font.rom", 0x10000, 0x4000, CRC(4bd54f95) SHA1(3ce8e35790eb4689b21e14c7ecdd4b63943ee158))
	ROM_LOAD("4700buns.rom", 0x14000, 0x8000, CRC(c9398e11) SHA1(e89ea1e8e583392e2dd9debb8a4b6a162f58ba91))
	ROM_LOAD("4700jush.rom", 0x1c000, 0x8000, CRC(4debfd2d) SHA1(6442c1c5cece64c6dae90cc6ae3675f070d93e06))
	ROM_LOAD("4700wor1.rom", 0x24000, 0xc000, CRC(5f39a727) SHA1(f5af1d2a8bcf247f78847e1a9d995e581df87e8e))
	ROM_LOAD("4700wor2.rom", 0x30000, 0xc000, CRC(d9909451) SHA1(4c8ea05c09b40c41888fa18db065575a317fda16))
	ROM_LOAD("4700kdr1.rom", 0x3c000, 0x4000, CRC(f8c7f0db) SHA1(df07e89fa0b1c7874f9cdf184c136f964fea4ff4))
	ROM_LOAD("4700kdr2.rom", 0x40000, 0x4000, CRC(69e87c31) SHA1(c63db26660da96af56f8a7d3ea18544b9ae5a37c))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("4700kfn.rom", 0, 0x20000, CRC(956dc96d) SHA1(9ed3ab6d893632b9246e91b412cd5db519e7586b))

	/* Matsushita Bunsetsu Henkan ROM must be emulated */
	ROM_REGION(0x20000, "bunsetsu", 0)
	ROM_LOAD("4700budi.rom", 0, 0x20000, CRC(f94590f8) SHA1(1ebb06062428fcdc66808a03761818db2bba3c73))
ROM_END

void msx2_state::fs4700(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: mb8877a, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine
	// Matsushita switched device

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "font", 0, 2, 0, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_BUNSETSU, "buns", 0, 2, 1, 2, "maincpu", 0x14000).set_bunsetsu_region_tag("bunsetsu");
	add_internal_slot(config, MSX_SLOT_ROM, "jush", 0, 3, 1, 2, "maincpu", 0x1c000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "wor1", 3, 0, 0, 3, "maincpu", 0x24000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr1", 3, 0, 3, 1, "maincpu", 0x3c000);
	add_internal_slot(config, MSX_SLOT_ROM, "wor2", 3, 1, 0, 3, "maincpu", 0x30000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr2", 3, 1, 3, 1, "maincpu", 0x40000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	MSX_MATSUSHITA(config, "matsushita", 0);

	msx_mb8877a(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - National FS-5000 */

ROM_START(fs5000)
	ROM_REGION(0x30000, "maincpu", 0)
	ROM_LOAD("5000bios.rom", 0x0000, 0x8000, CRC(a44ea707) SHA1(59967765d6e9328909dee4dac1cbe4cf9d47d315))
	ROM_LOAD("5000ext.rom",  0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("5000disk.rom", 0xc000, 0x4000, CRC(ae4e65b7) SHA1(073feb8bb645d935e099afaf61e6f04f52adee42))
	ROM_LOAD("5000rtc.rom", 0x10000, 0x8000, CRC(03351598) SHA1(98bbfa3ab07b7a5cad55d7ddf7cbd9440caa2a86))
	ROM_LOAD("5000kdr.rom", 0x18000, 0x8000, CRC(b2db6bf5) SHA1(3a9a942ed888dd641cddf8deada1879c454df3c6))
	ROM_FILL(0x20000, 0x10000, 0xff)

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("5000kfn.rom", 0, 0x20000, CRC(c61ddc5d) SHA1(5e872d5853698731a0ed22fb72dbcdfd59cd19c3))
ROM_END

void msx2_state::fs5000(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: wd2793, 2 3.5" DSDD drives
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "empty1", 0, 1, 0, 4, "maincpu", 0x20000);
	add_internal_slot(config, MSX_SLOT_ROM, "empty2", 0, 2, 0, 4, "maincpu", 0x20000);
	add_internal_slot(config, MSX_SLOT_ROM, "empty3", 0, 3, 0, 4, "maincpu", 0x20000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 0, 1, 2, "maincpu", 0x18000);
	add_internal_slot(config, MSX_SLOT_ROM, "rtcrom", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - National FS-5500F2*/
/* The National FS-5500 had two versions: */
/* F1 has 1 floppy drive */
/* F2 has 2 floppy drives */

ROM_START(fs5500f1)
	ROM_REGION(0x30000, "maincpu", 0)
	ROM_LOAD("5500bios.rom", 0x0000, 0x8000, CRC(5bf38e13) SHA1(44e0dd215b2a9f0770dd76fb49187c05b083eed9))
	ROM_LOAD("5500ext.rom",  0x8000, 0x4000, CRC(3c42c367) SHA1(4be8371f3b03e70ddaca495958345f3c4f8e2d36))
	ROM_LOAD("5500disk.rom", 0xc000, 0x4000, CRC(1e7d6512) SHA1(78cd7f847e77fd8cd51a647efb2725ba93f4c471))
	ROM_LOAD("5500imp.rom", 0x10000, 0x8000, CRC(6173a88c) SHA1(b677a861b67e8763a11d5dcf52416b42493ade57))
	ROM_LOAD("5500kdr.rom", 0x18000, 0x8000, CRC(b2db6bf5) SHA1(3a9a942ed888dd641cddf8deada1879c454df3c6))
	ROM_FILL(0x20000, 0x10000, 0xff)

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("5500kfn.rom", 0, 0x20000, CRC(956dc96d) SHA1(9ed3ab6d893632b9246e91b412cd5db519e7586b))
ROM_END

void msx2_state::fs5500f1(machine_config &config)
{
	msx2(config);
	// YM2149 in (S-1985 MSX Engine)
	// FDC: mb8877a, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine
	// Matsushita switched device

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "empty1", 0, 1, 0, 4, "maincpu", 0x20000);
	add_internal_slot(config, MSX_SLOT_ROM, "empty2", 0, 2, 0, 4, "maincpu", 0x20000);
	add_internal_slot(config, MSX_SLOT_ROM, "empty3", 0, 3, 0, 4, "maincpu", 0x20000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 0, 1, 2, "maincpu", 0x18000);
	add_internal_slot(config, MSX_SLOT_ROM, "imp", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	MSX_MATSUSHITA(config, "matsushita", 0);

	msx_mb8877a(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - National FS-5500F2*/
/* The National FS-5500 had two versions: */
/* F1 has 1 floppy drive */
/* F2 has 2 floppy drives */

ROM_START(fs5500f2)
	ROM_REGION(0x30000, "maincpu", 0)
	ROM_LOAD("5500bios.rom", 0x0000, 0x8000, CRC(5bf38e13) SHA1(44e0dd215b2a9f0770dd76fb49187c05b083eed9))
	ROM_LOAD("5500ext.rom",  0x8000, 0x4000, CRC(3c42c367) SHA1(4be8371f3b03e70ddaca495958345f3c4f8e2d36))
	ROM_LOAD("5500disk.rom", 0xc000, 0x4000, CRC(1e7d6512) SHA1(78cd7f847e77fd8cd51a647efb2725ba93f4c471))
	ROM_LOAD("5500imp.rom", 0x10000, 0x8000, CRC(6173a88c) SHA1(b677a861b67e8763a11d5dcf52416b42493ade57))
	ROM_LOAD("5500kdr.rom", 0x18000, 0x8000, CRC(b2db6bf5) SHA1(3a9a942ed888dd641cddf8deada1879c454df3c6))
	ROM_FILL(0x20000, 0x10000, 0xff)

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("5500kfn.rom", 0, 0x20000, CRC(956dc96d) SHA1(9ed3ab6d893632b9246e91b412cd5db519e7586b))
ROM_END

void msx2_state::fs5500f2(machine_config &config)
{
	msx2(config);
	// YM2149 in (S-1985 MSX Engine)
	// FDC: mb8877a, 2 3.5" DSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine
	// Matsushita switched device

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "empty1", 0, 1, 0, 4, "maincpu", 0x20000);
	add_internal_slot(config, MSX_SLOT_ROM, "empty2", 0, 2, 0, 4, "maincpu", 0x20000);
	add_internal_slot(config, MSX_SLOT_ROM, "empty3", 0, 3, 0, 4, "maincpu", 0x20000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 0, 1, 2, "maincpu", 0x18000);
	add_internal_slot(config, MSX_SLOT_ROM, "imp", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	MSX_MATSUSHITA(config, "matsushita", 0);

	msx_mb8877a(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Panasonic FS-A1 */

ROM_START(fsa1)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("a1bios.rom",   0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("a1ext.rom",    0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("a1desk1.rom", 0x10000, 0x8000, CRC(99c48147) SHA1(63098f27beac9eca6b39d837d2a552395df33fe1))
	ROM_LOAD("a1desk2.rom", 0x18000, 0x8000, CRC(7f6f4aa1) SHA1(7f5b76605e3d898cc4b5aacf1d7682b82fe84353))
ROM_END

void msx2_state::fsa1(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64 KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "desk1", 3, 2, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_ROM, "desk2", 3, 3, 1, 2, "maincpu", 0x18000);
}

/* MSX2 - Panasonic FS-A1 (a) */

ROM_START(fsa1a)
	ROM_REGION(0x1c000, "maincpu", 0)
	ROM_LOAD("a1bios.rom",   0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("a1ext.rom",    0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("a1desk1a.rom", 0xc000, 0x8000, CRC(25b5b170) SHA1(d9307bfdaab1312d25e38af7c0d3a7671a9f716b))
	ROM_LOAD("a1desk2.rom", 0x14000, 0x8000, CRC(7f6f4aa1) SHA1(7f5b76605e3d898cc4b5aacf1d7682b82fe84353))
ROM_END

void msx2_state::fsa1a(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 0, 0, 4);  /* 64KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "desk1", 3, 2, 1, 2, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_ROM, "desk2", 3, 3, 1, 2, "maincpu", 0x14000);
}

/* MSX2 - Panasonic FS-A1F */

ROM_START(fsa1f)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("a1fbios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("a1fext.rom",   0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("a1fdisk.rom",  0xc000, 0x4000, CRC(e25cacca) SHA1(607cfca605eaf82e3efa33459d6583efb7ecc13b))
	ROM_LOAD("a1fkdr.rom",  0x10000, 0x8000, CRC(2dbea5ec) SHA1(ea35cc2cad9cfdf56cae224d8ee41579de37f000))
	ROM_LOAD("a1fcock.rom", 0x18000, 0x8000, CRC(5c2948cd) SHA1(4a99f2444f29c2b642efd6f084081d6fd96bfa9b))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("a1fkfn.rom", 0, 0x20000, CRC(c61ddc5d) SHA1(5e872d5853698731a0ed22fb72dbcdfd59cd19c3))
ROM_END

void msx2_state::fsa1f(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "fkdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "fcock", 3, 3, 1, 2, "maincpu", 0x18000);

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Panasonic FS-A1FM */

ROM_START(fsa1fm)
	ROM_REGION(0x110000, "maincpu", 0)
	ROM_LOAD("a1fmbios.rom",  0x0000,   0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("a1fmext.rom",   0x8000,   0x4000, CRC(ad295b5d) SHA1(d552319a19814494e3016de4b8f010e8f7b97e02))
	ROM_LOAD("a1fmdisk.rom",  0xc000,   0x4000, CRC(e25cacca) SHA1(607cfca605eaf82e3efa33459d6583efb7ecc13b))
	ROM_LOAD("a1fmfirm.rom", 0x10000, 0x100000, CRC(8ce0ece7) SHA1(f89e3d8f3b6855c29d71d3149cc762e0f6918ad5))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("a1fmkfn.rom", 0, 0x20000, CRC(c61ddc5d) SHA1(5e872d5853698731a0ed22fb72dbcdfd59cd19c3))

	/* Matsushita 12 dots Kanji ROM must be emulated */
	ROM_REGION(0x20000, "kanji12", 0)
	ROM_LOAD("a1fmkf12.rom", 0, 0x20000, CRC(340d1ef7) SHA1(a7a23dc01314e88381eee88b4878b39931ab4818))
ROM_END

void msx2_state::fsa1fm(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// Integrated 1200baud modem

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
/*  MSX_LAYOUT_SLOT (3, 1, 1, 4, MODEM_ROM, 0x20000, 0x10000) */ /* Modem Mapper of FS-CM1/A1FM must be emulated */
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
/*  MSX_LAYOUT_SLOT (3, 3, 0, 4, FSA1FM_ROM, 0x100000, 0x10000) */ /* Panasonic FS-A1FM Mapper must be emulated */

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Panasonic FS-A1MK2 */

ROM_START(fsa1mk2)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("a1mkbios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("a1mk2ext.rom",  0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("a1mkcoc1.rom",  0xc000, 0x8000, CRC(0eda3f57) SHA1(2752cd89754c05abdf7c23cba132d38e3ef0f27d))
	ROM_LOAD("a1mkcoc2.rom", 0x14000, 0x4000, CRC(756d7128) SHA1(e194d290ebfa4595ce0349ea2fc15442508485b0))
	ROM_LOAD("a1mkcoc3.rom", 0x18000, 0x8000, CRC(c1945676) SHA1(a3f4e2e4934074925d775afe30ac72f150ede543))
ROM_END

void msx2_state::fsa1mk2(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64 KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "coc1", 3, 1, 1, 2, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_ROM, "coc2", 3, 2, 1, 1, "maincpu", 0x14000);
	add_internal_slot(config, MSX_SLOT_ROM, "coc3", 3, 3, 1, 2, "maincpu", 0x18000);
}

/* MSX2 - Philips NMS-8220 - 2 possible sets (/00 /16) */

ROM_START(nms8220)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8220bios.rom.u14", 0x0000, 0x8000, BAD_DUMP CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("8220ext.rom.u14",  0x8000, 0x4000, BAD_DUMP CRC(06e4f5e6) SHA1(f5eb0a396097572589f2a6efeed045044e9425e4))
	ROM_LOAD("8220pen.rom.u13",  0xc000, 0x4000, CRC(3d38c53e) SHA1(cb754aed85b3e97a7d3c5894310df7ca18f89f41))
ROM_END

void msx2_state::nms8220(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000).set_ramio_bits(0xf8);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "pen", 3, 3, 1, 1, "maincpu", 0xc000);
}

/* MSX2 - Philips NMS-8220 (a) */

ROM_START(nms8220a)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8220bios.rom.u14", 0x0000, 0x8000, BAD_DUMP CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("8220ext.rom.u14",  0x8000, 0x4000, BAD_DUMP CRC(06e4f5e6) SHA1(f5eb0a396097572589f2a6efeed045044e9425e4))
	ROM_LOAD("8220pena.rom.u13", 0xc000, 0x4000, CRC(17817b5a) SHA1(5df95d033ae70b107697b69470126ce1b7ae9eb5))
ROM_END

void msx2_state::nms8220a(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000).set_ramio_bits(0xf8);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "pen", 3, 3, 1, 1, "maincpu", 0xc000);
}

/* MSX2 - Philips NMS-8245 - 2 possible sets (/00 /16) */
/* /00 - A16 = 0 */
/* /16 - A16 = 1 */
/* /19 - Azerty keyboard */

ROM_START(nms8245)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("nms8245.u7", 0x0000, 0x20000, BAD_DUMP CRC(0c827d5f) SHA1(064e706cb1f12b99b329944ceeedc0efc3b2d9be))
ROM_END

void msx2_state::nms8245(machine_config &config)
{
	msx2_pal(config);
	// XTAL: 21328.1 (different from default)
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8245F */
/* NMS-8245/19? */

ROM_START(nms8245f)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("nms8245.u7", 0x0000, 0x20000, BAD_DUMP CRC(0c827d5f) SHA1(064e706cb1f12b99b329944ceeedc0efc3b2d9be))
ROM_END

void msx2_state::nms8245f(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x10000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x18000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0x1c000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8250 */
/* Labels taken from an NMS-8250/00 */

ROM_START(nms8250)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("d23c256eac.ic119", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("d23128ec.ic118",   0x8000, 0x4000, CRC(66237ecf) SHA1(5c1f9c7fb655e43d38e5dd1fcc6b942b2ff68b02))
	ROM_LOAD("jq00014.ic117",    0xc000, 0x4000, CRC(ca3307d3) SHA1(c3efedda7ab947a06d9345f7b8261076fa7ceeef))
ROM_END

void msx2_state::nms8250(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8250F */

ROM_START(nms8250f)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("nms8250fbios.rom", 0x0000, 0x8000, CRC(5cd35ced) SHA1(b034764e6a8978db60b1d652917f5e24a66a7925))
	ROM_LOAD("nms8250fext.rom",  0x8000, 0x4000, CRC(781ba055) SHA1(fd4bcc81a8160a1dea06036c5f79d200f948f4d6))
	ROM_LOAD("nms8250fdisk.rom", 0xc000, 0x4000, CRC(13b60725) SHA1(58ba1887e8fd21c912b6859cae6514bd874ffcca))
ROM_END

void msx2_state::nms8250f(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8250J */

ROM_START(nms8250j)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8250jbios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("8250jext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("8250jdisk.rom", 0xc000, 0x4000, CRC(ca3307d3) SHA1(c3efedda7ab947a06d9345f7b8261076fa7ceeef))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("8250jkfn.rom", 0x00000, 0x20000, CRC(5a59926e) SHA1(6acaf2eeb57f65f7408235d5e07b7563229de799))
ROM_END

void msx2_state::nms8250j(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: wd2793?, 1 3.5" DSDD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8255 */

ROM_START(nms8255)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8255bios.rom.ic119", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("8255ext.rom.ic118",  0x8000, 0x4000, CRC(66237ecf) SHA1(5c1f9c7fb655e43d38e5dd1fcc6b942b2ff68b02))
	ROM_LOAD("8255disk.rom.ic117", 0xc000, 0x4000, CRC(ca3307d3) SHA1(c3efedda7ab947a06d9345f7b8261076fa7ceeef))
ROM_END

void msx2_state::nms8255(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 2 3.5" DSDD drives
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8255F */

ROM_START(nms8255f)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("nms8255fbios.rom", 0x0000, 0x8000, CRC(5cd35ced) SHA1(b034764e6a8978db60b1d652917f5e24a66a7925))
	ROM_LOAD("nms8255fext.rom",  0x8000, 0x4000, CRC(781ba055) SHA1(fd4bcc81a8160a1dea06036c5f79d200f948f4d6))
	ROM_LOAD("nms8255fdisk.rom", 0xc000, 0x4000, CRC(13b60725) SHA1(58ba1887e8fd21c912b6859cae6514bd874ffcca))
ROM_END

void msx2_state::nms8255f(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 2 3.5" DSDD drives
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8260 */
/* Prototype created by JVC for Philips. Based on an NMS-8250 with the floppy drive removed and replaced with a 20MB JVC harddisk */

ROM_START(nms8260)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("nms8260bios.rom", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("nms8260ext.rom",  0x8000, 0x4000, CRC(66237ecf) SHA1(5c1f9c7fb655e43d38e5dd1fcc6b942b2ff68b02))
	ROM_LOAD("nms8260disk.rom", 0xc000, 0x4000, CRC(ca3307d3) SHA1(c3efedda7ab947a06d9345f7b8261076fa7ceeef))
	ROM_LOAD("nms8260hdd.rom", 0x10000, 0x4000, CRC(0051afc3) SHA1(77f9fe964f6d8cb8c4af3b5fe63ce6591d5288e6))
ROM_END

void msx2_state::nms8260(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drives
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "hdd", 2, 0, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	// There is actually only an FDC inside the real thing. With a floppy controller to attach an external floppy drive
	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8270 - Not confirmed to exist yet */

/* MSX2 - Philips NMS-8280 - 2 possible sets (/00 /16) */

ROM_START(nms8280)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8280bios.rom.ic119", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("8280ext.rom.ic118",  0x8000, 0x4000, CRC(66237ecf) SHA1(5c1f9c7fb655e43d38e5dd1fcc6b942b2ff68b02))
	ROM_LOAD("8280disk.rom.ic117", 0xc000, 0x4000, CRC(ca3307d3) SHA1(c3efedda7ab947a06d9345f7b8261076fa7ceeef))
ROM_END

void msx2_state::nms8280(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 2 3.5" DSDD drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8280F */

ROM_START(nms8280f)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8280fbios.rom", 0x0000, 0x8000, CRC(5cd35ced) SHA1(b034764e6a8978db60b1d652917f5e24a66a7925))
	ROM_LOAD("8280fext.rom",  0x8000, 0x4000, CRC(781ba055) SHA1(fd4bcc81a8160a1dea06036c5f79d200f948f4d6))
	ROM_LOAD("8280fdisk.rom", 0xc000, 0x4000, CRC(13b60725) SHA1(58ba1887e8fd21c912b6859cae6514bd874ffcca))
ROM_END

void msx2_state::nms8280f(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 2 3.5" DSDD drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips NMS-8280G */

ROM_START(nms8280g)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8280gbios.rom.ic119", 0x0000, 0x8000, CRC(8fa060e2) SHA1(b17d9bea0eb16a1aa2d0ccbd7c9488da9f57698e))
	ROM_LOAD("8280gext.rom.ic118",  0x8000, 0x4000, CRC(41e36d03) SHA1(4ab7b2030d022f5486abaab22aaeaf8aa23e05f3))
	ROM_LOAD("8280gdisk.rom.ic117", 0xc000, 0x4000, CRC(d0beebb8) SHA1(d1001f93c87ff7fb389e418e33bf7bc81bdbb65f))
ROM_END

void msx2_state::nms8280g(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 2 3.5" DSDD drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips VG-8230 (u11 - exp, u12 - basic, u13 - disk */

ROM_START(vg8230)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8230bios.rom.u12", 0x0000, 0x8000, CRC(b31c851d) SHA1(0de3c802057560560a03d7965fcc4cff69f8575c))
	ROM_LOAD("8230ext.rom.u11",  0x8000, 0x4000, CRC(8f84f783) SHA1(3288894e1be6af705871499b23c85732dbc40993))
	ROM_LOAD("8230disk.rom.u13", 0xc000, 0x4000, CRC(77c4e5bc) SHA1(849f93867ff7846b27f84d0be418569faf058ac2))
ROM_END

void msx2_state::vg8230(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000).set_ramio_bits(0xf8);   /* 64KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips VG-8230J */

ROM_START(vg8230j)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8230jbios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("8230jext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("8230jdisk.rom", 0xc000, 0x4000, CRC(7639758a) SHA1(0f5798850d11b316a4254b222ca08cc4ad6d4da2))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("8230jkfn.rom", 0x00000, 0x20000, CRC(5a59926e) SHA1(6acaf2eeb57f65f7408235d5e07b7563229de799))
ROM_END

void msx2_state::vg8230j(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: wd2793?, 1 3.5" SSDD drive?
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips VG-8235 3 psosible basic and ext roms (/00 /02 /19) */

ROM_START(vg8235)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8235bios.rom.u48", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("8235ext.rom.u49",  0x8000, 0x4000, CRC(66237ecf) SHA1(5c1f9c7fb655e43d38e5dd1fcc6b942b2ff68b02))
	ROM_LOAD("8235disk.rom.u50", 0xc000, 0x4000, CRC(51daeb25) SHA1(8954e59aa79310c7b719ecf0cde1e82fb731dcd1))
ROM_END

void msx2_state::vg8235(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips VG-8235F */

ROM_START(vg8235f)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8235fbios.rom.u48", 0x0000, 0x8000, CRC(c0577a50) SHA1(3926cdd91fa89657a811463e48cfbdb350676e51))
	ROM_LOAD("8235fext.rom.u49",  0x8000, 0x4000, CRC(e235d5c8) SHA1(792e6b2814ab783d06c7576c1e3ccd6a9bbac34a))
	ROM_LOAD("8235fdisk.rom.u50", 0xc000, 0x4000, CRC(77c4e5bc) SHA1(849f93867ff7846b27f84d0be418569faf058ac2))
ROM_END

void msx2_state::vg8235f(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 1 3.5" SSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0xf8);   /* 128KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_ssdd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Philips VG-8240 */

ROM_START(vg8240)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("8240bios.rom", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("8240ext.rom",  0x8000, 0x4000, CRC(66237ecf) SHA1(5c1f9c7fb655e43d38e5dd1fcc6b942b2ff68b02))
	ROM_LOAD("8240disk.rom", 0xc000, 0x4000, CRC(ca3307d3) SHA1(c3efedda7ab947a06d9345f7b8261076fa7ceeef))
ROM_END

void msx2_state::vg8240(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000).set_ramio_bits(0xf8);   /* 64KB Mapper RAM */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sanyo MPC-2300 */

ROM_START(mpc2300)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("2300bios.rom", 0x0000, 0x8000, CRC(e7d08e29) SHA1(0f851ee7a1cf79819f61cc89e9948ee72a413802))
	ROM_LOAD("2300ext.rom",  0x8000, 0x4000, CRC(3d7dc718) SHA1(e1f834b28c3ee7c9f79fe6fbf2b23c8a0617892b))
ROM_END

void msx2_state::mpc2300(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x20000);   /* 128KB?? Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
}

/* MSX2 - Sanyo MPC-2500FD */

ROM_START(mpc2500f)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("mpc2500fdbios.rom", 0x0000, 0x8000, CRC(e7d08e29) SHA1(0f851ee7a1cf79819f61cc89e9948ee72a413802))
	ROM_LOAD("mpc2500fdext.rom",  0x8000, 0x4000, CRC(3d7dc718) SHA1(e1f834b28c3ee7c9f79fe6fbf2b23c8a0617892b))
	ROM_LOAD("mpc2500fddisk.rom", 0xc000, 0x4000, CRC(38454059) SHA1(58ac78bba29a06645ca8d6a94ef2ac68b743ad32))
ROM_END

void msx2_state::mpc2500f(machine_config &config)
{
	msx2(config);
	// YM2149
	// FDC: wd2793?, 1? 3.5" DSDD drive?
	// 2 Cartridge slots?
	// S-3527 MSX Eninge

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 2, 3, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 1, 1, 2, "maincpu", 0xC000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000);   /* 64KB?? Mapper RAM */

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sanyo Wavy MPC-25FD */

ROM_START(mpc25fd)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("25fdbios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("25fdext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("25fddisk.rom", 0xc000, 0x4000, CRC(38454059) SHA1(58ac78bba29a06645ca8d6a94ef2ac68b743ad32))
ROM_END

void msx2_state::mpc25fd(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793, 1 drive
	// 1 Cartridge slot (slot 1)
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 2, 3, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000);   /* 128KB?? RAM */

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sanyo Wavy MPC-27 */

ROM_START(mpc27)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("mpc27bios.rom", 0x0000, 0x8000, CRC(ba81b3dd) SHA1(4ce41fcc1a603411ec4e99556409c442078f0ecf))
	ROM_LOAD("mpc27ext.rom",  0x8000, 0x4000, CRC(90ca25b5) SHA1(fd9fa78bac25aa3c0792425b21d14e364cf7eea4))
	ROM_LOAD("mpc27disk.rom", 0xc000, 0x4000, CRC(38454059) SHA1(58ac78bba29a06645ca8d6a94ef2ac68b743ad32))
	ROM_LOAD("mlp27.rom",    0x10000, 0x2000, CRC(8f9e6ba0) SHA1(c3a47480c9dd2235f40f9a53dab68e3c48adca01))
	ROM_RELOAD(0x12000, 0x2000)
ROM_END

void msx2_state::mpc27(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-3527 MSX Engine)
	// FDC: wd2793?, 1 drive
	// 2 Cartridge slots?
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x20000);   /* 128KB?? RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 2, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "lpen", 3, 3, 1, 1, "maincpu", 0x10000);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sanyo Wavy PHC-23 = PHC-23J(B)*/

ROM_START(phc23)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("23bios.rom", 0x0000, 0x8000, CRC(ba81b3dd) SHA1(4ce41fcc1a603411ec4e99556409c442078f0ecf))
	ROM_LOAD("23ext.rom",  0x8000, 0x4000, CRC(90ca25b5) SHA1(fd9fa78bac25aa3c0792425b21d14e364cf7eea4))
ROM_END

void msx2_state::phc23(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 2, 0, 4);  /* 64KB RAM */

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Sanyo Wavy PHC-55FD2 */

ROM_START(phc55fd2)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("phc55fd2bios.rom", 0x0000, 0x8000, CRC(ba81b3dd) SHA1(4ce41fcc1a603411ec4e99556409c442078f0ecf))
	ROM_LOAD("phc55fd2ext.rom",  0x8000, 0x4000, CRC(90ca25b5) SHA1(fd9fa78bac25aa3c0792425b21d14e364cf7eea4))
	ROM_LOAD("phc55fd2disk.rom", 0xc000, 0x4000, CRC(38454059) SHA1(58ac78bba29a06645ca8d6a94ef2ac68b743ad32))
ROM_END

void msx2_state::phc55fd2(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: wd2793?, 2 3.5" DSDD drives
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x20000);   /* 128KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 2, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sanyo Wavy PHC-77 */

ROM_START(phc77)
	ROM_REGION(0x90000, "maincpu", 0)
	ROM_LOAD("phc77bios.rom",      0x0000,  0x8000, CRC(ba81b3dd) SHA1(4ce41fcc1a603411ec4e99556409c442078f0ecf))
	ROM_LOAD("phc77ext.rom",       0x8000,  0x4000, CRC(90ca25b5) SHA1(fd9fa78bac25aa3c0792425b21d14e364cf7eea4))
	ROM_LOAD("phc77disk.rom",      0xc000,  0x4000, CRC(38454059) SHA1(58ac78bba29a06645ca8d6a94ef2ac68b743ad32))
	ROM_LOAD("phc77msxwrite.rom", 0x10000, 0x80000, CRC(ef02e4f3) SHA1(4180544158a57c99162269e33e4f2c77c9fce84e))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("phc77kfn.rom", 0x0000, 0x20000, CRC(3b8fdf44) SHA1(fc71561a64f73da0e0043d256f67fd18d7fc3a7f))
ROM_END

void msx2_state::phc77(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: wd2793?, 1 drive
	// 2 Cartridge slots
	// S-1985 MSX Engine
	// Builtin printer

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000);   /* 64KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 2, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "write", 3, 3, 1, 2, "maincpu", 0x10000);

	MSX_S1985(config, "s1985", 0);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sharp Epcom HotBit 2.0 */

ROM_START(hotbit20)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("hb2bios.rom",       0x0000, 0x8000, CRC(0160e8c9) SHA1(d0cfc35f22b150a1cb10decae4841dfe63b78251))
	ROM_LOAD("hb2ext.rom",        0x8000, 0x4000, CRC(08ced880) SHA1(4f2a7e0172f0214f025f23845f6e053d0ffd28e8))
	ROM_LOAD("xbasic2.rom",       0xc000, 0x4000, CRC(2825b1a0) SHA1(47370bec7ca1f0615a54eda548b07fbc0c7ef398))
	ROM_LOAD("microsoldisk.rom", 0x10000, 0x4000, CRC(6704ef81) SHA1(a3028515ed829e900cc8deb403e17b09a38bf9b0))
ROM_END

void msx2_state::hotbit20(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: microsol, 1 or 2 drives?
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 1, 1, 0, 1, "maincpu", 0x8000); /* EXT */
	add_internal_slot(config, MSX_SLOT_ROM, "xbasic", 1, 1, 1, 1, "maincpu", 0xc000); /* XBASIC */
	add_internal_slot(config, MSX_SLOT_DISK5, "disk", 1, 3, 1, 1, "maincpu", 0x10000).set_tags("fdc", "fdc:0", "fdc:1", "fdc:2", "fdc:3"); /* Microsol controller */
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 2, 0, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);

	msx_microsol(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F1 */

ROM_START(hbf1)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("f1bios.rom",   0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("f1ext.rom",    0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("f1note1.rom",  0xc000, 0x4000, CRC(84810ea8) SHA1(9db72bb78792595a12499c821048504dc96ef848))
	ROM_LOAD("f1note2.rom", 0x10000, 0x8000, CRC(e32e5ee0) SHA1(aa78fc9bcd2343f84cf790310a768ee47f90c841))
	ROM_LOAD("f1note3.rom", 0x18000, 0x8000, CRC(73eb9329) SHA1(58accf41a90693874b86ce98d8d43c27beb8b6dc))
ROM_END

void msx2_state::hbf1(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "note1", 3, 0, 1, 1, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_ROM, "note2", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_ROM, "note3", 3, 2, 1, 2, "maincpu", 0x18000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram", 3, 3, 0, 4);  /* 64KB RAM */
}

/* MSX2 - Sony HB-F1II */

ROM_START(hbf12)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("f12bios.rom",   0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("f12ext.rom",    0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("f12note1.rom",  0xc000, 0x4000, CRC(dcacf970) SHA1(30d914cda2180889a40a3328e0a0c1327f4eaa10))
	ROM_LOAD("f12note2.rom", 0x10000, 0x8000, CRC(b0241a61) SHA1(ed2fea5c2a3c2e58d4f69f9d636e08574486a2b1))
	ROM_LOAD("f12note3.rom", 0x18000, 0x8000, CRC(44a10e6a) SHA1(917d1c079e03c4a44de864f123d03c4e32c8daae))
ROM_END

void msx2_state::hbf12(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "note1", 3, 0, 1, 1, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_ROM, "note2", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_ROM, "note3", 3, 2, 1, 2, "maincpu", 0x18000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 3, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
}

/* MSX2 - Sony HB-F1XD */

ROM_START(hbf1xd)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("f1xdbios.rom.ic27", 0x0000, 0x8000, BAD_DUMP CRC(ba81b3dd) SHA1(4ce41fcc1a603411ec4e99556409c442078f0ecf))
	ROM_LOAD("f1xdext.rom.ic27",  0x8000, 0x4000, BAD_DUMP CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("f1xddisk.rom.ic27", 0xc000, 0x4000, BAD_DUMP CRC(54c73ad6) SHA1(12f2cc79b3d09723840bae774be48c0d721ec1c6))
ROM_END

void msx2_state::hbf1xd(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1895 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 3, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */

	MSX_S1985(config, "s1985", 0);

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F1XDMK2 */

ROM_START(hbf1xdm2)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("f1m2bios.rom.ic27", 0x0000, 0x8000, BAD_DUMP CRC(ba81b3dd) SHA1(4ce41fcc1a603411ec4e99556409c442078f0ecf))
	ROM_LOAD("f1m2ext.rom.ic27",  0x8000, 0x4000, BAD_DUMP CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("f1m2disk.rom.ic27", 0xc000, 0x4000, BAD_DUMP CRC(54c73ad6) SHA1(12f2cc79b3d09723840bae774be48c0d721ec1c6))
ROM_END

void msx2_state::hbf1xdm2(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 3, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F5 */

ROM_START(hbf5)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("hbf5bios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("hbf5ext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("hbf5note.rom", 0xc000, 0x4000, CRC(0cdc0777) SHA1(06ba91d6732ee8a2ecd5dcc38b0ce42403d86708))
ROM_END

void msx2_state::hbf5(machine_config &config)
{
	msx2_pal(config);
	// YM2149
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "note", 0, 1, 1, 1, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x10000);   /* 64KB?? Mapper RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
}

/* MSX2 - Sony HB-F500 */

ROM_START(hbf500)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("f500bios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("f500ext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("f500disk.rom", 0xc000, 0x4000, CRC(f7f5b0ea) SHA1(e93b8da1e8dddbb3742292b0e5e58731b90e9313))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("f500kfn.rom", 0, 0x20000, CRC(5a59926e) SHA1(6acaf2eeb57f65f7408235d5e07b7563229de799))
ROM_END

void msx2_state::hbf500(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram1", 0, 0, 2, 2);   /* 32KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 0, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 0, 2, 0, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F500F */

ROM_START(hbf500f)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("hbf500fbios.rom", 0x0000, 0x8000, CRC(440dae3c) SHA1(fedd9b682d056ddd1e9b3d281723e12f859b2e69))
	ROM_LOAD("hbf500fext.rom",  0x8000, 0x4000, CRC(e235d5c8) SHA1(792e6b2814ab783d06c7576c1e3ccd6a9bbac34a))
	ROM_LOAD("hbf500fdisk.rom", 0xc000, 0x4000, CRC(6e718f5c) SHA1(0e081572f84555dc13bdb0c7044a19d6c164d985))
ROM_END

void msx2_state::hbf500f(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 3.5" DSDD drive
	// 3 Cartridge slots or 2 Cartridge slots and 1 expansion slot ?

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram1", 0, 0, 2, 2);   /* 32KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 0, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 0, 2, 0, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
}

/* MSX2 - Sony HB-F500P */

ROM_START(hbf500p)
	ROM_REGION(0x1c000, "maincpu", 0)
	ROM_LOAD("500pbios.rom.ic41", 0x0000, 0x8000, CRC(b31c851d) SHA1(0de3c802057560560a03d7965fcc4cff69f8575c))
	ROM_LOAD("500pext.ic47",      0x8000, 0x8000, CRC(cdd4824a) SHA1(505031f1e8396a6e0cb11c1540e6e7f6999d1191))
	ROM_FILL(0x10000, 0xc000, 0xff)
ROM_END

void msx2_state::hbf500p(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 3.5" DSDD drive
	// 3 Cartridge slots or 2 Cartridge slots and 1 expansion slot ?

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram1", 0, 0, 2, 2);   /* 32KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 0, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 0, 2, 0, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "empty", 3, 0, 0, 4, "maincpu", 0xc000);     // Empty? or is this the 3rd cartridge/expansion slot ?
}

/* MSX2 - Sony HB-F700D */

ROM_START(hbf700d)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("700dbios.rom.ic5", 0x0000, 0x8000, CRC(e975aa79) SHA1(cef16eb95502ba6ab2265fcafcedde470a101541))
	ROM_LOAD("700dext.ic6",      0x8000, 0x8000, CRC(100cf756) SHA1(317722fa36c2ed31c07c5218b43490fd5badf1f8))
ROM_END

void msx2_state::hbf700d(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 3, 0, 4).set_total_size(0x40000).set_ramio_bits(0x80);   /* 256KB Mapper RAM */

	MSX_S1985(config, "s1985", 0);

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F700F */

ROM_START(hbf700f)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("700fbios.ic5", 0x0000, 0x8000, CRC(440dae3c) SHA1(fedd9b682d056ddd1e9b3d281723e12f859b2e69))
	ROM_LOAD("700fext.ic6",  0x8000, 0x8000, CRC(7c8b07b1) SHA1(ecacb20ba0a9bbd25e8c0f128d64dd66f8cd8bee))
ROM_END

void msx2_state::hbf700f(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 3, 0, 4).set_total_size(0x40000).set_ramio_bits(0x80);   /* 256KB Mapper RAM */

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F700P */

ROM_START(hbf700p)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("700pbios.rom.ic5", 0x0000, 0x8000, CRC(b31c851d) SHA1(0de3c802057560560a03d7965fcc4cff69f8575c))
	ROM_LOAD("700pext.ic6",      0x8000, 0x8000, CRC(63e1bffc) SHA1(496698a60432490dc1306c8cc1d4a6ded275261a))
ROM_END

void msx2_state::hbf700p(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 3, 0, 4).set_total_size(0x40000).set_ramio_bits(0x80);   /* 256KB Mapper RAM */

	MSX_S1985(config, "s1985", 0);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F700S */

ROM_START(hbf700s)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("700sbios.rom.ic5", 0x0000, 0x8000, CRC(c2b889a5) SHA1(4811956f878c3e03da46317f787cdc4bebc86f47))
	ROM_LOAD("700sext.ic6",      0x8000, 0x8000, CRC(28d1badf) SHA1(ae3ed88a2d7034178e08f7bdf5409f462bf67fc9))
ROM_END

void msx2_state::hbf700s(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 3, 0, 4).set_total_size(0x40000).set_ramio_bits(0x80);   /* 256KB Mapper RAM */

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F900 */
ROM_START(hbf900)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("f900bios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("f900ext.rom",   0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("f900disk.rom",  0xc000, 0x4000, CRC(f83d0ea6) SHA1(fc760d1d7b16370abc7eea39955f230b95b37df6))
	ROM_LOAD("f900util.rom", 0x10000, 0x4000, CRC(bc6c7c66) SHA1(558b7383544542cf7333700ff90c3efbf93ba2a3))
	ROM_FILL(0x14000, 0x4000, 0x00)

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("f900kfn.rom", 0, 0x20000, CRC(5a59926e) SHA1(6acaf2eeb57f65f7408235d5e07b7563229de799))
ROM_END

void msx2_state::hbf900(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: wd2793, 2 3.5" DSDD drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 1, 0, 4).set_total_size(0x40000).set_ramio_bits(0x80);   /* 256KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "empty", 3, 3, 1, 1, "maincpu", 0x14000);    // Empty/unknown, optional fmpac rom used to be loaded here, or should the util rom be loaded?

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F900 (a) */
ROM_START(hbf900a)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("f900bios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("f900ext.rom",   0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("f900disa.rom",  0xc000, 0x4000, CRC(54c73ad6) SHA1(12f2cc79b3d09723840bae774be48c0d721ec1c6))
	ROM_LOAD("f900util.rom", 0x10000, 0x4000, CRC(bc6c7c66) SHA1(558b7383544542cf7333700ff90c3efbf93ba2a3))
	ROM_FILL(0x14000, 0x4000, 0x00)

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("f900kfn.rom", 0, 0x20000, CRC(5a59926e) SHA1(6acaf2eeb57f65f7408235d5e07b7563229de799))
ROM_END

void msx2_state::hbf900a(machine_config &config)
{
	msx2(config);
	// AY8910/YM2149?
	// FDC: wd2793, 2 3.5" DSDD drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 1, 0, 4).set_total_size(0x40000).set_ramio_bits(0x80);   /* 256KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "empty", 3, 3, 1, 1, "maincpu", 0x14000);    // Empty/unknown, optional fmpac rom used to be loaded here, or should the util rom be loaded?

	msx_wd2793(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-F9P */

ROM_START(hbf9p)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("f9pbios.rom.ic11",   0x0000, 0x8000, CRC(b31c851d) SHA1(0de3c802057560560a03d7965fcc4cff69f8575c))
	ROM_LOAD("f9pfirm1.ic12",      0x8000, 0x8000, CRC(524f67aa) SHA1(41a186afced50ca6312cb5b6c4adb684faca6232))
	ROM_LOAD("f9pfirm2.rom.ic13", 0x10000, 0x8000, CRC(ea97069f) SHA1(2d1880d1f5a6944fcb1b198b997a3d90ecd1903d))
ROM_END

void msx2_state::hbf9p(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "firm1", 3, 0, 0, 2, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "firm2", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Sony HB-F9P Russian */

ROM_START(hbf9pr)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("f9prbios.rom", 0x0000, 0x8000, CRC(39d7674a) SHA1(47642bb0a2c46a82100543dc3970d0a49fc53b69))
	ROM_LOAD("f9prext.rom",  0x8000, 0x4000, CRC(8b966f50) SHA1(65253cb38ab11084f355a2d4ad78fa6c64cbe660))
ROM_END

void msx2_state::hbf9pr(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000);   /* 128KB Mapper RAM */

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Sony HB-F9S */

ROM_START(hbf9s)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("f9sbios.ic11",   0x0000, 0x8000, CRC(c2b889a5) SHA1(4811956f878c3e03da46317f787cdc4bebc86f47))
	ROM_LOAD("f9sfirm1.ic12",  0x8000, 0x8000, CRC(cf39620b) SHA1(1166a93d7185ba024bdf2bfa9a30e1c447fb6db1))
	ROM_LOAD("f9sfirm2.ic13", 0x10000, 0x8000, CRC(ea97069f) SHA1(2d1880d1f5a6944fcb1b198b997a3d90ecd1903d))
ROM_END

void msx2_state::hbf9s(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "firm1", 3, 0, 0, 2, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "firm2", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Sony HB-G900AP */

/* IC109 - 32KB Basic ROM SLOT#00 0000-7FFF */
/* IC112 - 16KB Basic ROM SLOT#01 0000-3FFF */
/* IC117 - 16KB Disk ROM SLOT#01 4000-7FFF */
/* IC123 - 32KB ROM RS232C ROM SLOT#02 4000-7FFF / Video Utility ROM SLOT#03 4000-7FFF */

/* MSX2 - Sony HB-G900AP */
ROM_START(hbg900ap)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("g900bios.rom",  0x0000, 0x8000, CRC(b31c851d) SHA1(0de3c802057560560a03d7965fcc4cff69f8575c))
	ROM_LOAD("g900ext.rom",   0x8000, 0x4000, CRC(8f84f783) SHA1(3288894e1be6af705871499b23c85732dbc40993))
	ROM_LOAD("g900disk.rom",  0xc000, 0x4000, CRC(54c73ad6) SHA1(12f2cc79b3d09723840bae774be48c0d721ec1c6))
	ROM_LOAD("g900232c.rom", 0x10000, 0x4000, CRC(06cf1da6) SHA1(373aa82d0426830880a7344ef98f7309d93814c7))
	ROM_LOAD("g900util.rom", 0x14000, 0x4000, CRC(d0417c20) SHA1(8779b004e7605a3c419825f0373a5d8fa84e1d5b))
ROM_END

void msx2_state::hbg900ap(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 0, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
/*  MSX_LAYOUT_SLOT ("rs232c", 0, 2, 1, 1, "maincpu", 0x10000) */ /* RS232C must be emulated */
	add_internal_slot(config, MSX_SLOT_ROM, "util", 0, 3, 1, 1, "maincpu", 0x14000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x80000).set_ramio_bits(0x80);   /* 512KB Mapper RAM */

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Sony HB-G900P - 3x 32KB ROMs */

ROM_START(hbg900p)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("g900bios.rom",  0x0000, 0x8000, CRC(b31c851d) SHA1(0de3c802057560560a03d7965fcc4cff69f8575c))
	ROM_LOAD("g900ext.rom",   0x8000, 0x4000, CRC(8f84f783) SHA1(3288894e1be6af705871499b23c85732dbc40993))
	ROM_LOAD("g900disk.rom",  0xc000, 0x4000, CRC(54c73ad6) SHA1(12f2cc79b3d09723840bae774be48c0d721ec1c6))
	ROM_LOAD("g900232c.rom", 0x10000, 0x4000, CRC(06cf1da6) SHA1(373aa82d0426830880a7344ef98f7309d93814c7))
	ROM_LOAD("g900util.rom", 0x14000, 0x4000, CRC(d0417c20) SHA1(8779b004e7605a3c419825f0373a5d8fa84e1d5b))
ROM_END

void msx2_state::hbg900p(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 0, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
/*  MSX_LAYOUT_SLOT ("rs232c", 0, 2, 1, 1, "maincpu", 0x10000) */ /* RS232C must be emulated */
	add_internal_slot(config, MSX_SLOT_ROM, "util", 0, 3, 1, 1, "maincpu", 0x14000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Talent TPC-310 */
ROM_START(tpc310)
	ROM_REGION(0x1c000, "maincpu", 0)
	ROM_LOAD("tpc310bios.rom",   0x0000, 0x8000, CRC(8cd3e845) SHA1(7bba23669b7abfb6a142f9e1735b847d6e4e8267))
	ROM_LOAD("tpc310ext.rom",    0x8000, 0x4000, CRC(094a9e7a) SHA1(39dfc46260f99b670916b1e55f67a5d4136e6e54))
	ROM_LOAD("dpf550disk.rom",   0xc000, 0x4000, CRC(347b1b44) SHA1(c1d83c559e1e6a6da961eafa55aab105681c634c))
	ROM_LOAD("tpc310turbo.rom", 0x10000, 0x4000, CRC(0ea62a4d) SHA1(181bf58da7184e128cd419da3109b93344a543cf))
	ROM_LOAD("tpc310acc.rom",   0x14000, 0x8000, CRC(4fb8fab3) SHA1(cdeb0ed8adecaaadb78d5a5364fd603238591685))
ROM_END

void msx2_state::tpc310(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: mb8877a?, 1 3.5" DSDD drive
	// 1 Cartridge slot (slot 2)
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 1, 0, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "turbo", 3, 0, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_ROM, "acc", 3, 1, 1, 2, "maincpu", 0x14000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK2, "disk", 3, 2, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	msx_mb8877a(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Talent TPP-311 */

ROM_START(tpp311)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("311bios.rom", 0x0000, 0x8000, CRC(8cd3e845) SHA1(7bba23669b7abfb6a142f9e1735b847d6e4e8267))
	ROM_LOAD("311ext.rom",  0x8000, 0x4000, CRC(094a9e7a) SHA1(39dfc46260f99b670916b1e55f67a5d4136e6e54))
	ROM_LOAD("311logo.rom", 0xc000, 0x8000, CRC(0e6ecb9f) SHA1(e45ddc5bf1a1e63756d11fb43fc50276ca35cab0))
ROM_END

void msx2_state::tpp311(machine_config &config)
{
	msx2_pal(config, NO_CARTLIST);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 0 Cartridge slots?
	// 64KB VRAM

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 1, 0, 0, 4).set_total_size(0x10000);   /* 64KB?? Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "logo", 2, 0, 1, 2, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);

	msx2_64kb_vram(config);
}

/* MSX2 - Talent TPS-312 */

ROM_START(tps312)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("312bios.rom",   0x0000, 0x8000, CRC(8cd3e845) SHA1(7bba23669b7abfb6a142f9e1735b847d6e4e8267))
	ROM_LOAD("312ext.rom",    0x8000, 0x4000, CRC(094a9e7a) SHA1(39dfc46260f99b670916b1e55f67a5d4136e6e54))
	ROM_LOAD("312plan.rom",   0xc000, 0x8000, CRC(b3a6aaf6) SHA1(6de80e863cdd7856ab7aac4c238224a5352bda3b))
	ROM_LOAD("312write.rom", 0x14000, 0x4000, CRC(63c6992f) SHA1(93682f5baba7697c40088e26f99ee065c78e83b8))
ROM_END

void msx2_state::tps312(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?
	// 64KB VRAM

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 1, 0, 0, 4).set_total_size(0x20000);   /* 128KB?? Mapper RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "write", 3, 1, 1, 1, "maincpu", 0x14000);
	add_internal_slot(config, MSX_SLOT_ROM, "plan", 3, 2, 1, 1, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_ROM, "planlow", 3, 2, 0, 1, "maincpu", 0x10000);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 3, msx_cart, nullptr);

	msx2_64kb_vram(config);
}

/* MSX2 - Toshiba HX-23 */

ROM_START(hx23)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("hx23bios.rom", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("hx23ext.rom",  0x8000, 0x4000, CRC(06e4f5e6) SHA1(f5eb0a396097572589f2a6efeed045044e9425e4))
	ROM_LOAD("hx23word.rom", 0xc000, 0x8000, CRC(39b3e1c0) SHA1(9f7cfa932bd7dfd0d9ecaadc51655fb557c2e125))
ROM_END

void msx2_state::hx23(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?
	// 64KB VRAM

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_RAM, "ram1", 0, 0, 2, 2);   /* 32KB RAM */
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM, "ram2", 3, 0, 0, 2);   /* 32KB RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 3, 1, 2, "maincpu", 0xc000);

	msx2_64kb_vram(config);
}

/* MSX2 - Toshiba HX-23F */

ROM_START(hx23f)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("hx23fbios.rom", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("hx23fext.rom",  0x8000, 0x4000, CRC(06e4f5e6) SHA1(f5eb0a396097572589f2a6efeed045044e9425e4))
	ROM_LOAD("hx23fword.rom", 0xc000, 0x8000, CRC(39b3e1c0) SHA1(9f7cfa932bd7dfd0d9ecaadc51655fb557c2e125))
ROM_END

void msx2_state::hx23f(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 3, 1, 2, "maincpu", 0xc000);
}

/* MSX2 - Toshiba HX-23I */

ROM_START(hx23i)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("hx23ibios.rom", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("hx23iext.rom",  0x8000, 0x4000, CRC(06e4f5e6) SHA1(f5eb0a396097572589f2a6efeed045044e9425e4))
	ROM_LOAD("hx23iword.rom", 0xc000, 0x8000, CRC(d50db5b4) SHA1(64cf27a6be1393b1da9f8d5d43df617c9f22fbd2))
ROM_END

void msx2_state::hx23i(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985)
	// FDC: None, 0 drives
	// 2 Cartridge slots?
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x20000);   /* 128KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "word", 3, 3, 1, 2, "maincpu", 0xc000);

	MSX_S1985(config, "s1985", 0);
}

/* MSX@ - Toshiba HX-33 */

ROM_START(hx33)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("hx33bios.rom", 0x0000, 0x8000, CRC(3891e0f7) SHA1(7dfb18262d48e559fffb4199acbe29d9b4bee9db))
	ROM_LOAD("hx33ext.rom",  0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("hx33firm.rom", 0xc000, 0x8000, CRC(d05b5ca6) SHA1(7eea205044af48cfde9b7fff277d961704c4d45c))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("hx33kfn.rom", 0x0000, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx2_state::hx33(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985)
	// FDC: None, 0, drives
	// 2 Cartridge slots?
	// RS232C builtin?
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "firm", 3, 2, 1, 2, "maincpu", 0xc000);

	MSX_S1985(config, "s1985", 0);

	msx2_64kb_vram(config);
}

/* MSX@ - Toshiba HX-34 */

ROM_START(hx34)
ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("hx34bios.rom",  0x0000, 0x8000, CRC(3891e0f7) SHA1(7dfb18262d48e559fffb4199acbe29d9b4bee9db))
	ROM_LOAD("hx34ext.rom",   0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	// hx34disk.rom has contents of floppy registers at offset 3ff0-3ff7 and mirrored at 3ff8-3fff
	ROM_LOAD("hx34disk.rom",  0xc000, 0x4000, BAD_DUMP CRC(b6203bc8) SHA1(d31236e8b2491bca678d905546b365e9d365b072))
	ROM_LOAD("hx34firm.rom", 0x10000, 0x8000, CRC(d05b5ca6) SHA1(7eea205044af48cfde9b7fff277d961704c4d45c))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("hx34kfn.rom", 0x0000, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx2_state::hx34(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985)
	// FDC: wd2793??, 1 3.5" DSDD drive
	// 2 Cartridge slots?
	// RS232C builtin?
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_DISK6, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "firm", 3, 3, 1, 2, "maincpu", 0x10000);

	MSX_S1985(config, "s1985", 0);

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX@ - Toshiba HX-34I */

ROM_START(hx34i)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("hx34ibios.rom",  0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("hx34iext.rom",   0x8000, 0x4000, CRC(06e4f5e6) SHA1(f5eb0a396097572589f2a6efeed045044e9425e4))
	// hx34idisk.rom has contents of floppy registers at offset 3ff0-3ff7 and mirrored at 3ff8-3fff
	ROM_LOAD("hx34idisk.rom",  0xc000, 0x4000, BAD_DUMP CRC(b6203bc8) SHA1(d31236e8b2491bca678d905546b365e9d365b072))
	ROM_LOAD("hx34ifirm.rom", 0x10000, 0x8000, CRC(f9e29c66) SHA1(3289336b2c12161fd926a7e5ce865770ae7038af))
ROM_END

void msx2_state::hx34i(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985)
	// FDC: wd2793??, 1 3.5" DSDD drive
	// 2 Cartridge slots?
	// RS232C builtin?
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_DISK6, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "firm", 3, 3, 1, 2, "maincpu", 0x10000);

	MSX_S1985(config, "s1985", 0);

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Toshiba FS-TM1 */

ROM_START(fstm1)
	ROM_REGION(0x1c000, "maincpu", 0)
	ROM_LOAD("fstm1bios.rom",   0x0000, 0x8000, CRC(d1e11d52) SHA1(7a69e9b9595f3b0060155f4b419c915d4d9d8ca1))
	ROM_LOAD("fstm1ext.rom",    0x8000, 0x4000, CRC(4eebe9b1) SHA1(a4bdbdb20bf9fd3c492a890fbf541bf092eaa8e1))
	ROM_LOAD("fstm1desk1.rom",  0xc000, 0x8000, CRC(8b802086) SHA1(30737040d90c136d34dd409fe579bc4cca11c469))
	ROM_LOAD("fstm1desk2.rom", 0x14000, 0x8000, CRC(304820ea) SHA1(ff6e07d3976b0874164fae680ae028d598752049))
ROM_END

void msx2_state::fstm1(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-1985)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "desk1", 3, 2, 1, 2, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_ROM, "desk2", 3, 3, 1, 2, "maincpu", 0x14000);

	MSX_S1985(config, "s1985", 0);
}

/* MSX2 - Victor HC-90 */

ROM_START(victhc90)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("hc90bios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("hc90ext.rom",   0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("hc90disk.rom",  0xc000, 0x4000, CRC(11bca2ed) SHA1(a7a34671bddb48fa6c74182e2977f9129558ec32))
	ROM_LOAD("hc90firm.rom", 0x10000, 0x4000, CRC(53791d91) SHA1(caeffdd654394726c8c0824b21af7ff51c0b1031))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("hc90kfn.rom", 0x0000, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx2_state::victhc90(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985)
	// FDC: wd2793?, 1 3.5" DSDD drive
	// RS232C builtin
	// 2nd CPU HD-64B180 @ 6.144 MHz
	// 1 Cartridge slot (slot 1 or 2?)
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "firm", 0, 1, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot", 1, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Victor HC-95 */

ROM_START(victhc95)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("hc95bios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("hc95ext.rom",   0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("hc95disk.rom",  0xc000, 0x4000, CRC(11bca2ed) SHA1(a7a34671bddb48fa6c74182e2977f9129558ec32))
	ROM_LOAD("hc95firm.rom", 0x10000, 0x4000, CRC(53791d91) SHA1(caeffdd654394726c8c0824b21af7ff51c0b1031))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("hc95kfn.rom", 0x0000, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx2_state::victhc95(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985)
	// FDC: wd2793?, 2 3.5" DSDD drive
	// RS232C builtin
	// 2nd CPU HD-64B180 @ 6.144 MHz
	// 1 Cartridge slot (slot 1 or 2?)
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "firm", 0, 1, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot", 1, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Victor HC-95A */

ROM_START(victhc95a)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("hc95abios.rom",  0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("hc95aext.rom",   0x8000, 0x4000, CRC(4a48779c) SHA1(b8e30d604d319d511cbfbc61e5d8c38fbb9c5a33))
	ROM_LOAD("hc95adisk.rom",  0xc000, 0x4000, CRC(11bca2ed) SHA1(a7a34671bddb48fa6c74182e2977f9129558ec32))
	ROM_LOAD("hc95afirm.rom", 0x10000, 0x4000, CRC(53791d91) SHA1(caeffdd654394726c8c0824b21af7ff51c0b1031))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("hc95akfn.rom", 0x0000, 0x20000, CRC(d23d4d2d) SHA1(db03211b7db46899df41db2b1dfbec972109a967))
ROM_END

void msx2_state::victhc95a(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-1985)
	// FDC: wd2793?, 2 3.5" DSDD drive
	// RS232C builtin
	// 2nd CPU HD-64B180 @ 6.144 MHz
	// 1 Cartridge slot (slot 1 or 2?)
	// S-1985 MSX Engine
	// V9958 VDP

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "firm", 0, 1, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 0, 2, 0, 4).set_total_size(0x40000); // 256KB Mapper RAM
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot", 1, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");

	MSX_S1985(config, "s1985", 0);

	msx_wd2793_force_ready(config);
	msx_2_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Yamaha CX7M */

ROM_START(cx7m)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("cx7mbios.rom", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("cx7mext.rom",  0x8000, 0x4000, CRC(66237ecf) SHA1(5c1f9c7fb655e43d38e5dd1fcc6b942b2ff68b02))
ROM_END

void msx2_state::cx7m(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 3, msx_yamaha_60pin, "sfg05");
}

/* MSX2 - Yamaha CX7M/128 */

ROM_START(cx7m128)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("cx7mbios.rom", 0x0000, 0x8000, CRC(6cdaf3a5) SHA1(6103b39f1e38d1aa2d84b1c3219c44f1abb5436e))
	ROM_LOAD("cx7mext.rom",  0x8000, 0x4000, CRC(66237ecf) SHA1(5c1f9c7fb655e43d38e5dd1fcc6b942b2ff68b02))
	ROM_LOAD("yrm502.rom",   0xc000, 0x4000, CRC(51f7ddd1) SHA1(2a4b4a4657e3077df8a88f98210b76883d3702b1))
ROM_END

void msx2_state::cx7m128(machine_config &config)
{
	msx2_pal(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "yrm502", 3, 1, 1, 1, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000).set_ramio_bits(0x80);   /* 128KB Mapper RAM */
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 3, msx_yamaha_60pin, "sfg05");
}

/* MSX2 - Yamaha YIS-503 III R */

ROM_START(y503iiir)
	ROM_REGION(0x1c000, "maincpu", 0)
	ROM_LOAD("yis503iiirbios.rom", 0x0000, 0x8000, CRC(e7d08e29) SHA1(0f851ee7a1cf79819f61cc89e9948ee72a413802))
	ROM_LOAD("yis503iiirext.rom",  0x8000, 0x4000, CRC(34d21778) SHA1(03bf6d2ac86f5c9ab618e155442787c700f99fed))
	ROM_LOAD("yis503iiircpm.rom",  0xc000, 0x4000, CRC(417bf00e) SHA1(f4f7a54cdf5a9dd6c59f7cb219c2c5eb0a00fa8a))
	ROM_LOAD("yis503iiirnet.rom", 0x10000, 33121,  CRC(b10fb61c) SHA1(af2b7004a8888d7a72eee937783fccaca0f38621))  // Very odd size for a rom...
ROM_END

void msx2_state::y503iiir(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527)
	// FDC: wd2793?, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// Networking builtin
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "cpm", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 128KB Mapper RAM
	add_internal_slot(config, MSX_SLOT_ROM, "net", 3, 3, 0, 3, "maincpu", 0x10000);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Yamaha YIS-503 III R Estonian */

ROM_START(y503iiire)
	ROM_REGION(0x1c000, "maincpu", 0)
	ROM_LOAD("yis503iiirebios.rom", 0x0000, 0x8000, CRC(d0c20f54) SHA1(ebb7eb540a390509edfd36c84288ba85e63f2d1f))
	ROM_LOAD("yis503iiireext.rom",  0x8000, 0x4000, CRC(34d21778) SHA1(03bf6d2ac86f5c9ab618e155442787c700f99fed))
	ROM_LOAD("yis503iiirecpm.rom",  0xc000, 0x4000, CRC(417bf00e) SHA1(f4f7a54cdf5a9dd6c59f7cb219c2c5eb0a00fa8a))
	ROM_LOAD("yis503iiirenet.rom", 0x10000, 33121,  CRC(b10fb61c) SHA1(af2b7004a8888d7a72eee937783fccaca0f38621))  // Very odd size for a rom...
ROM_END

void msx2_state::y503iiire(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527)
	// FDC: wd2793?, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// Networking builtin
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "cpm", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 128KB Mapper RAM
	add_internal_slot(config, MSX_SLOT_ROM, "net", 3, 3, 0, 3, "maincpu", 0x10000);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Yamaha YIS604 */

ROM_START(yis60464)
	ROM_REGION(0xc000, "maincpu", 0)
	ROM_LOAD("yis604bios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("yis604ext.rom",  0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("yis604kfn.rom", 0x0000, 0x20000, CRC(5a59926e) SHA1(6acaf2eeb57f65f7408235d5e07b7563229de799))
ROM_END

void msx2_state::yis60464(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-3527)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 3, msx_yamaha_60pin, "sfg05");
}

/* MSX2 - Yamaha YIS604/128 */

ROM_START(yis604)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("yis604bios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("yis604ext.rom",  0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("yrm502.rom",     0xc000, 0x4000, CRC(51f7ddd1) SHA1(2a4b4a4657e3077df8a88f98210b76883d3702b1))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("yis604kfn.rom", 0x0000, 0x20000, CRC(5a59926e) SHA1(6acaf2eeb57f65f7408235d5e07b7563229de799))
ROM_END

void msx2_state::yis604(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-3527)
	// FDC: None, 0 drives
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "yrm502", 3, 1, 1, 1, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000); // 64KB Mapper RAM
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 3, msx_yamaha_60pin, "sfg05");
}

/* MSX2 - Yamaha YIS-805/128 */

ROM_START(y805128)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("yis805128bios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("yis805128ext.rom",  0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("yis805128disk.rom", 0xc000, 0x4000, CRC(ab94a273) SHA1(4b08a057e5863ade179dcf8bc9377e90940e6d61))
	ROM_LOAD("yrm502.rom",       0x10000, 0x4000, CRC(51f7ddd1) SHA1(2a4b4a4657e3077df8a88f98210b76883d3702b1))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("yis805128kfn.rom", 0x0000, 0x20000, CRC(5a59926e) SHA1(6acaf2eeb57f65f7408235d5e07b7563229de799))
ROM_END

void msx2_state::y805128(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-3527)
	// FDC: wd2793?, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "yrm502", 3, 1, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 128KB Mapper RAM
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 3, msx_yamaha_60pin, "sfg05");

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Yamaha YIS-805R2/128 */

ROM_START(y805128r2)
	ROM_REGION(0x28000, "maincpu", 0)
	ROM_LOAD("yis805128r2bios.rom",   0x0000,  0x8000, CRC(e7d08e29) SHA1(0f851ee7a1cf79819f61cc89e9948ee72a413802))
	ROM_LOAD("yis805128r2ext.rom",    0x8000,  0x4000, CRC(34d21778) SHA1(03bf6d2ac86f5c9ab618e155442787c700f99fed))
	ROM_LOAD("yis805128r2disk.rom",   0xc000,  0x4000, CRC(9eb7e24d) SHA1(3a481c7b7e4f0406a55952bc5b9f8cf9d699376c))
	ROM_LOAD("yis805128r2net.rom",   0x10000,  0x8000, CRC(0e345b43) SHA1(e8fd2bbc1bdab12c73a0fec178a190f9063547bb))
	ROM_LOAD("yis805128r2paint.rom", 0x18000, 0x10000, CRC(1bda68a3) SHA1(7fd2a28c4fdaeb140f3c8c8fb90271b1472c97b9))
ROM_END

void msx2_state::y805128r2(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527)
	// FDC: wd2793?, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine
	// Networking built in

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "paint", 3, 0, 0, 4, "maincpu", 0x18000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 128KB Mapper RAM
	add_internal_slot(config, MSX_SLOT_ROM, "net", 3, 3, 0, 2, "maincpu", 0x10000);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Yamaha YIS-805R2/128 Estonian */

ROM_START(y805128r2e)
	ROM_REGION(0x28000, "maincpu", 0)
	ROM_LOAD("yis805128r2ebios.rom",   0x0000,  0x8000, CRC(d0c20f54) SHA1(ebb7eb540a390509edfd36c84288ba85e63f2d1f))
	ROM_LOAD("yis805128r2eext.rom",    0x8000,  0x4000, CRC(34d21778) SHA1(03bf6d2ac86f5c9ab618e155442787c700f99fed))
	ROM_LOAD("yis805128r2edisk.rom",   0xc000,  0x4000, CRC(9eb7e24d) SHA1(3a481c7b7e4f0406a55952bc5b9f8cf9d699376c))
	ROM_LOAD("yis805128r2enet.rom",   0x10000,  0x8000, CRC(0e345b43) SHA1(e8fd2bbc1bdab12c73a0fec178a190f9063547bb))
	ROM_LOAD("yis805128r2epaint.rom", 0x18000, 0x10000, CRC(1bda68a3) SHA1(7fd2a28c4fdaeb140f3c8c8fb90271b1472c97b9))
ROM_END

void msx2_state::y805128r2e(machine_config &config)
{
	msx2_pal(config);
	// YM2149 (in S-3527)
	// FDC: wd2793?, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine
	// Networking built in

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "paint", 3, 0, 0, 4, "maincpu", 0x18000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 1, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x20000); // 128KB Mapper RAM
	add_internal_slot(config, MSX_SLOT_ROM, "net", 3, 3, 0, 2, "maincpu", 0x10000);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/* MSX2 - Yamaha YIS-805/256 */

ROM_START(y805256)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("yis805256bios.rom", 0x0000, 0x8000, CRC(9b3e7b97) SHA1(0081ea0d25bc5cd8d70b60ad8cfdc7307812c0fd))
	ROM_LOAD("yis805256ext.rom",  0x8000, 0x4000, CRC(43e7a7fc) SHA1(0fbd45ef3dd7bb82d4c31f1947884f411f1ca344))
	ROM_LOAD("yis805256disk.rom", 0xc000, 0x4000, CRC(ab94a273) SHA1(4b08a057e5863ade179dcf8bc9377e90940e6d61))
	ROM_LOAD("yrm502.rom",       0x10000, 0x4000, CRC(51f7ddd1) SHA1(2a4b4a4657e3077df8a88f98210b76883d3702b1))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("yis805256kfn.rom", 0x0000, 0x20000, CRC(5a59926e) SHA1(6acaf2eeb57f65f7408235d5e07b7563229de799))
ROM_END

void msx2_state::y805256(machine_config &config)
{
	msx2(config);
	// YM2149 (in S-3527)
	// FDC: wd2793?, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// S-3527 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 0, 1, 0, 1, "maincpu", 0x8000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 0, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "yrm502", 3, 1, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x40000); // 256KB Mapper RAM
	add_cartridge_slot<3>(config, MSX_SLOT_YAMAHA_EXPANSION, "expansion", 3, 3, msx_yamaha_60pin, "sfg05");

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2_floplist(config);
}

/********************************  MSX 2+ **********************************/

/* MSX2+ - Ciel Expert 3 IDE */

ROM_START(expert3i)
	ROM_REGION(0x24000, "maincpu", 0)
	ROM_LOAD("exp30bios.rom", 0x0000,  0x8000, CRC(a10bb1ce) SHA1(5029cf47031b22bd5d1f68ebfd3be6d6da56dfe9))
	ROM_LOAD("exp30ext.rom",  0x8000,  0x4000, CRC(6bcf4100) SHA1(cc1744c6c513d6409a142b4fb42fbe70a95d9b7f))
	ROM_LOAD("cieldisk.rom",  0xc000,  0x4000, CRC(bb550b09) SHA1(0274dd9b5096065a7f4ed019101124c9bd1d56b8))
	ROM_LOAD("exp30mus.rom", 0x10000,  0x4000, CRC(9881b3fd) SHA1(befebc916bfdb5e8057040f0ae82b5517a7750db))
	ROM_LOAD("ide240a.rom",  0x14000, 0x10000, CRC(7adf857f) SHA1(8a919dbeed92db8c06a611279efaed8552810239))
ROM_END

void msx2_state::expert3i(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: wd2793, 1 or 2? drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 1, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 1, 1, 1, 1, "maincpu", 0x10000).set_ym2413_tag("ym2413");
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 1, 2, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "ide", 1, 3, 0, 4, "maincpu", 0x14000);         /* IDE hardware needs to be emulated */
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 2, 0, 0, 4).set_total_size(0x40000);       /* 256KB?? Mapper RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);

	msx_ym2413(config);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Ciel Expert 3 Turbo */

/* Uses a Z84C0010 - CMOS processor working at 7 MHz */
ROM_START(expert3t)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("exp30bios.rom", 0x0000, 0x8000, CRC(a10bb1ce) SHA1(5029cf47031b22bd5d1f68ebfd3be6d6da56dfe9))
	ROM_LOAD("exp30ext.rom",  0x8000, 0x4000, CRC(6bcf4100) SHA1(cc1744c6c513d6409a142b4fb42fbe70a95d9b7f))
	ROM_LOAD("cieldisk.rom",  0xc000, 0x4000, CRC(bb550b09) SHA1(0274dd9b5096065a7f4ed019101124c9bd1d56b8))
	ROM_LOAD("exp30mus.rom", 0x10000, 0x4000, CRC(9881b3fd) SHA1(befebc916bfdb5e8057040f0ae82b5517a7750db))
	ROM_LOAD("turbo.rom",    0x14000, 0x4000, CRC(ab528416) SHA1(d468604269ae7664ac739ea9f922a05e14ffa3d1))
ROM_END

void msx2_state::expert3t(machine_config &config)
{
	msx2plus(config);
	// AY8910
	// FDC: wd2793?, 1 or 2? drives
	// 4 Cartridge/Expansion slots?
	// FM/YM2413 built-in

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 1, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 1, 1, 1, 1, "maincpu", 0x10000).set_ym2413_tag("ym2413");
	add_internal_slot(config, MSX_SLOT_ROM, "turbo", 1, 2, 1, 1, "maincpu", 0x14000);          /* Turbo hardware needs to be emulated */
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 1, 3, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 2, 0, 0, 4).set_total_size(0x40000);       /* 256KB Mapper RAM */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);

	msx_ym2413(config);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Gradiente Expert AC88+ */

ROM_START(expertac)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("ac88bios.rom", 0x0000, 0x8000, CRC(9ce0da44) SHA1(1fc2306911ab6e1ebdf7cb8c3c34a7f116414e88))
	ROM_LOAD("ac88ext.rom",  0x8000, 0x4000, CRC(c74c005c) SHA1(d5528825c7eea2cfeadd64db1dbdbe1344478fc6))
	ROM_LOAD("panadisk.rom", 0xc000, 0x4000, CRC(17fa392b) SHA1(7ed7c55e0359737ac5e68d38cb6903f9e5d7c2b6))
	ROM_LOAD("ac88asm.rom", 0x10000, 0x4000, CRC(a8a955ae) SHA1(91e522473a8470511584df3ee5b325ea5e2b81ef))
	ROM_LOAD("xbasic2.rom", 0x14000, 0x4000, CRC(2825b1a0) SHA1(47370bec7ca1f0615a54eda548b07fbc0c7ef398))
ROM_END

void msx2_state::expertac(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: wd2793?, 1 or 2? drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000);   /* 64KB Mapper RAM?? */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "asm", 3, 1, 1, 1, "maincpu", 0x10000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 2, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "xbasic", 3, 3, 1, 1, "maincpu", 0x14000);

	msx_wd2793_force_ready(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Gradiente Expert DDX+ */

ROM_START(expertdx)
	ROM_REGION(0x1c000, "maincpu", 0)
	ROM_LOAD("ddxbios.rom",  0x0000, 0x8000, CRC(e00af3dc) SHA1(5c463dd990582e677c8206f61035a7c54d8c67f0))
	ROM_LOAD("ddxext.rom",   0x8000, 0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("panadisk.rom", 0xc000, 0x4000, CRC(17fa392b) SHA1(7ed7c55e0359737ac5e68d38cb6903f9e5d7c2b6))
	ROM_LOAD("xbasic2.rom", 0x10000, 0x4000, CRC(2825b1a0) SHA1(47370bec7ca1f0615a54eda548b07fbc0c7ef398))
	ROM_LOAD("kanji.rom",   0x14000, 0x8000, CRC(b4fc574d) SHA1(dcc3a67732aa01c4f2ee8d1ad886444a4dbafe06))
ROM_END

void msx2_state::expertdx(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive?
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 1, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "xbasic", 1, 2, 1, 1, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 1, 3, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 2, 0, 0, 4).set_total_size(0x10000);   /* 64KB Mapper RAM?? */
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 3, 0, msx_cart, nullptr);
	/* Kanji? */

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Panasonic FS-A1FX */

ROM_START(fsa1fx)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("a1fxbios.rom",  0x0000, 0x8000, CRC(19771608) SHA1(e90f80a61d94c617850c415e12ad70ac41e66bb7))
	ROM_LOAD("a1fxext.rom",   0x8000, 0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("a1fxdisk.rom",  0xc000, 0x4000, CRC(2bda0184) SHA1(2a0d228afde36ac7c5d3c2aac9c7c664dd071a8c))
	ROM_LOAD("a1fxkdr.rom",  0x10000, 0x8000, CRC(a068cba9) SHA1(1ef3956f7f918873fb9b031339bba45d1e5e5878))
	ROM_LOAD("a1fxcock.rom", 0x18000, 0x8000, CRC(f662e6eb) SHA1(9d67fab55b85f4ac4f5924323a70020eb8589057))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("a1fxkfn.rom", 0, 0x20000, CRC(b244f6cf) SHA1(e0e99cd91e88ce2676445663f832c835d74d6fd4))
ROM_END

void msx2_state::fsa1fx(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "cock", 3, 3, 1, 2, "maincpu", 0x18000);

	msx_matsushita_device &matsushita(MSX_MATSUSHITA(config, "matsushita", 0));
	matsushita.turbo_callback().set(FUNC(msx2_state::turbo_w));

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0xff);

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Panasonic FS-A1WSX */

ROM_START(fsa1wsx)
	ROM_REGION(0x21c000, "maincpu", 0)
	ROM_LOAD("a1wsbios.rom",  0x0000,   0x8000, CRC(358ec547) SHA1(f4433752d3bf876bfefb363c749d4d2e08a218b6))
	ROM_LOAD("a1wsext.rom",   0x8000,   0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("a1wsdisk.rom",  0xc000,   0x4000, CRC(ac7d92b4) SHA1(b7068e2aab02072852ca249596b7550ac632c4c2))
	ROM_LOAD("a1wskdr.rom",  0x10000,   0x8000, CRC(b4fc574d) SHA1(dcc3a67732aa01c4f2ee8d1ad886444a4dbafe06))
	ROM_LOAD("a1wsmusp.rom", 0x18000,   0x4000, CRC(456e494e) SHA1(6354ccc5c100b1c558c9395fa8c00784d2e9b0a3))
	ROM_LOAD("a1wsfirm.rom", 0x1c000, 0x200000, CRC(e363595d) SHA1(3330d9b6b76e3c4ccb7cf252496ed15d08b95d3f))

	ROM_REGION(0x40000, "kanji", 0)
	ROM_LOAD("a1wskfn.rom", 0, 0x40000, CRC(1f6406fb) SHA1(5aff2d9b6efc723bc395b0f96f0adfa83cc54a49))
ROM_END

void msx2_state::fsa1wsx(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// FM built-in
	// No cassette port

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 0, 2, 1, 1, "maincpu", 0x18000).set_ym2413_tag("ym2413");
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_PANASONIC08, "firm", 3, 3, 0, 4, "maincpu", 0x1c000);

	msx_matsushita_device &matsushita(MSX_MATSUSHITA(config, "matsushita", 0));
	matsushita.turbo_callback().set(FUNC(msx2_state::turbo_w));

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0xff);

	msx_ym2413(config);

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Panasonic FS-A1WX */

ROM_START(fsa1wx)
	ROM_REGION(0x21c000, "maincpu", 0)
	ROM_LOAD("a1wxbios.rom",  0x0000,   0x8000, CRC(19771608) SHA1(e90f80a61d94c617850c415e12ad70ac41e66bb7))
	ROM_LOAD("a1wxext.rom",   0x8000,   0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("a1wxdisk.rom",  0xc000,   0x4000, CRC(2bda0184) SHA1(2a0d228afde36ac7c5d3c2aac9c7c664dd071a8c))
	ROM_LOAD("a1wxkdr.rom",  0x10000,   0x8000, CRC(a068cba9) SHA1(1ef3956f7f918873fb9b031339bba45d1e5e5878))
	ROM_LOAD("a1wxmusp.rom", 0x18000,   0x4000, CRC(456e494e) SHA1(6354ccc5c100b1c558c9395fa8c00784d2e9b0a3))
	ROM_LOAD("a1wxfirm.rom", 0x1c000, 0x200000, CRC(283f3250) SHA1(d37ab4bd2bfddd8c97476cbe7347ae581a6f2972))

	ROM_REGION(0x40000, "kanji", 0)
	ROM_LOAD("a1wxkfn.rom", 0, 0x40000, CRC(1f6406fb) SHA1(5aff2d9b6efc723bc395b0f96f0adfa83cc54a49))
ROM_END

void msx2_state::fsa1wx(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// FM built-in
	// MSX Engine T9769A

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 0, 2, 1, 1, "maincpu", 0x18000).set_ym2413_tag("ym2413");
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_PANASONIC08, "firm", 3, 3, 0, 4, "maincpu", 0x1c000);

	msx_matsushita_device &matsushita(MSX_MATSUSHITA(config, "matsushita", 0));
	matsushita.turbo_callback().set(FUNC(msx2_state::turbo_w));

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0xff);

	msx_ym2413(config);

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Panasonic FS-A1WX (a) */
ROM_START(fsa1wxa)
	ROM_REGION(0x21c000, "maincpu", 0)
	ROM_LOAD("a1wxbios.rom",  0x0000,   0x8000, CRC(19771608) SHA1(e90f80a61d94c617850c415e12ad70ac41e66bb7))
	ROM_LOAD("a1wxext.rom",   0x8000,   0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("a1wxdisk.rom",  0xc000,   0x4000, CRC(2bda0184) SHA1(2a0d228afde36ac7c5d3c2aac9c7c664dd071a8c))
	ROM_LOAD("a1wxkdr.rom",  0x10000,   0x8000, CRC(a068cba9) SHA1(1ef3956f7f918873fb9b031339bba45d1e5e5878))
	ROM_LOAD("a1wxmusp.rom", 0x18000,   0x4000, CRC(456e494e) SHA1(6354ccc5c100b1c558c9395fa8c00784d2e9b0a3))
	ROM_LOAD("a1wxfira.rom", 0x1c000, 0x200000, CRC(58440a8e) SHA1(8e0d4a77e7d5736e8225c2df4701509363eb230f))

	ROM_REGION(0x40000, "kanji", 0)
	ROM_LOAD("a1wxkfn.rom", 0, 0x40000, CRC(1f6406fb) SHA1(5aff2d9b6efc723bc395b0f96f0adfa83cc54a49))
ROM_END

void msx2_state::fsa1wxa(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots?
	// FM built-in

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 0, 2, 1, 1, "maincpu", 0x18000).set_ym2413_tag("ym2413");
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_PANASONIC08, "firm", 3, 3, 0, 4, "maincpu", 0x1c000);

	msx_matsushita_device &matsushita(MSX_MATSUSHITA(config, "matsushita", 0));
	matsushita.turbo_callback().set(FUNC(msx2_state::turbo_w));

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0xff);

	msx_ym2413(config);

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Sanyo Wavy PHC-35J */

ROM_START(phc35j)
	ROM_REGION(0x14000, "maincpu", 0)
	ROM_LOAD("35jbios.rom", 0x0000, 0x8000, CRC(358ec547) SHA1(f4433752d3bf876bfefb363c749d4d2e08a218b6))
	ROM_LOAD("35jext.rom",  0x8000, 0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("35jkdr.rom",  0xc000, 0x8000, CRC(b4fc574d) SHA1(dcc3a67732aa01c4f2ee8d1ad886444a4dbafe06))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("35jkfn.rom", 0, 0x20000, CRC(c9651b32) SHA1(84a645becec0a25d3ab7a909cde1b242699a8662))
ROM_END

void msx2_state::phc35j(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0xc000);

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0xff);
}

/* MSX2+ - Sanyo Wavy PHC-70FD1 */

ROM_START(phc70fd)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("70fdbios.rom", 0x0000, 0x8000, CRC(19771608) SHA1(e90f80a61d94c617850c415e12ad70ac41e66bb7))
	ROM_LOAD("70fdext.rom",  0x8000, 0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("70fddisk.rom", 0xc000, 0x4000, CRC(db7f1125) SHA1(9efa744be8355675e7bfdd3976bbbfaf85d62e1d))
	ROM_LOAD("70fdkdr.rom", 0x10000, 0x8000, CRC(a068cba9) SHA1(1ef3956f7f918873fb9b031339bba45d1e5e5878))
	ROM_LOAD("70fdmus.rom", 0x18000, 0x4000, CRC(5c32eb29) SHA1(aad42ba4289b33d8eed225d42cea930b7fc5c228))
	ROM_LOAD("70fdbas.rom", 0x1c000, 0x4000, CRC(da7be246) SHA1(22b3191d865010264001b9d896186a9818478a6b))

	ROM_REGION(0x20000, "kanji", 0)
	ROM_LOAD("70fdkfn.rom", 0, 0x20000, CRC(c9651b32) SHA1(84a645becec0a25d3ab7a909cde1b242699a8662))
ROM_END

void msx2_state::phc70fd(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// FM built-in

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 3, 3, 1, 1, "maincpu", 0x18000).set_ym2413_tag("ym2413");
	add_internal_slot(config, MSX_SLOT_ROM, "bas", 3, 3, 2, 1, "maincpu", 0x1c000);

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0xff);

	msx_ym2413(config);

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Sanyo Wavy PHC-70FD2 */
ROM_START(phc70fd2)
	ROM_REGION(0x20000, "maincpu", 0)
	ROM_LOAD("70f2bios.rom", 0x0000, 0x8000, CRC(19771608) SHA1(e90f80a61d94c617850c415e12ad70ac41e66bb7))
	ROM_LOAD("70f2ext.rom",  0x8000, 0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("70f2disk.rom", 0xc000, 0x4000, CRC(db7f1125) SHA1(9efa744be8355675e7bfdd3976bbbfaf85d62e1d))
	ROM_LOAD("70f2kdr.rom", 0x10000, 0x8000, CRC(a068cba9) SHA1(1ef3956f7f918873fb9b031339bba45d1e5e5878))
	ROM_LOAD("70f2mus.rom", 0x18000, 0x4000, CRC(5c32eb29) SHA1(aad42ba4289b33d8eed225d42cea930b7fc5c228))
	ROM_LOAD("70f2bas.rom", 0x1c000, 0x4000, CRC(da7be246) SHA1(22b3191d865010264001b9d896186a9818478a6b))

	ROM_REGION(0x40000, "kanji", 0)
	ROM_LOAD("70f2kfn.rom", 0, 0x40000, CRC(9a850db9) SHA1(bcdb4dae303dfe5234f372d70a5e0271d3202c36))
ROM_END

void msx2_state::phc70fd2(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 2 3.5" DSDD drives
	// 2 Cartridge slots
	// FM built-in

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_DISK3, "disk", 3, 2, 1, 1, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 3, 3, 1, 1, "maincpu", 0x18000).set_ym2413_tag("ym2413");
	add_internal_slot(config, MSX_SLOT_ROM, "bas", 3, 3, 2, 1, "maincpu", 0x1c000);

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0xff);

	msx_ym2413(config);

	msx_tc8566af(config);
	msx_2_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Sony HB-F1XDJ */

ROM_START(hbf1xdj)
	ROM_REGION(0x11c000, "maincpu", 0)
	ROM_LOAD("f1xjbios.rom",  0x0000,   0x8000, CRC(00870134) SHA1(e2fbd56e42da637609d23ae9df9efd1b4241b18a))
	ROM_LOAD("f1xjext.rom",   0x8000,   0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("f1xjdisk.rom",  0xc000,   0x4000, CRC(a21f5266) SHA1(c1bb307a570ab833e3bfcc4a58a4f4e12dc1df0f))
	ROM_LOAD("f1xjkdr.rom",  0x10000,   0x8000, CRC(a068cba9) SHA1(1ef3956f7f918873fb9b031339bba45d1e5e5878))
	ROM_LOAD("f1xjmus.rom",  0x18000,   0x4000, CRC(5c32eb29) SHA1(aad42ba4289b33d8eed225d42cea930b7fc5c228))
	ROM_LOAD("f1xjfirm.rom", 0x1c000, 0x100000, CRC(77be583f) SHA1(ade0c5ba5574f8114d7079050317099b4519e88f))

	ROM_REGION(0x40000, "kanji", 0)
	ROM_LOAD("f1xjkfn.rom", 0, 0x40000, CRC(7016dfd0) SHA1(218d91eb6df2823c924d3774a9f455492a10aecb))
ROM_END

void msx2_state::hbf1xdj(machine_config &config)
{
	msx2plus(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// FM built-in
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_SONY08, "firm", 0, 3, 0, 4, "maincpu", 0x1c000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 2, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 3, 3, 1, 1, "maincpu", 0x18000).set_ym2413_tag("ym2413");

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0x00);

	MSX_S1985(config, "s1985", 0);

	msx_ym2413(config);

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Sony HB-F1XV */

ROM_START(hbf1xv)
	ROM_REGION(0x11c000, "maincpu", 0)
	ROM_LOAD("f1xvbios.rom",  0x0000,   0x8000, CRC(2c7ed27b) SHA1(174c9254f09d99361ff7607630248ff9d7d8d4d6))
	ROM_LOAD("f1xvext.rom",   0x8000,   0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("f1xvdisk.rom",  0xc000,   0x4000, CRC(04e4e533) SHA1(5a4e7dbbfb759109c7d2a3b38bda9c60bf6ffef5))
	ROM_LOAD("f1xvkdr.rom",  0x10000,   0x8000, CRC(b4fc574d) SHA1(dcc3a67732aa01c4f2ee8d1ad886444a4dbafe06))
	ROM_LOAD("f1xvmus.rom",  0x18000,   0x4000, CRC(5c32eb29) SHA1(aad42ba4289b33d8eed225d42cea930b7fc5c228))
	ROM_LOAD("f1xvfirm.rom", 0x1c000, 0x100000, CRC(77be583f) SHA1(ade0c5ba5574f8114d7079050317099b4519e88f))

	ROM_REGION(0x40000, "kanji", 0)
	ROM_LOAD("f1xvkfn.rom", 0, 0x40000, CRC(7016dfd0) SHA1(218d91eb6df2823c924d3774a9f455492a10aecb))
ROM_END

void msx2_state::hbf1xv(machine_config &config)
{
	msx2plus(config);
	// YM2149 (in S-1985 MSX Engine)
	// FDC: wd2793, 1 3.5" DSDD drives
	// 2 Cartridge slots
	// FM built-in
	// S-1985 MSX Engine

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_SONY08, "firm", 0, 3, 0, 4, "maincpu", 0x1c000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x10000).set_ramio_bits(0x80);   /* 64KB Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot_mirrored(config, MSX_SLOT_DISK1, "disk", 3, 2, 1, 2, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 3, 3, 1, 1, "maincpu", 0x18000).set_ym2413_tag("ym2413");

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0x00);

	MSX_S1985(config, "s1985", 0);

	msx_ym2413(config);

	msx_wd2793(config);
	msx_1_35_dd_drive(config);
	msx2plus_floplist(config);
}

/* MSX2+ - Sony HB-F9S+ */

ROM_START(hbf9sp)
	ROM_REGION(0x18000, "maincpu", 0)
	ROM_LOAD("f9spbios.rom",  0x0000, 0x8000, CRC(994d3a80) SHA1(03556d380a9bd413faf1b9e3cbd7da47c7238775))
	ROM_LOAD("f9spext.rom",   0x8000, 0x4000, CRC(b8ba44d3) SHA1(fe0254cbfc11405b79e7c86c7769bd6322b04995))
	ROM_LOAD("f9psfrm1.rom",  0xc000, 0x4000, CRC(43d4cef1) SHA1(8948704bad9ff27873fa9ccd0ef89868e2bd6479))
	ROM_LOAD("f9spfrm2.rom", 0x10000, 0x8000, CRC(ea97069f) SHA1(2d1880d1f5a6944fcb1b198b997a3d90ecd1903d))
ROM_END

void msx2_state::hbf9sp(machine_config &config)
{
	msx2plus(config);
	// AY8910/YM2149?
	// FDC: None, 0 drives
	// 2 Cartridge slots?

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 0, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "firm1", 3, 0, 1, 1, "maincpu", 0xc000);
	add_internal_slot(config, MSX_SLOT_ROM, "firm2", 3, 1, 1, 2, "maincpu", 0x10000);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 2, 0, 4).set_total_size(0x10000);   /* 64KB?? Mapper RAM */

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0x00);
}

/* MSX Turbo-R - Panasonic FS-A1GT */

ROM_START(fsa1gt)
	ROM_REGION(0x46c000, "maincpu", 0)
	ROM_LOAD("a1gtbios.rom",  0x0000,   0x8000, CRC(937c8dbb) SHA1(242e73d8284a012b275c0a266844ebbc4269d787))
	ROM_LOAD("a1gtext.rom",   0x8000,   0x4000, CRC(70aea0fe) SHA1(018d7a5222f28514908fb1b1513286a6558a6d05))
	ROM_LOAD("a1gtdos.rom",   0xc000,  0x10000, CRC(bb2a0eae) SHA1(4880bf34f1c86fff5456ec2b4cf70d02339e2caa))
	ROM_LOAD("a1gtkdr.rom",  0x1c000,   0x8000, CRC(eaf0d125) SHA1(5b39c1ccd3a213b78e02927f56a9abc72cd8c28d))
	ROM_LOAD("a1gtmus.rom",  0x24000,   0x4000, CRC(f5f93437) SHA1(6aea1aef5ec31c1826c22edf580525f93baad425))
	ROM_LOAD("a1gtopt.rom",  0x28000,   0x4000, CRC(50d11f60) SHA1(b4433a3975c57dd440d6bf12dbd28b2ac1b90ef4))
	ROM_LOAD("a1gtkfn.rom",  0x2c000,  0x40000, CRC(1f6406fb) SHA1(5aff2d9b6efc723bc395b0f96f0adfa83cc54a49))
	ROM_LOAD("a1gtfirm.rom", 0x6c000, 0x400000, CRC(feefeadc) SHA1(e779c338eb91a7dea3ff75f3fde76b8af22c4a3a))
ROM_END

void msx2_state::fsa1gt(machine_config &config)
{
	turbor(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// FM built-in
	// MIDI

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 0, 2, 1, 1, "maincpu", 0x24000).set_ym2413_tag("ym2413");
	add_internal_slot(config, MSX_SLOT_ROM, "opt", 0, 3, 1, 1, "maincpu", 0x28000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 2, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x20000);   /* 128KB?? Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x1c000);
	add_internal_slot(config, MSX_SLOT_DISK4, "dos", 3, 2, 1, 3, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "firm", 3, 3, 0, 4, "maincpu", 0x6c000);

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0x00);

	msx_ym2413(config);

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msxr_floplist(config);
}

/* MSX Turbo-R - Panasonic FS-A1ST */

ROM_START(fsa1st)
	ROM_REGION(0x46c000, "maincpu", 0)
	ROM_LOAD("a1stbios.rom",  0x0000,   0x8000, CRC(77b94ae0) SHA1(f078b5ec56884bfb81481d45c7151418770bff5a))
	ROM_LOAD("a1stext.rom",   0x8000,   0x4000, CRC(2c2c77a4) SHA1(373412f9c32762de1c3a7e27fc3d80614e0a0c8e))
	ROM_LOAD("a1stdos.rom",   0xc000,  0x10000, CRC(1fc71407) SHA1(5d2186658adcf4ce0c2d3232384b5712341108e5))
	ROM_LOAD("a1stkdr.rom",  0x1c000,   0x8000, CRC(eaf0d125) SHA1(5b39c1ccd3a213b78e02927f56a9abc72cd8c28d))
	ROM_LOAD("a1stmus.rom",  0x24000,   0x4000, CRC(fd7dec41) SHA1(e002a9b426732e6c2d31e548c40cf7c122348ce3))
	ROM_LOAD("a1stopt.rom",  0x28000,   0x4000, CRC(c6a4a2a1) SHA1(cb06dea7b025745f9d2b87dcf03ded615287ead3))
	ROM_LOAD("a1stkfn.rom",  0x2c000,  0x40000, CRC(1f6406fb) SHA1(5aff2d9b6efc723bc395b0f96f0adfa83cc54a49))
	ROM_LOAD("a1stfirm.rom", 0x6c000, 0x400000, CRC(139ac99c) SHA1(c212b11fda13f83dafed688c54d098e7e47ab225))
ROM_END

void msx2_state::fsa1st(machine_config &config)
{
	turbor(config);
	// AY8910/YM2149?
	// FDC: tc8566af, 1 3.5" DSDD drive
	// 2 Cartridge slots
	// FM built-in

	add_internal_slot(config, MSX_SLOT_ROM, "bios", 0, 0, 0, 2, "maincpu", 0x0000);
	add_internal_slot(config, MSX_SLOT_MUSIC, "mus", 0, 2, 1, 1, "maincpu", 0x24000).set_ym2413_tag("ym2413");
	add_internal_slot(config, MSX_SLOT_ROM, "opt", 0, 3, 1, 1, "maincpu", 0x28000);
	add_cartridge_slot<1>(config, MSX_SLOT_CARTRIDGE, "cartslot1", 1, 0, msx_cart, nullptr);
	add_cartridge_slot<2>(config, MSX_SLOT_CARTRIDGE, "cartslot2", 1, 0, msx_cart, nullptr);
	add_internal_slot(config, MSX_SLOT_RAM_MM, "ram_mm", 3, 0, 0, 4).set_total_size(0x20000);   /* 128KB?? Mapper RAM */
	add_internal_slot(config, MSX_SLOT_ROM, "ext", 3, 1, 0, 1, "maincpu", 0x8000);
	add_internal_slot(config, MSX_SLOT_ROM, "kdr", 3, 1, 1, 2, "maincpu", 0x1c000);
	add_internal_slot(config, MSX_SLOT_DISK4, "dos", 3, 2, 1, 3, "maincpu", 0xc000).set_tags("fdc", "fdc:0", "fdc:1");
	add_internal_slot(config, MSX_SLOT_ROM, "firm", 3, 3, 0, 4, "maincpu", 0x6c000);

	MSX_SYSTEMFLAGS(config, "sysflags", m_maincpu, 0x00);

	msx_ym2413(config);

	msx_tc8566af(config);
	msx_1_35_dd_drive(config);
	msxr_floplist(config);
}

} // anonymous namespace


/*   YEAR  NAME        PARENT    COMPAT MACHINE     INPUT     CLASS      INIT        COMPANY       FULLNAME */
/* MSX1 */
COMP(1986, ax150,      0,        0,     ax150,      msx,      msx_state, empty_init, "Al Alamiah", "AX-150 (Arabic) (MSX1)", 0)
COMP(1986, ax170,      0,        0,     ax170,      msx,      msx_state, empty_init, "Al Alamiah", "AX-170 (Arabic) (MSX1)", 0)
COMP(1983, canonv8,    0,        0,     canonv8,    msx,      msx_state, empty_init, "Canon", "V-8 (MSX1)", 0)
COMP(1983, canonv10,   canonv20, 0,     canonv10,   msx,      msx_state, empty_init, "Canon", "V-10 (MSX1)", 0)
COMP(1983, canonv20,   0,        0,     canonv20,   msx,      msx_state, empty_init, "Canon", "V-20 (MSX1)", 0)
COMP(1983, canonv20e,  canonv20, 0,     canonv20,   msx,      msx_state, empty_init, "Canon", "V-20E (MSX1)", 0) // Different Euro keyboard layout?
COMP(1983, canonv20f,  canonv20, 0,     canonv20,   msx,      msx_state, empty_init, "Canon", "V-20F (MSX1)", 0) // Different French keyboard layout?
COMP(1983, canonv20g,  canonv20, 0,     canonv20,   msx,      msx_state, empty_init, "Canon", "V-20G (MSX1)", 0) // Different German keyboard layout?
COMP(1983, canonv20s,  canonv20, 0,     canonv20,   msx,      msx_state, empty_init, "Canon", "V-20S (MSX1)", 0) // Different Spanish keyboard layout?
COMP(1984, mx10,       0,        0,     mx10,       msx,      msx_state, empty_init, "Casio", "MX-10 (MSX1)", 0)
COMP(1984, mx101,      mx10,     0,     mx101,      msx,      msx_state, empty_init, "Casio", "MX-101 (MSX1)", 0)
COMP(1984, mx15,       mx10,     0,     mx15,       msx,      msx_state, empty_init, "Casio", "MX-15 (MSX1)", 0)
COMP(1984, pv7,        0,        0,     pv7,        msx,      msx_state, empty_init, "Casio", "PV-7 (MSX1)", 0)
COMP(1984, pv16,       0,        0,     pv16,       msx,      msx_state, empty_init, "Casio", "PV-16 (MSX1)", 0)
COMP(198?, cpc88,      0,        0,     cpc88,      msxkr,    msx_state, empty_init, "Daewoo", "CPC-88 (Korea) (MSX1)", 0)
COMP(1984, dpc100,     dpc200,   0,     dpc100,     msxkr,    msx_state, empty_init, "Daewoo", "IQ-1000 DPC-100 (Korea) (MSX1)", 0)
COMP(1984, dpc180,     dpc200,   0,     dpc180,     msxkr,    msx_state, empty_init, "Daewoo", "IQ-1000 DPC-180 (Korea) (MSX1)", 0)
COMP(1984, dpc200,     0,        0,     dpc200,     msxkr,    msx_state, empty_init, "Daewoo", "IQ-1000 DPC-200 (Korea) (MSX1)", 0)
COMP(1985, dpc200e,    0,        0,     dpc200e,    msx,      msx_state, empty_init, "Daewoo", "DPC-200E (MSX1)", 0)
COMP(1983, cpc50a,     cpc51,    0,     cpc50a,     msxkr,    msx_state, empty_init, "Daewoo", "Zemmix CPC-50A (Korea) (MSX1)", 0)
COMP(1983, cpc50b,     cpc51,    0,     cpc50b,     msxkr,    msx_state, empty_init, "Daewoo", "Zemmix CPC-50B (Korea) (MSX1)", 0)
COMP(1986, cpc51,      0,        0,     cpc51,      msxkr,    msx_state, empty_init, "Daewoo", "Zemmix CPC-51 (Korea) (MSX1)", 0)
COMP(1985, dgnmsx,     0,        0,     dgnmsx,     msx,      msx_state, empty_init, "Eurohard S.A.", "Dragon MSX-64 (MSX1)", 0)
COMP(1983, fdpc200,    0,        0,     fdpc200,    msx,      msx_state, empty_init, "Fenner", "DPC-200 (Italy) (MSX1)", 0)
COMP(1984, fpc500,     0,        0,     fpc500,     msx,      msx_state, empty_init, "Fenner", "FPC-500 (Italy) (MSX1)", 0)
COMP(1986, fspc800,    0,        0,     fspc800,    msx,      msx_state, empty_init, "Fenner", "SPC-800 (Italy) (MSX1)", 0)
COMP(1984, bruc100,    0,        0,     bruc100,    msx,      msx_state, empty_init, "Frael", "Bruc 100-1 (MSX1)", 0)
COMP(1983, fmx,        0,        0,     fmx,        msxjp,    msx_state, empty_init, "Fujitsu", "FM-X (MSX1)", 0)
COMP(1984, gsfc80u,    0,        0,     gsfc80u,    msxkr,    msx_state, empty_init, "Goldstar", "FC-80U (MSX1)", 0)
COMP(1983, gsfc200,    0,        0,     gsfc200,    msx,      msx_state, empty_init, "Goldstar", "FC-200 (MSX1)", 0)
COMP(198?, gfc1080,    0,        0,     gfc1080,    msxkr,    msx_state, empty_init, "Goldstar", "GFC-1080 (MSX1)", 0)
COMP(198?, gfc1080a,   0,        0,     gfc1080a,   msxkr,    msx_state, empty_init, "Goldstar", "GFC-1080A (MSX1)", 0)
COMP(1983, expert10,   expert13, 0,     expert10,   expert10, msx_state, empty_init, "Gradiente", "Expert 1.0 (Brazil) (MSX1)", 0)
COMP(1984, expert11,   expert13, 0,     expert11,   expert11, msx_state, empty_init, "Gradiente", "Expert 1.1 (Brazil) (MSX1)", 0)
COMP(1984, expert13,   0,        0,     expert13,   expert11, msx_state, empty_init, "Gradiente", "Expert 1.3 (Brazil) (MSX1)", 0)
COMP(1985, expertdp,   0,        0,     expertdp,   expert11, msx_state, empty_init, "Gradiente", "Expert DDPlus (Brazil) (MSX1)", 0)
COMP(1984, expertpl,   0,        0,     expertpl,   expert11, msx_state, empty_init, "Gradiente", "Expert Plus (Brazil) (MSX1)", 0)
COMP(1984, mbh2,       0,        0,     mbh2,       msxjp,    msx_state, empty_init, "Hitachi", "MB-H2 (MSX1)", 0)
COMP(1984, mbh25,      0,        0,     mbh25,      msxjp,    msx_state, empty_init, "Hitachi", "MB-H25 (MSX1)", 0)
COMP(1983, mbh50,      0,        0,     mbh50,      msxjp,    msx_state, empty_init, "Hitachi", "MB-H50 (MSX1)", 0)
COMP(1983, jvchc7gb,   0,        0,     jvchc7gb,   msx,      msx_state, empty_init, "JVC", "HC-7GB (MSX1)", 0)
COMP(198?, mlf48,      0,        0,     mlf48,      msx,      msx_state, empty_init, "Mitsubishi", "ML-F48 (MSX1)", 0)
COMP(1983, mlf80,      0,        0,     mlf80,      msx,      msx_state, empty_init, "Mitsubishi", "ML-F80 (MSX1)", 0)
COMP(1984, mlf110,     0,        0,     mlf110,     msxjp,    msx_state, empty_init, "Mitsubishi", "ML-F110 (MSX1)", 0)
COMP(1984, mlf120,     0,        0,     mlf120,     msxjp,    msx_state, empty_init, "Mitsubishi", "ML-F120 (MSX1)", 0)
COMP(1983, mlfx1,      0,        0,     mlfx1,      msx,      msx_state, empty_init, "Mitsubishi", "ML-FX1 (MSX1)", 0)
COMP(1984, cf1200,     0,        0,     cf1200,     msxjp,    msx_state, empty_init, "National / Matsushita", "CF-1200 (Japan) (MSX1)", 0)
COMP(1983, cf2000,     0,        0,     cf2000,     msxjp,    msx_state, empty_init, "National / Matsushita", "CF-2000 (Japan) (MSX1)", 0)
COMP(1984, cf2700,     0,        0,     cf2700,     msxjp,    msx_state, empty_init, "National / Matsushita", "CF-2700 (Japan) (MSX1)", 0)
COMP(1984, cf3000,     0,        0,     cf3000,     msxjp,    msx_state, empty_init, "National / Matsushita", "CF-3000 (Japan) (MSX1)", 0)
COMP(1985, cf3300,     0,        0,     cf3300,     msxjp,    msx_state, empty_init, "National / Matsushita", "CF-3300 (Japan) (MSX1)", 0)
COMP(1985, fs1300,     0,        0,     fs1300,     msxjp,    msx_state, empty_init, "National / Matsushita", "FS-1300 (Japan) (MSX1)", 0)
COMP(1985, fs4000,     0,        0,     fs4000,     msxjp,    msx_state, empty_init, "National / Matsushita", "FS-4000 (Japan) (MSX1)", 0)
COMP(1985, fs4000a,    fs4000,   0,     fs4000a,    msxjp,    msx_state, empty_init, "National / Matsushita", "FS-4000 (alt) (Japan) (MSX1)", 0)
COMP(1983, phc2,       0,        0,     phc2,       msx,      msx_state, empty_init, "Olympia", "PHC-2 (MSX1)" , 0)
COMP(19??, phc28,      0,        0,     phc28,      msx,      msx_state, empty_init, "Olympia", "PHC-28 (MSX1)", 0)
COMP(1984, cf2700g,    0,        0,     cf2700g,    msx,      msx_state, empty_init, "Panasonic", "CF-2700G (Germany) (MSX1)", 0)
COMP(198?, perfect1,   0,        0,     perfect1,   msx,      msx_state, empty_init, "Perfect", "Perfect1 (MSX1)", MACHINE_NOT_WORKING)
COMP(1983, nms801,     0,        0,     nms801,     msx,      msx_state, empty_init, "Philips", "NMS-801 (MSX1)", 0)
COMP(1984, vg8000,     vg8010,   0,     vg8000,     msx,      msx_state, empty_init, "Philips", "VG-8000 (MSX1)", 0)
COMP(1984, vg8010,     0,        0,     vg8010,     msx,      msx_state, empty_init, "Philips", "VG-8010 (MSX1)", 0)
COMP(1984, vg8010f,    vg8010,   0,     vg8010f,    msx,      msx_state, empty_init, "Philips", "VG-8010F (MSX1)" , 0)
COMP(1985, vg802000,   vg802020, 0,     vg802000,   msx,      msx_state, empty_init, "Philips", "VG-8020-00 (MSX1)", 0)
COMP(1985, vg802020,   0,        0,     vg802020,   msx,      msx_state, empty_init, "Philips", "VG-8020-20 (MSX1)", 0)
COMP(19??, vg8020f,    vg802020, 0,     vg8020f,    msx,      msx_state, empty_init, "Philips", "VG-8020F (MSX1)", 0)
COMP(1985, piopx7,     0,        0,     piopx7,     msx,      msx_state, empty_init, "Pioneer", "PX-07 Palcom (MSX1)", 0)
COMP(1985, piopx7uk,   piopx7,   0,     piopx7uk,   msx,      msx_state, empty_init, "Pioneer", "PX-07UK Palcom (MSX1)", 0)
COMP(1984, piopxv60,   piopx7,   0,     piopxv60,   msxjp,    msx_state, empty_init, "Pioneer", "PX-V60 (MSX1)", 0)
COMP(19??, spc800,     0,        0,     spc800,     msx,      msx_state, empty_init, "Samsung", "SPC-800 (MSX1)", 0)
COMP(1985, mpc64,      0,        0,     mpc64,      msxjp,    msx_state, empty_init, "Sanyo", "MPC-64 (MSX1)", 0)
COMP(1985, mpc100,     0,        0,     mpc100,     msx,      msx_state, empty_init, "Sanyo", "MPC-100 (MSX1)", 0)
COMP(1983, mpc200,     0,        0,     mpc200,     msx,      msx_state, empty_init, "Sanyo", "MPC-200 (MSX1)", 0)
COMP(1983, mpc200sp,   mpc200,   0,     mpc200sp,   msx,      msx_state, empty_init, "Sanyo", "MPC-200SP (MSX1)", 0) // Spanish keyboard?
COMP(1983, phc28l,     0,        0,     phc28l,     msx,      msx_state, empty_init, "Sanyo", "PHC-28L (MSX1)", 0)
COMP(1983, phc28s,     0,        0,     phc28s,     msx,      msx_state, empty_init, "Sanyo", "PHC-28S (MSX1)", 0)
COMP(19??, mpc10,      0,        0,     mpc10,      msx,      msx_state, empty_init, "Sanyo", "Wavy MPC-10 (MSX1)", 0)
COMP(1985, hotbit11,   hotbi13p, 0,     hotbit11,   hotbit,   msx_state, empty_init, "Sharp / Epcom", "HB-8000 Hotbit 1.1 (MSX1)", 0)
COMP(1985, hotbit12,   hotbi13p, 0,     hotbit12,   hotbit,   msx_state, empty_init, "Sharp / Epcom", "HB-8000 Hotbit 1.2 (MSX1)", 0)
COMP(1985, hotbi13b,   hotbi13p, 0,     hotbi13b,   hotbit,   msx_state, empty_init, "Sharp / Epcom", "HB-8000 Hotbit 1.3b (MSX1)", 0)
COMP(1985, hotbi13p,   0,        0,     hotbi13p,   hotbit,   msx_state, empty_init, "Sharp / Epcom", "HB-8000 Hotbit 1.3p (MSX1)", 0)
COMP(198?, hb10,       hb10p,    0,     hb10,       msxjp,    msx_state, empty_init, "Sony", "HB-10 (MSX1)", 0)
COMP(1985, hb10p,      0,        0,     hb10p,      msx,      msx_state, empty_init, "Sony", "HB-10P (MSX1)", 0)
COMP(1984, hb101p,     0,        0,     hb101p,     msx,      msx_state, empty_init, "Sony", "HB-101P (MSX1)", 0)
COMP(1985, hb20p,      0,        0,     hb20p,      msx,      msx_state, empty_init, "Sony", "HB-20P (Spanish) (MSX1)", 0)
COMP(1985, hb201,      hb201p,   0,     hb201,      msxjp,    msx_state, empty_init, "Sony", "HB-201 (Japan) (MSX1)", 0)
COMP(1985, hb201p,     0,        0,     hb201p,     msx,      msx_state, empty_init, "Sony", "HB-201P (MSX1)", 0)
COMP(1984, hb501p,     0,        0,     hb501p,     msx,      msx_state, empty_init, "Sony", "HB-501P (MSX1)", 0)
COMP(1983, hb55,       hb55p,    0,     hb55,       msxjp,    msx_state, empty_init, "Sony", "HB-55 (MSX1)", 0)
COMP(1983, hb55d,      hb55p,    0,     hb55d,      msx,      msx_state, empty_init, "Sony", "HB-55D (Germany) (MSX1)", 0)
COMP(1983, hb55p,      0,        0,     hb55p,      msx,      msx_state, empty_init, "Sony", "HB-55P (MSX1)", 0)
COMP(1984, hb701fd,    0,        0,     hb701fd,    msxjp,    msx_state, empty_init, "Sony", "HB-701FD (MSX1)", 0)
COMP(1983, hb75d,      hb75p,    0,     hb75d,      msx,      msx_state, empty_init, "Sony", "HB-75D (Germany) (MSX1)", 0)
COMP(1983, hb75p,      0,        0,     hb75p,      msx,      msx_state, empty_init, "Sony", "HB-75P (MSX1)", 0)
COMP(1985, svi728,     0,        0,     svi728,     msx,      msx_state, empty_init, "Spectravideo", "SVI-728 (MSX1)", 0)
COMP(1985, svi738,     0,        0,     svi738,     msx,      msx_state, empty_init, "Spectravideo", "SVI-738 (MSX1)", 0)
COMP(1983, svi738ar,   svi738,   0,     svi738ar,   msx,      msx_state, empty_init, "Spectravideo", "SVI-738 (Arabic) (MSX1)", 0)
COMP(1983, svi738dk,   svi738,   0,     svi738dk,   msx,      msx_state, empty_init, "Spectravideo", "SVI-738 (Denmark) (MSX1)", 0)
COMP(1983, svi738sp,   svi738,   0,     svi738sp,   msx,      msx_state, empty_init, "Spectravideo", "SVI-738 (Spain) (MSX1)", 0)
COMP(1983, svi738sw,   svi738,   0,     svi738sw,   msx,      msx_state, empty_init, "Spectravideo", "SVI-738 (Swedish) (MSX1)", 0)
COMP(1983, svi738pl,   svi738,   0,     svi738pl,   msx,      msx_state, empty_init, "Spectravideo", "SVI-738 (Poland) (MSX1)", 0)
COMP(1983, tadpc200,   dpc200,   0,     tadpc200,   msx,      msx_state, empty_init, "Talent", "DPC-200 (MSX1)", 0)
COMP(1983, tadpc20a,   dpc200,   0,     tadpc20a,   msx,      msx_state, empty_init, "Talent", "DPC-200A (MSX1)", 0)
COMP(1984, hx10,       0,        0,     hx10,       msx,      msx_state, empty_init, "Toshiba", "HX-10 (MSX1)", 0)
COMP(1984, hx10d,      hx10,     0,     hx10d,      msxjp,    msx_state, empty_init, "Toshiba", "HX-10D (MSX1)", 0)
COMP(1984, hx10dp,     hx10,     0,     hx10dp,     msxjp,    msx_state, empty_init, "Toshiba", "HX-10DP (MSX1)", 0)
COMP(1984, hx10e,      hx10,     0,     hx10e,      msx,      msx_state, empty_init, "Toshiba", "HX-10E (MSX1)", 0)
COMP(1984, hx10f,      hx10,     0,     hx10f,      msx,      msx_state, empty_init, "Toshiba", "HX-10F (MSX1)", 0)
COMP(1984, hx10s,      hx10,     0,     hx10s,      msx,      msx_state, empty_init, "Toshiba", "HX-10S (MSX1)", 0)
COMP(1984, hx10sa,     hx10,     0,     hx10sa,     msxjp,    msx_state, empty_init, "Toshiba", "HX-10SA (MSX1)", 0)
COMP(1984, hx20,       0,        0,     hx20,       msx,      msx_state, empty_init, "Toshiba", "HX-20 (MSX1)", 0)
COMP(1984, hx20i,      hx20,     0,     hx20i,      msx,      msx_state, empty_init, "Toshiba", "HX-20I (MSX1)", 0)
COMP(1984, hx21,       0,        0,     hx21,       msxjp,    msx_state, empty_init, "Toshiba", "HX-21 (MSX1)", MACHINE_NOT_WORKING) // Does not go into firmware
COMP(1984, hx21i,      hx21,     0,     hx21i,      msx,      msx_state, empty_init, "Toshiba", "HX-21I (MSX1)", 0)
COMP(1984, hx22,       0,        0,     hx22,       msxjp,    msx_state, empty_init, "Toshiba", "HX-22 (MSX1)", MACHINE_NOT_WORKING) // Does not go into firmware
COMP(1984, hx22i,      hx22,     0,     hx22i,      msx,      msx_state, empty_init, "Toshiba", "HX-22I (MSX1)", 0)
COMP(198?, hc5,        hc7,      0,     hc5,        msxjp,    msx_state, empty_init, "Victor", "HC-5 (MSX1)", 0)
COMP(198?, hc6,        hc7,      0,     hc6,        msxjp,    msx_state, empty_init, "Victor", "HC-6 (MSX1)", 0)
COMP(198?, hc7,        0,        0,     hc7,        msxjp,    msx_state, empty_init, "Victor", "HC-7 (MSX1)", 0)
COMP(1984, cx5f1,      cx5f,     0,     cx5f1,      msxjp,    msx_state, empty_init, "Yamaha", "CX5F (w/SFG01) (MSX1)", 0)
COMP(1984, cx5f,       0,        0,     cx5f,       msxjp,    msx_state, empty_init, "Yamaha", "CX5F (w/SFG05) (MSX1)", 0)
COMP(1984, cx5m,       cx5m128,  0,     cx5m,       msx,      msx_state, empty_init, "Yamaha", "CX5M (MSX1)", 0)
COMP(1984, cx5m128,    0,        0,     cx5m128,    msx,      msx_state, empty_init, "Yamaha", "CX5M-128 (MSX1)", 0)
COMP(1984, cx5m2,      cx5m128,  0,     cx5m2,      msx,      msx_state, empty_init, "Yamaha", "CX5MII (MSX1)", 0)
COMP(1984, yis303,     0,        0,     yis303,     msx,      msx_state, empty_init, "Yamaha", "YIS303 (MSX1)", 0)
COMP(1984, yis503,     0,        0,     yis503,     msx,      msx_state, empty_init, "Yamaha", "YIS503 (MSX1)", 0)
COMP(19??, yis503f,    yis503,   0,     yis503f,    msx,      msx_state, empty_init, "Yamaha", "YIS503F (MSX1)", 0)
COMP(1984, yis503ii,   yis503,   0,     yis503ii,   msx,      msx_state, empty_init, "Yamaha", "YIS503II (MSX1)", 0)
COMP(1986, y503iir,    yis503,   0,     y503iir,    msx,      msx_state, empty_init, "Yamaha", "YIS503IIR (Russian) (MSX1)", 0)
COMP(1986, y503iir2,   yis503,   0,     y503iir2,   msx,      msx_state, empty_init, "Yamaha", "YIS503IIR (Estonian) (MSX1)", 0)
COMP(1984, yis503m,    yis503,   0,     yis503m,    msx,      msx_state, empty_init, "Yamaha", "YIS503M (MSX1)", 0)
COMP(1984, yc64,       0,        0,     yc64,       msx,      msx_state, empty_init, "Yashica", "YC-64 (MSX1)", 0)
COMP(1984, mx64,       0,        0,     mx64,       msxkr,    msx_state, empty_init, "Yeno", "MX64 (MSX1)", 0)

/* MSX2 */
COMP(1986, ax350,      0,        0,     ax350,      msx2,     msx2_state, empty_init, "Al Alamiah", "AX-350 (Arabic) (MSX2)", 0)
COMP(1986, ax370,      0,        0,     ax370,      msx2,     msx2_state, empty_init, "Al Alamiah", "AX-370 (Arabic) (MSX2)", 0)
COMP(1985, canonv25,   0,        0,     canonv25,   msx2,     msx2_state, empty_init, "Canon", "V-25 (MSX2)", 0)
COMP(1985, canonv30,   0,        0,     canonv30,   msx2,     msx2_state, empty_init, "Canon", "V-30 (MSX2)", 0)
COMP(1985, canonv30f,  canonv30, 0,     canonv30f,  msx2,     msx2_state, empty_init, "Canon", "V-30F (MSX2)", 0)
COMP(1986, cpc300,     0,        0,     cpc300,     msx2kr,   msx2_state, empty_init, "Daewoo", "IQ-2000 CPC-300 (Korea) (MSX2)", 0)
COMP(1986, cpc300e,    0,        0,     cpc300e,    msx2kr,   msx2_state, empty_init, "Daewoo", "IQ-2000 CPC-300E (Korea) (MSX2)", 0)
COMP(1985, cpc330k,    0,        0,     cpc330k,    msx2kr,   msx2_state, empty_init, "Daewoo", "CPC-330K KOBO (Korea) (MSX2)", 0)
COMP(1988, cpc400,     0,        0,     cpc400,     msx2kr,   msx2_state, empty_init, "Daewoo", "X-II CPC-400 (Korea) (MSX2)", 0)
COMP(1988, cpc400s,    0,        0,     cpc400s,    msx2kr,   msx2_state, empty_init, "Daewoo", "X-II CPC-400S (Korea) (MSX2)", 0)
COMP(1990, cpc61,      0,        0,     cpc61,      msx2kr,   msx2_state, empty_init, "Daewoo", "Zemmix CPC-61 (Korea) (MSX2)", 0)
COMP(1991, cpg120,     0,        0,     cpg120,     msx2kr,   msx2_state, empty_init, "Daewoo", "Zemmix CPG-120 Normal (Korea) (MSX2)", MACHINE_NOT_WORKING) // v9958 not added
COMP(198?, fpc900,     0,        0,     fpc900,     msx2,     msx2_state, empty_init, "Fenner", "FPC-900 (MSX2)", 0)
COMP(1986, expert20,   0,        0,     expert20,   msx2,     msx2_state, empty_init, "Gradiente", "Expert 2.0 (Brazil) (MSX2)", 0)
COMP(198?, mbh70,      0,        0,     mbh70,      msx2jp,   msx2_state, empty_init, "Hitachi", "MB-H70 (MSX2)", MACHINE_NOT_WORKING) // Firmware not working
COMP(1987, kmc5000,    0,        0,     kmc5000,    msx2jp,   msx2_state, empty_init, "Kawai", "KMC-5000 (MSX2)", 0)
COMP(1985, mlg1,       0,        0,     mlg1,       msx2,     msx2_state, empty_init, "Mitsubishi", "ML-G1 (MSX2)", 0)
COMP(198?, mlg3,       0,        0,     mlg3,       msx2,     msx2_state, empty_init, "Mitsubishi", "ML-G3 (MSX2)", 0)
COMP(1985, mlg10,      0,        0,     mlg10,      msx2jp,   msx2_state, empty_init, "Mitsubishi", "ML-G10 (MSX2)", 0)
COMP(1983, mlg30,      0,        0,     mlg30,      msx2,     msx2_state, empty_init, "Mitsubishi", "ML-G30 (MSX2)", 0)
COMP(1985, fs5500f1,   fs5500f2, 0,     fs5500f1,   msx2jp,   msx2_state, empty_init, "National / Matsushita", "FS-5500F1 (Japan) (MSX2)", 0)
COMP(1985, fs5500f2,   0,        0,     fs5500f2,   msx2jp,   msx2_state, empty_init, "National / Matsushita", "FS-5500F2 (Japan) (MSX2)", 0)
COMP(1986, fs4500,     0,        0,     fs4500,     msx2jp,   msx2_state, empty_init, "National / Matsushita", "FS-4500 (Japan) (MSX2)", 0)
COMP(1986, fs4700,     0,        0,     fs4700,     msx2jp,   msx2_state, empty_init, "National / Matsushita", "FS-4700 (Japan) (MSX2)", 0)
COMP(1986, fs5000,     0,        0,     fs5000,     msx2jp,   msx2_state, empty_init, "National / Matsushita", "FS-5000F2 (Japan) (MSX2)", 0)
COMP(1986, fs4600,     0,        0,     fs4600,     msx2jp,   msx2_state, empty_init, "National / Matsushita", "FS-4600 (Japan) (MSX2)", 0)
COMP(1986, fsa1,       fsa1a,    0,     fsa1,       msx2jp,   msx2_state, empty_init, "Panasonic / Matsushita", "FS-A1 / 1st released version (Japan) (MSX2)", 0)
COMP(1986, fsa1a,      0,        0,     fsa1a,      msx2jp,   msx2_state, empty_init, "Panasonic / Matsushita", "FS-A1 / 2nd released version (Japan) (MSX2)", 0)
COMP(1987, fsa1mk2,    0,        0,     fsa1mk2,    msx2jp,   msx2_state, empty_init, "Panasonic / Matsushita", "FS-A1MK2 (Japan) (MSX2)", 0)
COMP(1987, fsa1f,      0,        0,     fsa1f,      msx2jp,   msx2_state, empty_init, "Panasonic / Matsushita", "FS-A1F (Japan) (MSX2)", 0)
COMP(1987, fsa1fm,     0,        0,     fsa1fm,     msx2jp,   msx2_state, empty_init, "Panasonic / Matsushita", "FS-A1FM (Japan) (MSX2)", 0)
COMP(1986, nms8220,    nms8220a, 0,     nms8220,    msx2,     msx2_state, empty_init, "Philips", "NMS-8220 (12-jun-1986) (MSX2)", 0)
COMP(1986, nms8220a,   0,        0,     nms8220a,   msx2,     msx2_state, empty_init, "Philips", "NMS-8220 (13-aug-1986) (MSX2)", 0)
COMP(1986, vg8230,     0,        0,     vg8230,     msx2,     msx2_state, empty_init, "Philips", "VG-8230 (MSX2)", 0)
COMP(19??, vg8230j,    vg8230,   0,     vg8230j,    msx2jp,   msx2_state, empty_init, "Philips", "VG-8230J (MSX2)", MACHINE_NOT_WORKING) // Screen flashes a few times before going into basic
COMP(1986, vg8235,     0,        0,     vg8235,     msx2,     msx2_state, empty_init, "Philips", "VG-8235 (MSX2)", 0)
COMP(1986, vg8235f,    vg8235,   0,     vg8235f,    msx2,     msx2_state, empty_init, "Philips", "VG-8235F (MSX2)", 0)
COMP(1986, vg8240,     0,        0,     vg8240,     msx2,     msx2_state, empty_init, "Philips", "VG-8240 (MSX2)", 0)
COMP(1986, nms8245,    0,        0,     nms8245,    msx2,     msx2_state, empty_init, "Philips", "NMS-8245 (MSX2)", 0)
COMP(1986, nms8245f,   nms8245,  0,     nms8245f,   msx2,     msx2_state, empty_init, "Philips", "NMS-8245F (MSX2)", 0)
COMP(1986, nms8250,    nms8255,  0,     nms8250,    msx2,     msx2_state, empty_init, "Philips", "NMS-8250 (MSX2)", 0)
COMP(1986, nms8250f,   nms8255,  0,     nms8250f,   msx2,     msx2_state, empty_init, "Philips", "NMS-8250F (MSX2)", 0) // French keyboard
COMP(19??, nms8250j,   nms8255,  0,     nms8250j,   msx2jp,   msx2_state, empty_init, "Philips", "NMS-8250J (MSX2)", 0)
COMP(1986, nms8255,    0,        0,     nms8255,    msx2,     msx2_state, empty_init, "Philips", "NMS-8255 (MSX2)", 0)
COMP(1986, nms8255f,   nms8255,  0,     nms8255f,   msx2,     msx2_state, empty_init, "Philips", "NMS-8255F (MSX2)", 0) // French keyboard
COMP(1986, nms8260,    0,        0,     nms8260,    msx2,     msx2_state, empty_init, "Philips", "NMS-8260 (Prototype) (MSX2)", MACHINE_NOT_WORKING)
COMP(1986, nms8280,    0,        0,     nms8280,    msx2,     msx2_state, empty_init, "Philips", "NMS-8280 (MSX2)", 0)
COMP(1986, nms8280f,   nms8280,  0,     nms8280f,   msx2,     msx2_state, empty_init, "Philips", "NMS-8280F (MSX2)", 0) // French keyboard
COMP(1986, nms8280g,   nms8280,  0,     nms8280g,   msx2,     msx2_state, empty_init, "Philips", "NMS-8280G (MSX2)", 0)
COMP(19??, mpc2300,    0,        0,     mpc2300,    msx2,     msx2_state, empty_init, "Sanyo", "MPC-2300 (MSX2)", MACHINE_NOT_WORKING) // Keyboard responds differently
COMP(198?, mpc2500f,   0,        0,     mpc2500f,   msx2,     msx2_state, empty_init, "Sanyo", "MPC-2500FD (MSX2)", MACHINE_NOT_WORKING) // Russian keyboard?
COMP(19??, mpc25fd,    0,        0,     mpc25fd,    msx2,     msx2_state, empty_init, "Sanyo", "Wavy MPC-25FD (MSX2)", 0)
COMP(198?, mpc27,      0,        0,     mpc27,      msx2jp,   msx2_state, empty_init, "Sanyo", "Wavy MPC-27 (MSX2)", MACHINE_NOT_WORKING) // Light pen not emulated
COMP(1988, phc23,      0,        0,     phc23,      msx2jp,   msx2_state, empty_init, "Sanyo", "Wavy PHC-23 (Japan) (MSX2)", 0)
COMP(198?, phc55fd2,   0,        0,     phc55fd2,   msx2jp,   msx2_state, empty_init, "Sanyo", "Wavy PHC-55FD2 (MSX2)", 0)
COMP(198?, phc77,      0,        0,     phc77,      msx2jp,   msx2_state, empty_init, "Sanyo", "Wavy PHC-77 (MSX2)", MACHINE_NOT_WORKING) // Firmware not emulated
COMP(1986, hbf1,       0,        0,     hbf1,       msx2jp,   msx2_state, empty_init, "Sony", "HB-F1 (Japan) (MSX2)", MACHINE_NOT_WORKING ) // Screen stays a single color after a while
COMP(1987, hbf12,      0,        0,     hbf12,      msx2jp,   msx2_state, empty_init, "Sony", "HB-F1II (Japan) (MSX2)", MACHINE_NOT_WORKING ) // Screen stays a single color after a while
COMP(1987, hbf1xd,     0,        0,     hbf1xd,     msx2jp,   msx2_state, empty_init, "Sony", "HB-F1XD (Japan) (MSX2)", 0)
COMP(1988, hbf1xdm2,   0,        0,     hbf1xdm2,   msx2jp,   msx2_state, empty_init, "Sony", "HB-F1XDMK2 (Japan) (MSX2)", 0)
COMP(19??, hbf5,       0,        0,     hbf5,       msx2,     msx2_state, empty_init, "Sony", "HB-F5 (MSX2)", 0)
COMP(1985, hbf9p,      0,        0,     hbf9p,      msx2,     msx2_state, empty_init, "Sony", "HB-F9P (MSX2)", 0)
COMP(19??, hbf9pr,     hbf9p,    0,     hbf9pr,     msx2,     msx2_state, empty_init, "Sony", "HB-F9P Russian (MSX2)", MACHINE_NOT_WORKING) // Keyboard responds differently
COMP(1985, hbf9s,      hbf9p,    0,     hbf9s,      msx2,     msx2_state, empty_init, "Sony", "HB-F9S (MSX2)", 0)
COMP(1986, hbf500,     hbf500p,  0,     hbf500,     msx2jp,   msx2_state, empty_init, "Sony", "HB-F500 (Japan) (MSX2)", 0)
COMP(198?, hbf500f,    hbf500p,  0,     hbf500f,    msx2,     msx2_state, empty_init, "Sony", "HB-F500F (MSX2)", 0) // French keyboard?
COMP(1985, hbf500p,    0,        0,     hbf500p,    msx2,     msx2_state, empty_init, "Sony", "HB-F500P (MSX2)", 0)
COMP(1985, hbf700d,    hbf700p,  0,     hbf700d,    msx2,     msx2_state, empty_init, "Sony", "HB-F700D (Germany) (MSX2)", 0)
COMP(1985, hbf700f,    hbf700p,  0,     hbf700f,    msx2,     msx2_state, empty_init, "Sony", "HB-F700F (MSX2)", 0)
COMP(1985, hbf700p,    0,        0,     hbf700p,    msx2,     msx2_state, empty_init, "Sony", "HB-F700P (MSX2)", 0)
COMP(1985, hbf700s,    hbf700p,  0,     hbf700s,    msx2,     msx2_state, empty_init, "Sony", "HB-F700S (Spain) (MSX2)", 0)
COMP(1986, hbf900,     hbf900a,  0,     hbf900,     msx2jp,   msx2_state, empty_init, "Sony", "HB-F900 / 1st released version (Japan) (MSX2)", 0)
COMP(1986, hbf900a,    0,        0,     hbf900a,    msx2jp,   msx2_state, empty_init, "Sony", "HB-F900 / 2nd released version (Japan) (MSX2)", 0)
COMP(1986, hbg900ap,   hbg900p,  0,     hbg900ap,   msx2,     msx2_state, empty_init, "Sony", "HB-G900AP (MSX2)", 0 )
COMP(1986, hbg900p,    0,        0,     hbg900p,    msx2,     msx2_state, empty_init, "Sony", "HB-G900P (MSX2)", 0 )
COMP(1986, hotbit20,   0,        0,     hotbit20,   msx2,     msx2_state, empty_init, "Sharp / Epcom", "HB-8000 Hotbit 2.0 (MSX2)", 0) // Black screen
COMP(1986, tpc310,     0,        0,     tpc310,     msx2,     msx2_state, empty_init, "Talent", "TPC-310 (MSX2)", 0)
COMP(19??, tpp311,     0,        0,     tpp311,     msx2,     msx2_state, empty_init, "Talent", "TPP-311 (MSX2)", 0)
COMP(19??, tps312,     0,        0,     tps312,     msx2,     msx2_state, empty_init, "Talent", "TPS-312 (MSX2)", 0)
COMP(1986, hx23,       hx23i,    0,     hx23,       msx2,     msx2_state, empty_init, "Toshiba", "HX-23 (MSX2)", 0)
COMP(1986, hx23f,      hx23i,    0,     hx23f,      msx2,     msx2_state, empty_init, "Toshiba", "HX-23F (MSX2)", 0)
COMP(19??, hx23i,      0,        0,     hx23i,      msx2,     msx2_state, empty_init, "Toshiba", "HX-23I (MSX2)", 0)
COMP(1985, hx33,       0,        0,     hx33,       msx2jp,   msx2_state, empty_init, "Toshiba", "HX-33 (MSX2)", 0)
COMP(1985, hx34,       hx34i,    0,     hx34,       msx2jp,   msx2_state, empty_init, "Toshiba", "HX-34 (MSX2)", 0)
COMP(1985, hx34i,      0,        0,     hx34i,      msx,      msx2_state, empty_init, "Toshiba", "HX-34I (MSX2)", 0)
COMP(1985, fstm1,      0,        0,     fstm1,      msx,      msx2_state, empty_init, "Toshiba", "FS-TM1 (MSX2)", 0)
COMP(198?, victhc90,   victhc95, 0,     victhc90,   msxjp,    msx2_state, empty_init, "Victor", "HC-90 (MSX2)", MACHINE_NOT_WORKING) // 2nd cpu/turbo not emulated, firmware won't start
COMP(1986, victhc95,   0,        0,     victhc95,   msxjp,    msx2_state, empty_init, "Victor", "HC-95 (MSX2)", MACHINE_NOT_WORKING) // 2nd cpu/turbo not emulated, firmware won't start
COMP(1986, victhc95a,  victhc95, 0,     victhc95a,  msxjp,    msx2_state, empty_init, "Victor", "HC-95A (MSX2)", MACHINE_NOT_WORKING) // 2nd cpu/turbo not emulated, firmware won't start
COMP(1986, cx7m,       cx7m128,  0,     cx7m,       msx2,     msx2_state, empty_init, "Yamaha", "CX7M (MSX2)", 0)
COMP(1986, cx7m128,    0,        0,     cx7m128,    msx2,     msx2_state, empty_init, "Yamaha", "CX7M/128 (MSX2)", 0)
COMP(198?, y503iiir,   0,        0,     y503iiir,   msx2,     msx2_state, empty_init, "Yamaha", "YIS-503 III R (Russian) (MSX2)", MACHINE_NOT_WORKING) // Russian keyboard, floppy support broken
COMP(198?, y503iiire,  y503iiir, 0,     y503iiire,  msx2,     msx2_state, empty_init, "Yamaha", "YIS-503 III R (Estonian) (MSX2)", MACHINE_NOT_WORKING) // Russian/Estonian keyboard, floppy support broken
COMP(1985, yis60464,   yis604,   0,     yis60464,   msx2jp,   msx2_state, empty_init, "Yamaha", "YIS604 (64KB) (MSX2)", 0)
COMP(1985, yis604,     0,        0,     yis604,     msx2jp,   msx2_state, empty_init, "Yamaha", "YIS604 (128KB) (MSX2)", 0)
COMP(198?, y805128,    y805256,  0,     y805128,    msx2jp,   msx2_state, empty_init, "Yamaha", "YIS805/128 (Russian) (MSX2)", MACHINE_NOT_WORKING) // Floppy support broken
COMP(198?, y805128r2,  y805256,  0,     y805128r2,  msx2jp,   msx2_state, empty_init, "Yamaha", "YIS805R2/128 (Russian) (MSX2)", MACHINE_NOT_WORKING) // Floppy support broken
COMP(198?, y805128r2e, y805256,  0,     y805128r2e, msx2jp,   msx2_state, empty_init, "Yamaha", "YIS805R2/128 (Estonian) (MSX2)", MACHINE_NOT_WORKING) // Floppy support broken
COMP(198?, y805256,    0,        0,     y805256,    msx2jp,   msx2_state, empty_init, "Yamaha", "YIS805/256 (Russian) (MSX2)", MACHINE_NOT_WORKING) // Floppy support broken

/* MSX2+ */
COMP(19??, expert3i,   0,        0,     expert3i,   msx2,     msx2_state, empty_init, "Ciel", "Expert 3 IDE (MSX2+)", MACHINE_NOT_WORKING ) // Some hardware not emulated
COMP(1996, expert3t,   0,        0,     expert3t,   msx2,     msx2_state, empty_init, "Ciel", "Expert 3 Turbo (MSX2+)", MACHINE_NOT_WORKING ) // Some hardware not emulated
COMP(19??, expertac,   0,        0,     expertac,   msx2,     msx2_state, empty_init, "Gradiente", "Expert AC88+ (MSX2+)", MACHINE_NOT_WORKING ) // Some hardware not emulated
COMP(19??, expertdx,   0,        0,     expertdx,   msx2,     msx2_state, empty_init, "Gradiente", "Expert DDX+ (MSX2+)", MACHINE_NOT_WORKING ) // Some hardware not emulated
COMP(1988, fsa1fx,     0,        0,     fsa1fx,     msx2jp,   msx2_state, empty_init, "Panasonic / Matsushita", "FS-A1FX (Japan) (MSX2+)", 0 )
COMP(1988, fsa1wx,     fsa1wxa,  0,     fsa1wx,     msx2jp,   msx2_state, empty_init, "Panasonic / Matsushita", "FS-A1WX / 1st released version (Japan) (MSX2+)", 0 )
COMP(1988, fsa1wxa,    0,        0,     fsa1wxa,    msx2jp,   msx2_state, empty_init, "Panasonic / Matsushita", "FS-A1WX / 2nd released version (Japan) (MSX2+)", 0 )
COMP(1989, fsa1wsx,    0,        0,     fsa1wsx,    msx2jp,   msx2_state, empty_init, "Panasonic / Matsushita", "FS-A1WSX (Japan) (MSX2+)", 0 )
COMP(1988, hbf1xdj,    0,        0,     hbf1xdj,    msx2jp,   msx2_state, empty_init, "Sony", "HB-F1XDJ (Japan) (MSX2+)", 0 )
COMP(1989, hbf1xv,     0,        0,     hbf1xv,     msx2jp,   msx2_state, empty_init, "Sony", "HB-F1XV (Japan) (MSX2+)", 0 )
COMP(1988, phc70fd,    phc70fd2, 0,     phc70fd,    msx2jp,   msx2_state, empty_init, "Sanyo", "WAVY PHC-70FD (Japan) (MSX2+)", 0 )
COMP(1988, phc70fd2,   0,        0,     phc70fd2,   msx2jp,   msx2_state, empty_init, "Sanyo", "WAVY PHC-70FD2 (Japan) (MSX2+)", 0 )
COMP(1989, phc35j,     0,        0,     phc35j,     msx2jp,   msx2_state, empty_init, "Sanyo", "WAVY PHC-35J (Japan) (MSX2+)", 0)
COMP(19??, hbf9sp,     0,        0,     hbf9sp,     msx2jp,   msx2_state, empty_init, "Sony", "HB-F9S+ (MSX2+)", 0)

/* MSX Turbo-R */
/* Temporary placeholders, Turbo-R hardware is not supported yet */
COMP(19??, fsa1gt,     0,        0,     fsa1gt,     msx2jp,   msx2_state, empty_init, "Panasonic", "FS-A1GT (MSX Turbo-R)", MACHINE_NOT_WORKING)
COMP(19??, fsa1st,     0,        0,     fsa1st,     msx2jp,   msx2_state, empty_init, "Panasonic", "FS-A1ST (MSX Turbo-R)", MACHINE_NOT_WORKING)
