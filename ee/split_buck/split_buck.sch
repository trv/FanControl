EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:joystick
LIBS:stm32
LIBS:w_connectors
LIBS:split_buck-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Q_PMOS_GSD Q1
U 1 1 5AA40C0A
P 5100 3150
F 0 "Q1" H 5300 3200 50  0000 L CNN
F 1 "DMP2160U" H 5300 3100 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 5300 3250 50  0001 C CNN
F 3 "" H 5100 3150 50  0001 C CNN
	1    5100 3150
	1    0    0    1   
$EndComp
$Comp
L C C2
U 1 1 5AA40C84
P 4600 3400
F 0 "C2" H 4625 3500 50  0000 L CNN
F 1 ".047u" H 4625 3300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4638 3250 50  0001 C CNN
F 3 "" H 4600 3400 50  0001 C CNN
	1    4600 3400
	1    0    0    -1  
$EndComp
$Comp
L L L1
U 1 1 5AA40CDB
P 5500 3400
F 0 "L1" V 5450 3400 50  0000 C CNN
F 1 "100u" V 5575 3400 50  0000 C CNN
F 2 "Inductors_THT:L_Axial_L7.0mm_D3.3mm_P10.16mm_Horizontal_Fastron_MICC" H 5500 3400 50  0001 C CNN
F 3 "" H 5500 3400 50  0001 C CNN
	1    5500 3400
	0    1    -1   0   
$EndComp
$Comp
L D_Schottky D2
U 1 1 5AA40DD5
P 5200 3600
F 0 "D2" H 5200 3700 50  0000 C CNN
F 1 "D_Schottky" H 5200 3500 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 5200 3600 50  0001 C CNN
F 3 "" H 5200 3600 50  0001 C CNN
	1    5200 3600
	0    1    1    0   
$EndComp
$Comp
L D_Schottky D3
U 1 1 5AA40E20
P 5950 2950
F 0 "D3" H 5950 3050 50  0000 C CNN
F 1 "D_Schottky" H 5950 2850 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 5950 2950 50  0001 C CNN
F 3 "" H 5950 2950 50  0001 C CNN
	1    5950 2950
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 5AA40E6B
P 4850 2950
F 0 "R1" V 4930 2950 50  0000 C CNN
F 1 "100k" V 4850 2950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4780 2950 50  0001 C CNN
F 3 "" H 4850 2950 50  0001 C CNN
	1    4850 2950
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5AA40F47
P 4200 2950
F 0 "C1" H 4225 3050 50  0000 L CNN
F 1 "10u" H 4225 2850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4238 2800 50  0001 C CNN
F 3 "" H 4200 2950 50  0001 C CNN
	1    4200 2950
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5AA40FBD
P 5700 3600
F 0 "C3" H 5725 3700 50  0000 L CNN
F 1 "10u" H 5725 3500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5738 3450 50  0001 C CNN
F 3 "" H 5700 3600 50  0001 C CNN
	1    5700 3600
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J3
U 1 1 5AA4101F
P 6750 3150
F 0 "J3" H 6750 3350 50  0000 C CNN
F 1 "Conn_01x03" H 6750 2950 50  0000 C CNN
F 2 "Connectors:Fan_Pin_Header_Straight_1x03" H 6750 3150 50  0001 C CNN
F 3 "" H 6750 3150 50  0001 C CNN
	1    6750 3150
	1    0    0    1   
$EndComp
$Comp
L +12V #PWR01
U 1 1 5AA41209
P 4100 2750
F 0 "#PWR01" H 4100 2600 50  0001 C CNN
F 1 "+12V" H 4100 2890 50  0000 C CNN
F 2 "" H 4100 2750 50  0001 C CNN
F 3 "" H 4100 2750 50  0001 C CNN
	1    4100 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5AA41235
P 5200 3850
F 0 "#PWR02" H 5200 3600 50  0001 C CNN
F 1 "GND" H 5200 3700 50  0000 C CNN
F 2 "" H 5200 3850 50  0001 C CNN
F 3 "" H 5200 3850 50  0001 C CNN
	1    5200 3850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 5AA416CE
P 4200 3100
F 0 "#PWR03" H 4200 2850 50  0001 C CNN
F 1 "GND" H 4200 2950 50  0000 C CNN
F 2 "" H 4200 3100 50  0001 C CNN
F 3 "" H 4200 3100 50  0001 C CNN
	1    4200 3100
	1    0    0    -1  
$EndComp
Text Label 3400 3600 0    60   ~ 0
FAN_CTRL
Text Label 7350 2850 2    60   ~ 0
FAN_RPM
$Comp
L GND #PWR04
U 1 1 5AA426F0
P 6500 3300
F 0 "#PWR04" H 6500 3050 50  0001 C CNN
F 1 "GND" H 6500 3150 50  0000 C CNN
F 2 "" H 6500 3300 50  0001 C CNN
F 3 "" H 6500 3300 50  0001 C CNN
	1    6500 3300
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5AA42769
P 6250 3350
F 0 "R2" V 6330 3350 50  0000 C CNN
F 1 "9.1k" V 6250 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6180 3350 50  0001 C CNN
F 3 "" H 6250 3350 50  0001 C CNN
	1    6250 3350
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5AA427BE
P 6250 3750
F 0 "R3" V 6330 3750 50  0000 C CNN
F 1 "1.3k" V 6250 3750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6180 3750 50  0001 C CNN
F 3 "" H 6250 3750 50  0001 C CNN
	1    6250 3750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5AA42A5B
P 6250 3900
F 0 "#PWR05" H 6250 3650 50  0001 C CNN
F 1 "GND" H 6250 3750 50  0000 C CNN
F 2 "" H 6250 3900 50  0001 C CNN
F 3 "" H 6250 3900 50  0001 C CNN
	1    6250 3900
	1    0    0    -1  
$EndComp
Text Label 7350 3550 2    60   ~ 0
FAN_ADC
Connection ~ 4600 2750
Connection ~ 4850 2750
Wire Wire Line
	4600 3150 4900 3150
Wire Wire Line
	4600 3100 4600 3250
Connection ~ 4850 3150
Connection ~ 5200 3800
Connection ~ 4200 2750
Wire Wire Line
	4100 2750 5950 2750
Connection ~ 5200 2750
Wire Wire Line
	6450 3050 6550 3050
Wire Wire Line
	6450 2850 7350 2850
Wire Wire Line
	6500 3300 6500 3250
Wire Wire Line
	6500 3250 6550 3250
Connection ~ 4600 3150
Wire Wire Line
	4200 2750 4200 2800
Wire Wire Line
	4850 3100 4850 3150
Wire Wire Line
	4600 2800 4600 2750
Wire Wire Line
	4850 2800 4850 2750
Wire Wire Line
	5200 2950 5200 2750
Wire Wire Line
	5200 3350 5200 3450
Wire Wire Line
	5200 3400 5350 3400
Connection ~ 5200 3400
Wire Wire Line
	5650 3400 5950 3400
Wire Wire Line
	5200 3750 5200 3850
Wire Wire Line
	5200 3800 5950 3800
Wire Wire Line
	6250 3200 6250 3150
Connection ~ 6250 3150
Wire Wire Line
	6250 3500 6250 3600
Connection ~ 6250 3550
Wire Wire Line
	4600 3600 4600 3550
$Comp
L GND #PWR06
U 1 1 5AA67D59
P 1550 3250
F 0 "#PWR06" H 1550 3000 50  0001 C CNN
F 1 "GND" H 1550 3100 50  0000 C CNN
F 2 "" H 1550 3250 50  0001 C CNN
F 3 "" H 1550 3250 50  0001 C CNN
	1    1550 3250
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR07
U 1 1 5AA67EEC
P 2000 2950
F 0 "#PWR07" H 2000 2800 50  0001 C CNN
F 1 "+12V" H 2000 3090 50  0000 C CNN
F 2 "" H 2000 2950 50  0001 C CNN
F 3 "" H 2000 2950 50  0001 C CNN
	1    2000 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 2950 2000 2950
Wire Wire Line
	1450 3050 1550 3050
Wire Wire Line
	1550 3050 1550 3250
Wire Wire Line
	1550 3150 1450 3150
Connection ~ 1550 3150
$Comp
L GND #PWR08
U 1 1 5AA81CAD
P 1750 2750
F 0 "#PWR08" H 1750 2500 50  0001 C CNN
F 1 "GND" H 1750 2600 50  0000 C CNN
F 2 "" H 1750 2750 50  0001 C CNN
F 3 "" H 1750 2750 50  0001 C CNN
	1    1750 2750
	1    0    0    -1  
$EndComp
Text Label 1650 2450 0    60   ~ 0
FAN_CTRL
Text Label 1650 2550 0    60   ~ 0
FAN_RPM
Text Label 1650 2650 0    60   ~ 0
FAN_ADC
Wire Wire Line
	1450 2650 2100 2650
Wire Wire Line
	1450 2550 2100 2550
Wire Wire Line
	1450 2450 2100 2450
Wire Wire Line
	6450 2850 6450 3050
Wire Wire Line
	4600 3600 3400 3600
$Comp
L Conn_01x08 J1
U 1 1 5AA82FE8
P 1250 2850
F 0 "J1" H 1250 3250 50  0000 C CNN
F 1 "Conn_01x08" H 1250 2350 50  0000 C CNN
F 2 "custom:Fan_Controller_all" H 1250 2850 50  0001 C CNN
F 3 "" H 1250 2850 50  0001 C CNN
	1    1250 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	1450 2750 1750 2750
Wire Wire Line
	1450 2850 1550 2850
Wire Wire Line
	1550 2850 1550 2950
Connection ~ 1550 2950
$Comp
L D_Zener D1
U 1 1 5AAEA835
P 4600 2950
F 0 "D1" H 4600 3050 50  0000 C CNN
F 1 "D_Zener" H 4600 2850 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 4600 2950 50  0001 C CNN
F 3 "" H 4600 2950 50  0001 C CNN
	1    4600 2950
	0    1    1    0   
$EndComp
Text Label 6350 3150 0    30   ~ 0
FAN_VOUT
Text Label 4650 3150 0    30   ~ 0
GATE
Text Label 5250 3400 0    30   ~ 0
IND_SW
$Comp
L C C4
U 1 1 5AAF1587
P 5950 3600
F 0 "C4" H 5975 3700 50  0000 L CNN
F 1 "100u" H 5975 3500 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.00mm" H 5988 3450 50  0001 C CNN
F 3 "" H 5950 3600 50  0001 C CNN
	1    5950 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3450 5700 3400
Connection ~ 5700 3400
Wire Wire Line
	5700 3750 5700 3800
Connection ~ 5700 3800
Wire Wire Line
	5950 3800 5950 3750
Wire Wire Line
	5950 3100 5950 3450
Connection ~ 5950 3400
Wire Wire Line
	5950 3150 6550 3150
Connection ~ 5950 3150
Wire Wire Line
	5950 2750 5950 2800
Wire Wire Line
	6250 3550 7350 3550
$EndSCHEMATC
