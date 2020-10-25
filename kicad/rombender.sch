EESchema Schematic File Version 4
EELAYER 30 0
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
L Device:C_Small C1
U 1 1 5E4B638E
P 950 1400
F 0 "C1" V 698 1400 50  0000 C CNN
F 1 "22pF" V 789 1400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 988 1250 50  0001 C CNN
F 3 "~" H 950 1400 50  0001 C CNN
	1    950  1400
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5E4B6A4E
P 950 1700
F 0 "C2" V 1110 1700 50  0000 C CNN
F 1 "22pF" V 1201 1700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 988 1550 50  0001 C CNN
F 3 "~" H 950 1700 50  0001 C CNN
	1    950  1700
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5E4B75FF
P 1700 850
F 0 "R1" H 1770 896 50  0000 L CNN
F 1 "10k" H 1770 805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1630 850 50  0001 C CNN
F 3 "~" H 1700 850 50  0001 C CNN
	1    1700 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 750  2100 750 
$Comp
L power:VCC #PWR0102
U 1 1 5E4B8265
P 2100 750
F 0 "#PWR0102" H 2100 600 50  0001 C CNN
F 1 "VCC" H 2117 923 50  0000 C CNN
F 2 "" H 2100 750 50  0001 C CNN
F 3 "" H 2100 750 50  0001 C CNN
	1    2100 750 
	1    0    0    -1  
$EndComp
Connection ~ 2100 750 
Wire Wire Line
	2100 750  2400 750 
$Comp
L rombender-rescue:AVR-ISP-6-Connector J3
U 1 1 5E4B60F2
P 5300 1250
F 0 "J3" H 5021 1346 50  0000 R CNN
F 1 "AVR-ISP-6" H 5021 1255 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" V 5050 1300 50  0001 C CNN
F 3 " ~" H 4025 700 50  0001 C CNN
	1    5300 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5E4B78F9
P 5200 1650
F 0 "#PWR0104" H 5200 1400 50  0001 C CNN
F 1 "GND" H 5205 1477 50  0000 C CNN
F 2 "" H 5200 1650 50  0001 C CNN
F 3 "" H 5200 1650 50  0001 C CNN
	1    5200 1650
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0106
U 1 1 5E4B7F6C
P 5200 750
F 0 "#PWR0106" H 5200 600 50  0001 C CNN
F 1 "VCC" H 5217 923 50  0000 C CNN
F 2 "" H 5200 750 50  0001 C CNN
F 3 "" H 5200 750 50  0001 C CNN
	1    5200 750 
	1    0    0    -1  
$EndComp
Text Label 5700 1350 0    50   ~ 0
~RESET
Text Label 5700 1250 0    50   ~ 0
SCK
$Comp
L power:VCC #PWR02
U 1 1 5E58FD82
P 7300 3650
F 0 "#PWR02" H 7300 3500 50  0001 C CNN
F 1 "VCC" H 7317 3823 50  0000 C CNN
F 2 "" H 7300 3650 50  0001 C CNN
F 3 "" H 7300 3650 50  0001 C CNN
	1    7300 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5E590692
P 7800 5850
F 0 "#PWR03" H 7800 5600 50  0001 C CNN
F 1 "GND" H 7805 5677 50  0000 C CNN
F 2 "" H 7800 5850 50  0001 C CNN
F 3 "" H 7800 5850 50  0001 C CNN
	1    7800 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 5850 7800 5850
Wire Notes Line
	7300 3400 11200 3400
Wire Notes Line
	11200 3400 11200 500 
Wire Notes Line
	11200 500  7300 500 
Wire Notes Line
	7300 500  7300 3400
Text Notes 7350 600  0    50   ~ 0
SD Interface
Wire Wire Line
	10600 3000 10600 3100
Connection ~ 10100 3100
Wire Wire Line
	9550 3100 10100 3100
Wire Wire Line
	9550 3000 9550 3100
Connection ~ 9550 2700
$Comp
L Device:C C3
U 1 1 5E56C484
P 9550 2850
F 0 "C3" H 9436 2896 50  0000 R CNN
F 1 "10uF" H 9436 2805 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9588 2700 50  0001 C CNN
F 3 "~" H 9550 2850 50  0001 C CNN
	1    9550 2850
	1    0    0    -1  
$EndComp
Connection ~ 10600 2700
$Comp
L Device:C C4
U 1 1 5E56BB95
P 10600 2850
F 0 "C4" H 10715 2896 50  0000 L CNN
F 1 "0.1uF" H 10715 2805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10638 2700 50  0001 C CNN
F 3 "~" H 10600 2850 50  0001 C CNN
	1    10600 2850
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:AP2112K-3.3 U2
U 1 1 5E4C0B92
P 10100 2800
F 0 "U2" H 10100 3142 50  0000 C CNN
F 1 "AP2112K-3.3" H 10100 3051 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 10100 3125 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/AP2112.pdf" H 10100 2900 50  0001 C CNN
	1    10100 2800
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0111
U 1 1 5E4CDC82
P 10600 2550
F 0 "#PWR0111" H 10600 2400 50  0001 C CNN
F 1 "+3V3" H 10615 2723 50  0000 C CNN
F 2 "" H 10600 2550 50  0001 C CNN
F 3 "" H 10600 2550 50  0001 C CNN
	1    10600 2550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0112
U 1 1 5E4CE40D
P 9550 2550
F 0 "#PWR0112" H 9550 2400 50  0001 C CNN
F 1 "VCC" H 9567 2723 50  0000 C CNN
F 2 "" H 9550 2550 50  0001 C CNN
F 3 "" H 9550 2550 50  0001 C CNN
	1    9550 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5E4CE94C
P 10100 3150
F 0 "#PWR0113" H 10100 2900 50  0001 C CNN
F 1 "GND" H 10105 2977 50  0000 C CNN
F 2 "" H 10100 3150 50  0001 C CNN
F 3 "" H 10100 3150 50  0001 C CNN
	1    10100 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 3150 10100 3100
Wire Wire Line
	10400 2700 10600 2700
Wire Wire Line
	10600 2700 10600 2550
Wire Wire Line
	9800 2700 9700 2700
Wire Wire Line
	9550 2700 9550 2550
Wire Wire Line
	9800 2800 9700 2800
Wire Wire Line
	9700 2800 9700 2700
Connection ~ 9700 2700
Wire Wire Line
	9700 2700 9550 2700
Text Label 7400 1450 2    50   ~ 0
SCK
Text Label 7400 1350 2    50   ~ 0
MISO
Text Label 7400 1550 2    50   ~ 0
MOSI
$Comp
L power:GND #PWR0110
U 1 1 5E4CBDD1
P 9150 2350
F 0 "#PWR0110" H 9150 2100 50  0001 C CNN
F 1 "GND" H 9155 2177 50  0000 C CNN
F 2 "" H 9150 2350 50  0001 C CNN
F 3 "" H 9150 2350 50  0001 C CNN
	1    9150 2350
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0109
U 1 1 5E4CBA41
P 9150 750
F 0 "#PWR0109" H 9150 600 50  0001 C CNN
F 1 "+3V3" H 9165 923 50  0000 C CNN
F 2 "" H 9150 750 50  0001 C CNN
F 3 "" H 9150 750 50  0001 C CNN
	1    9150 750 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5E4C62BF
P 11100 2050
F 0 "#PWR0108" H 11100 1800 50  0001 C CNN
F 1 "GND" H 11105 1877 50  0000 C CNN
F 2 "" H 11100 2050 50  0001 C CNN
F 3 "" H 11100 2050 50  0001 C CNN
	1    11100 2050
	1    0    0    -1  
$EndComp
NoConn ~ 9400 1850
NoConn ~ 9400 1150
$Comp
L Connector:Micro_SD_Card J6
U 1 1 5E4C3978
P 10300 1450
F 0 "J6" H 10250 2167 50  0000 C CNN
F 1 "Micro_SD_Card" H 10250 2076 50  0000 C CNN
F 2 "Connector_Card:microSD_HC_Hirose_DM3D-SF" H 11450 1750 50  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/693072010801.pdf" H 10300 1450 50  0001 C CNN
F 4 "HR1941CT-ND" H 10300 1450 50  0001 C CNN "Vendor"
	1    10300 1450
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0123
U 1 1 5E5AA714
P 1450 5300
F 0 "#PWR0123" H 1450 5150 50  0001 C CNN
F 1 "VCC" H 1467 5473 50  0000 C CNN
F 2 "" H 1450 5300 50  0001 C CNN
F 3 "" H 1450 5300 50  0001 C CNN
	1    1450 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 5300 1450 5300
$Comp
L power:GND #PWR08
U 1 1 5E839A56
P 1250 7500
F 0 "#PWR08" H 1250 7250 50  0001 C CNN
F 1 "GND" H 1255 7327 50  0000 C CNN
F 2 "" H 1250 7500 50  0001 C CNN
F 3 "" H 1250 7500 50  0001 C CNN
	1    1250 7500
	1    0    0    -1  
$EndComp
NoConn ~ 850  7100
$Comp
L Device:C_Small C6
U 1 1 5E80DC7D
P 2500 750
F 0 "C6" V 2300 750 50  0000 C CNN
F 1 "0.1uF" V 2400 750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2538 600 50  0001 C CNN
F 3 "~" H 2500 750 50  0001 C CNN
	1    2500 750 
	0    1    1    0   
$EndComp
$Comp
L Device:C C9
U 1 1 5E80FFA2
P 8450 3800
F 0 "C9" H 8335 3754 50  0000 R CNN
F 1 "0.1uF" H 8335 3845 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8488 3650 50  0001 C CNN
F 3 "~" H 8450 3800 50  0001 C CNN
	1    8450 3800
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5E8174C3
P 8450 4050
F 0 "#PWR012" H 8450 3800 50  0001 C CNN
F 1 "GND" H 8455 3877 50  0000 C CNN
F 2 "" H 8450 4050 50  0001 C CNN
F 3 "" H 8450 4050 50  0001 C CNN
	1    8450 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 4050 8450 3950
$Comp
L power:GND #PWR09
U 1 1 5E831155
P 2750 750
F 0 "#PWR09" H 2750 500 50  0001 C CNN
F 1 "GND" H 2755 577 50  0000 C CNN
F 2 "" H 2750 750 50  0001 C CNN
F 3 "" H 2750 750 50  0001 C CNN
	1    2750 750 
	1    0    0    -1  
$EndComp
$Comp
L Memory_EPROM:27C256 U8
U 1 1 5E8A81AF
P 1250 6400
F 0 "U8" H 1250 7681 50  0000 C CNN
F 1 "DEVICE" H 1250 7590 50  0000 C CNN
F 2 "Package_DIP:DIP-28_W15.24mm" H 1250 6400 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc0014.pdf" H 1250 6400 50  0001 C CNN
	1    1250 6400
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal_GND24 Y1
U 1 1 5F1824F6
P 1200 1550
F 0 "Y1" V 1154 1794 50  0000 L CNN
F 1 "16MHz" V 1245 1794 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 1200 1550 50  0001 C CNN
F 3 "~" H 1200 1550 50  0001 C CNN
	1    1200 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	1050 1400 1200 1400
Wire Wire Line
	1050 1700 1200 1700
Wire Wire Line
	1400 1550 1400 1850
Wire Wire Line
	2600 750  2750 750 
Wire Wire Line
	2400 950  2400 750 
Connection ~ 2400 750 
Text Label 850  5500 2    50   ~ 0
MEM_A0
Text Label 850  5600 2    50   ~ 0
MEM_A1
Text Label 850  5700 2    50   ~ 0
MEM_A2
Text Label 850  5800 2    50   ~ 0
MEM_A3
Text Label 850  5900 2    50   ~ 0
MEM_A4
Text Label 850  6000 2    50   ~ 0
MEM_A5
Text Label 850  6100 2    50   ~ 0
MEM_A6
Text Label 850  6200 2    50   ~ 0
MEM_A7
Text Label 850  6300 2    50   ~ 0
MEM_A8
Text Label 850  6400 2    50   ~ 0
MEM_A9
Text Label 850  6500 2    50   ~ 0
MEM_A10
Text Label 850  6600 2    50   ~ 0
MEM_A11
Text Label 850  6700 2    50   ~ 0
MEM_A12
Text Label 850  6800 2    50   ~ 0
MEM_A13
Text Label 850  6900 2    50   ~ 0
MEM_A14
Text Label 1650 5500 0    50   ~ 0
MEM_D0
Text Label 1650 5600 0    50   ~ 0
MEM_D1
Text Label 1650 5700 0    50   ~ 0
MEM_D2
Text Label 1650 5800 0    50   ~ 0
MEM_D3
Text Label 1650 5900 0    50   ~ 0
MEM_D4
Text Label 1650 6000 0    50   ~ 0
MEM_D5
Text Label 1650 6100 0    50   ~ 0
MEM_D6
Text Label 1650 6200 0    50   ~ 0
MEM_D7
Text Label 850  7200 2    50   ~ 0
MEM_~CE
Text Label 850  7300 2    50   ~ 0
MEM_~OE
Text Label 7200 3850 2    50   ~ 0
MEM_A0
Text Label 7200 3950 2    50   ~ 0
MEM_A1
Text Label 7200 4050 2    50   ~ 0
MEM_A2
Text Label 7200 4150 2    50   ~ 0
MEM_A3
Text Label 7200 4250 2    50   ~ 0
MEM_A4
Text Label 7200 4350 2    50   ~ 0
MEM_A5
Text Label 7200 4450 2    50   ~ 0
MEM_A6
Text Label 7200 4550 2    50   ~ 0
MEM_A7
Text Label 7200 4650 2    50   ~ 0
MEM_A8
Text Label 7200 4750 2    50   ~ 0
MEM_A9
Text Label 7200 4850 2    50   ~ 0
MEM_A10
Text Label 7200 4950 2    50   ~ 0
MEM_A11
Text Label 7200 5050 2    50   ~ 0
MEM_A12
Text Label 7200 5150 2    50   ~ 0
MEM_A13
Text Label 7200 5250 2    50   ~ 0
MEM_A14
Text Label 8000 3850 0    50   ~ 0
MEM_D0
Text Label 8000 3950 0    50   ~ 0
MEM_D1
Text Label 8000 4050 0    50   ~ 0
MEM_D2
Text Label 8000 4150 0    50   ~ 0
MEM_D3
Text Label 8000 4250 0    50   ~ 0
MEM_D4
Text Label 8000 4350 0    50   ~ 0
MEM_D5
Text Label 8000 4450 0    50   ~ 0
MEM_D6
Text Label 8000 4550 0    50   ~ 0
MEM_D7
Text Label 7200 5450 2    50   ~ 0
MEM_~WE
Text Label 7200 5550 2    50   ~ 0
MEM_~OE
Text Label 7200 5650 2    50   ~ 0
MEM_~CE
Wire Wire Line
	850  1400 850  1550
Wire Wire Line
	1000 1550 850  1550
Connection ~ 850  1550
Wire Wire Line
	850  1550 850  1700
Wire Wire Line
	1400 1850 1100 1850
Wire Wire Line
	850  1850 850  1700
Connection ~ 850  1700
$Comp
L power:GND #PWR0101
U 1 1 5F385C84
P 1100 1850
F 0 "#PWR0101" H 1100 1600 50  0001 C CNN
F 1 "GND" H 1105 1677 50  0000 C CNN
F 2 "" H 1100 1850 50  0001 C CNN
F 3 "" H 1100 1850 50  0001 C CNN
	1    1100 1850
	1    0    0    -1  
$EndComp
Connection ~ 1100 1850
Wire Wire Line
	1100 1850 850  1850
Wire Wire Line
	1200 1400 1800 1400
Wire Wire Line
	1800 1400 1800 1450
Connection ~ 1200 1400
Wire Wire Line
	1800 1650 1800 1700
Wire Wire Line
	1800 1700 1200 1700
Connection ~ 1200 1700
Wire Wire Line
	1800 1250 1700 1250
$Comp
L MCU_Microchip_ATmega:ATmega64A-AU U1
U 1 1 5F1FD5D0
P 2400 2950
F 0 "U1" H 2400 861 50  0000 C CNN
F 1 "ATmega64A-AU" H 2400 770 50  0000 C CNN
F 2 "Package_QFP:TQFP-64_14x14mm_P0.8mm" H 2400 2950 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-8160-8-bit-avr-microcontroller-atmega64a-datasheet.pdf" H 2400 2950 50  0001 C CNN
	1    2400 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 950  2500 950 
Connection ~ 2400 950 
Text Label 1800 3450 2    50   ~ 0
MEM_A4
Text Label 1800 3550 2    50   ~ 0
MEM_A5
Text Label 3000 3650 0    50   ~ 0
MEM_A14
Text Label 1800 3650 2    50   ~ 0
MEM_A6
Text Label 1800 3750 2    50   ~ 0
MEM_A7
Text Label 3000 3050 0    50   ~ 0
MEM_A8
Text Label 3000 3150 0    50   ~ 0
MEM_A9
Text Label 3000 3250 0    50   ~ 0
MEM_A10
Text Label 3000 3350 0    50   ~ 0
MEM_A11
Text Label 3000 3450 0    50   ~ 0
MEM_A12
Text Label 3000 3550 0    50   ~ 0
MEM_A13
Text Label 1800 3050 2    50   ~ 0
MEM_A0
Text Label 1800 3150 2    50   ~ 0
MEM_A1
Text Label 1800 3250 2    50   ~ 0
MEM_A2
Text Label 1800 3350 2    50   ~ 0
MEM_A3
Text Label 3000 2250 0    50   ~ 0
SCK
Text Label 3000 2350 0    50   ~ 0
MOSI
Text Label 3000 2450 0    50   ~ 0
MISO
Text Label 1800 3950 2    50   ~ 0
PDI
Text Label 1800 4050 2    50   ~ 0
PDO
Text Label 3000 2150 0    50   ~ 0
SD_~CS
Text Label 1800 4450 2    50   ~ 0
MEM_~WE
Text Label 1800 4550 2    50   ~ 0
MEM_~OE
Text Label 1800 4650 2    50   ~ 0
MEM_~CE
Text Label 5700 1150 0    50   ~ 0
PDI
Text Label 5700 1050 0    50   ~ 0
PDO
Text Label 3000 1250 0    50   ~ 0
MEM_D0
Text Label 3000 1350 0    50   ~ 0
MEM_D1
Text Label 3000 1450 0    50   ~ 0
MEM_D2
Text Label 3000 1550 0    50   ~ 0
MEM_D3
Text Label 3000 1650 0    50   ~ 0
MEM_D4
Text Label 3000 1750 0    50   ~ 0
MEM_D5
Text Label 3000 1850 0    50   ~ 0
MEM_D6
Text Label 3000 1950 0    50   ~ 0
MEM_D7
Wire Wire Line
	1700 950  1700 1250
Text Label 1700 1100 2    50   ~ 0
~RESET
$Comp
L power:GND #PWR0103
U 1 1 5F21D5B0
P 2400 5250
F 0 "#PWR0103" H 2400 5000 50  0001 C CNN
F 1 "GND" H 2405 5077 50  0000 C CNN
F 2 "" H 2400 5250 50  0001 C CNN
F 3 "" H 2400 5250 50  0001 C CNN
	1    2400 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 5250 2400 4950
Wire Wire Line
	7300 3650 7600 3650
$Comp
L Memory_EEPROM:28C256 U4
U 1 1 5F308CA3
P 7600 4750
F 0 "U4" H 7600 6031 50  0000 C CNN
F 1 "28C256" H 7600 5940 50  0000 C CNN
F 2 "Package_SO:SOIC-28W_7.5x17.9mm_P1.27mm" H 7600 4750 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc0006.pdf" H 7600 4750 50  0001 C CNN
	1    7600 4750
	1    0    0    -1  
$EndComp
Connection ~ 7600 3650
Wire Wire Line
	7600 3650 8450 3650
Wire Wire Line
	9400 1550 8900 1550
Wire Wire Line
	8900 1550 8900 1450
Text Label 7400 1250 2    50   ~ 0
SD_~CS
Wire Wire Line
	8800 1750 9400 1750
Wire Wire Line
	9400 1450 9150 1450
Wire Wire Line
	9150 1450 9150 850 
Wire Wire Line
	8800 1750 8800 1550
Connection ~ 9150 850 
Wire Wire Line
	9150 2350 9150 2250
Connection ~ 9150 2250
Wire Wire Line
	9400 1650 9150 1650
Wire Wire Line
	9150 1650 9150 2250
$Comp
L Device:R R3
U 1 1 5F587E36
P 8450 1150
F 0 "R3" V 8243 1150 50  0000 C CNN
F 1 "10k" V 8334 1150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8380 1150 50  0001 C CNN
F 3 "~" H 8450 1150 50  0001 C CNN
	1    8450 1150
	0    -1   1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5F55509D
P 8850 1050
F 0 "R2" H 8780 1004 50  0000 R CNN
F 1 "10k" H 8780 1095 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8780 1050 50  0001 C CNN
F 3 "~" H 8850 1050 50  0001 C CNN
	1    8850 1050
	1    0    0    1   
$EndComp
NoConn ~ 8200 1950
NoConn ~ 8200 1850
NoConn ~ 8200 1750
NoConn ~ 8200 1650
Wire Wire Line
	8800 1550 8200 1550
Wire Wire Line
	8200 1450 8900 1450
Wire Wire Line
	8200 1350 9400 1350
Wire Wire Line
	7900 850  8850 850 
Connection ~ 7700 850 
$Comp
L power:VCC #PWR0114
U 1 1 5E4D165F
P 7700 850
F 0 "#PWR0114" H 7700 700 50  0001 C CNN
F 1 "VCC" H 7717 1023 50  0000 C CNN
F 2 "" H 7700 850 50  0001 C CNN
F 3 "" H 7700 850 50  0001 C CNN
	1    7700 850 
	1    0    0    -1  
$EndComp
NoConn ~ 7400 1650
NoConn ~ 7400 1750
NoConn ~ 7400 1850
NoConn ~ 7400 1950
Wire Wire Line
	8200 1150 8250 1150
$Comp
L Logic_LevelTranslator:TXS0108EPW U3
U 1 1 5F2086B2
P 7800 1550
F 0 "U3" H 7800 761 50  0000 C CNN
F 1 "TXS0108EPW" H 7800 670 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 7800 800 50  0001 C CNN
F 3 "www.ti.com/lit/ds/symlink/txs0108e.pdf" H 7800 1450 50  0001 C CNN
	1    7800 1550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7800 2250 8650 2250
Wire Wire Line
	8650 1150 8650 2250
Connection ~ 8650 2250
Wire Wire Line
	8650 2250 9150 2250
Wire Wire Line
	8850 900  8850 850 
Connection ~ 8850 850 
Wire Wire Line
	8850 850  9150 850 
Wire Wire Line
	8200 1250 8850 1250
Wire Wire Line
	8850 1200 8850 1250
Connection ~ 8850 1250
Wire Wire Line
	8850 1250 9400 1250
$Comp
L Device:R R4
U 1 1 5F5CFDDB
P 7000 1050
F 0 "R4" H 7070 1096 50  0000 L CNN
F 1 "10k" H 7070 1005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6930 1050 50  0001 C CNN
F 3 "~" H 7000 1050 50  0001 C CNN
	1    7000 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 1250 7000 1250
Wire Wire Line
	7000 850  7700 850 
Wire Wire Line
	8600 1150 8650 1150
Wire Wire Line
	7000 900  7000 850 
Wire Wire Line
	7000 1200 7000 1250
$Comp
L Device:Jumper JP1
U 1 1 5F5F1D6C
P 8550 700
F 0 "JP1" H 8550 964 50  0000 C CNN
F 1 "Jumper" H 8550 873 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 8550 700 50  0001 C CNN
F 3 "~" H 8550 700 50  0001 C CNN
	1    8550 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 700  8250 1150
Connection ~ 8250 1150
Wire Wire Line
	8250 1150 8300 1150
Wire Wire Line
	8850 700  9150 700 
Wire Wire Line
	9150 700  9150 750 
Connection ~ 9150 750 
Wire Wire Line
	9150 750  9150 850 
Wire Wire Line
	10100 3100 10600 3100
Text Notes 2150 3700 1    50   ~ 0
JTAG - PF[4:7]
NoConn ~ 1800 2050
$EndSCHEMATC
