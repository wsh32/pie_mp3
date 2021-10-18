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
L Sensor_Proximity:BPR-105F U1
U 1 1 615B2DDE
P 4450 3700
F 0 "U1" H 4450 4017 50  0000 C CNN
F 1 "TCRT5000" H 4450 3926 50  0000 C CNN
F 2 "wsh:TCRT5000" H 4450 3500 50  0001 C CNN
F 3 "http://www.ystone.com.tw/en/data/goods/IRPT/Photo%20Interrupters-Reflective%20Type.pdf" H 4450 3800 50  0001 C CNN
	1    4450 3700
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR01
U 1 1 615B4B73
P 4050 3100
F 0 "#PWR01" H 4050 2950 50  0001 C CNN
F 1 "VCC" H 4065 3273 50  0000 C CNN
F 2 "" H 4050 3100 50  0001 C CNN
F 3 "" H 4050 3100 50  0001 C CNN
	1    4050 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 615B5E62
P 4050 3300
F 0 "R1" H 4118 3346 50  0000 L CNN
F 1 "200" H 4118 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4050 3300 50  0001 C CNN
F 3 "~" H 4050 3300 50  0001 C CNN
	1    4050 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3100 4050 3200
Wire Wire Line
	4050 3400 4050 3600
Wire Wire Line
	4050 3600 4150 3600
$Comp
L power:GND #PWR02
U 1 1 615B678A
P 4050 3950
F 0 "#PWR02" H 4050 3700 50  0001 C CNN
F 1 "GND" H 4055 3777 50  0000 C CNN
F 2 "" H 4050 3950 50  0001 C CNN
F 3 "" H 4050 3950 50  0001 C CNN
	1    4050 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3950 4050 3800
Wire Wire Line
	4050 3800 4150 3800
$Comp
L Device:R_Small_US R2
U 1 1 615B6D57
P 4850 3300
F 0 "R2" H 4918 3346 50  0000 L CNN
F 1 "10K" H 4918 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4850 3300 50  0001 C CNN
F 3 "~" H 4850 3300 50  0001 C CNN
	1    4850 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 3400 4850 3600
Wire Wire Line
	4850 3600 4750 3600
$Comp
L power:VCC #PWR03
U 1 1 615B7453
P 4850 3100
F 0 "#PWR03" H 4850 2950 50  0001 C CNN
F 1 "VCC" H 4865 3273 50  0000 C CNN
F 2 "" H 4850 3100 50  0001 C CNN
F 3 "" H 4850 3100 50  0001 C CNN
	1    4850 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 3100 4850 3200
$Comp
L power:GND #PWR04
U 1 1 615B7AAF
P 4850 3950
F 0 "#PWR04" H 4850 3700 50  0001 C CNN
F 1 "GND" H 4855 3777 50  0000 C CNN
F 2 "" H 4850 3950 50  0001 C CNN
F 3 "" H 4850 3950 50  0001 C CNN
	1    4850 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 3950 4850 3800
Wire Wire Line
	4850 3800 4750 3800
$Comp
L Sensor_Proximity:BPR-105F U2
U 1 1 615BE5A9
P 5950 3700
F 0 "U2" H 5950 4017 50  0000 C CNN
F 1 "TCRT5000" H 5950 3926 50  0000 C CNN
F 2 "wsh:TCRT5000" H 5950 3500 50  0001 C CNN
F 3 "http://www.ystone.com.tw/en/data/goods/IRPT/Photo%20Interrupters-Reflective%20Type.pdf" H 5950 3800 50  0001 C CNN
	1    5950 3700
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR05
U 1 1 615BE5AF
P 5550 3100
F 0 "#PWR05" H 5550 2950 50  0001 C CNN
F 1 "VCC" H 5565 3273 50  0000 C CNN
F 2 "" H 5550 3100 50  0001 C CNN
F 3 "" H 5550 3100 50  0001 C CNN
	1    5550 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R3
U 1 1 615BE5B5
P 5550 3300
F 0 "R3" H 5618 3346 50  0000 L CNN
F 1 "200" H 5618 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 5550 3300 50  0001 C CNN
F 3 "~" H 5550 3300 50  0001 C CNN
	1    5550 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 3100 5550 3200
Wire Wire Line
	5550 3400 5550 3600
Wire Wire Line
	5550 3600 5650 3600
$Comp
L power:GND #PWR06
U 1 1 615BE5BE
P 5550 3950
F 0 "#PWR06" H 5550 3700 50  0001 C CNN
F 1 "GND" H 5555 3777 50  0000 C CNN
F 2 "" H 5550 3950 50  0001 C CNN
F 3 "" H 5550 3950 50  0001 C CNN
	1    5550 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 3950 5550 3800
Wire Wire Line
	5550 3800 5650 3800
$Comp
L Device:R_Small_US R4
U 1 1 615BE5C6
P 6350 3300
F 0 "R4" H 6418 3346 50  0000 L CNN
F 1 "10K" H 6418 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 6350 3300 50  0001 C CNN
F 3 "~" H 6350 3300 50  0001 C CNN
	1    6350 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 3400 6350 3600
Wire Wire Line
	6350 3600 6250 3600
$Comp
L power:VCC #PWR07
U 1 1 615BE5CE
P 6350 3100
F 0 "#PWR07" H 6350 2950 50  0001 C CNN
F 1 "VCC" H 6365 3273 50  0000 C CNN
F 2 "" H 6350 3100 50  0001 C CNN
F 3 "" H 6350 3100 50  0001 C CNN
	1    6350 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 3100 6350 3200
$Comp
L power:GND #PWR08
U 1 1 615BE5D5
P 6350 3950
F 0 "#PWR08" H 6350 3700 50  0001 C CNN
F 1 "GND" H 6355 3777 50  0000 C CNN
F 2 "" H 6350 3950 50  0001 C CNN
F 3 "" H 6350 3950 50  0001 C CNN
	1    6350 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 3950 6350 3800
Wire Wire Line
	6350 3800 6250 3800
$Comp
L power:VCC #PWR0101
U 1 1 615BE470
P 2650 3400
F 0 "#PWR0101" H 2650 3250 50  0001 C CNN
F 1 "VCC" H 2665 3573 50  0000 C CNN
F 2 "" H 2650 3400 50  0001 C CNN
F 3 "" H 2650 3400 50  0001 C CNN
	1    2650 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 3400 2650 3500
Wire Wire Line
	2650 3500 2450 3500
$Comp
L power:GND #PWR0102
U 1 1 615BEC01
P 2650 4100
F 0 "#PWR0102" H 2650 3850 50  0001 C CNN
F 1 "GND" H 2655 3927 50  0000 C CNN
F 2 "" H 2650 4100 50  0001 C CNN
F 3 "" H 2650 4100 50  0001 C CNN
	1    2650 4100
	1    0    0    -1  
$EndComp
Text Label 4850 3600 0    50   ~ 0
S1
Text Label 6350 3600 0    50   ~ 0
S2
Text Label 2450 3600 0    50   ~ 0
S1
Text Label 2450 3700 0    50   ~ 0
S2
$Comp
L Sensor_Proximity:BPR-105F U?
U 1 1 615D9950
P 7300 3700
F 0 "U?" H 7300 4017 50  0000 C CNN
F 1 "TCRT5000" H 7300 3926 50  0000 C CNN
F 2 "wsh:TCRT5000" H 7300 3500 50  0001 C CNN
F 3 "http://www.ystone.com.tw/en/data/goods/IRPT/Photo%20Interrupters-Reflective%20Type.pdf" H 7300 3800 50  0001 C CNN
	1    7300 3700
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 615D9956
P 6900 3100
F 0 "#PWR?" H 6900 2950 50  0001 C CNN
F 1 "VCC" H 6915 3273 50  0000 C CNN
F 2 "" H 6900 3100 50  0001 C CNN
F 3 "" H 6900 3100 50  0001 C CNN
	1    6900 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 615D995C
P 6900 3300
F 0 "R?" H 6968 3346 50  0000 L CNN
F 1 "200" H 6968 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 6900 3300 50  0001 C CNN
F 3 "~" H 6900 3300 50  0001 C CNN
	1    6900 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 3100 6900 3200
Wire Wire Line
	6900 3400 6900 3600
Wire Wire Line
	6900 3600 7000 3600
$Comp
L power:GND #PWR?
U 1 1 615D9965
P 6900 3950
F 0 "#PWR?" H 6900 3700 50  0001 C CNN
F 1 "GND" H 6905 3777 50  0000 C CNN
F 2 "" H 6900 3950 50  0001 C CNN
F 3 "" H 6900 3950 50  0001 C CNN
	1    6900 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 3950 6900 3800
Wire Wire Line
	6900 3800 7000 3800
$Comp
L Device:R_Small_US R?
U 1 1 615D996D
P 7700 3300
F 0 "R?" H 7768 3346 50  0000 L CNN
F 1 "10K" H 7768 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 7700 3300 50  0001 C CNN
F 3 "~" H 7700 3300 50  0001 C CNN
	1    7700 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 3400 7700 3600
Wire Wire Line
	7700 3600 7600 3600
$Comp
L power:VCC #PWR?
U 1 1 615D9975
P 7700 3100
F 0 "#PWR?" H 7700 2950 50  0001 C CNN
F 1 "VCC" H 7715 3273 50  0000 C CNN
F 2 "" H 7700 3100 50  0001 C CNN
F 3 "" H 7700 3100 50  0001 C CNN
	1    7700 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 3100 7700 3200
$Comp
L power:GND #PWR?
U 1 1 615D997C
P 7700 3950
F 0 "#PWR?" H 7700 3700 50  0001 C CNN
F 1 "GND" H 7705 3777 50  0000 C CNN
F 2 "" H 7700 3950 50  0001 C CNN
F 3 "" H 7700 3950 50  0001 C CNN
	1    7700 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 3950 7700 3800
Wire Wire Line
	7700 3800 7600 3800
Text Label 7700 3600 0    50   ~ 0
S3
$Comp
L Sensor_Proximity:BPR-105F U?
U 1 1 615DD24E
P 8600 3700
F 0 "U?" H 8600 4017 50  0000 C CNN
F 1 "TCRT5000" H 8600 3926 50  0000 C CNN
F 2 "wsh:TCRT5000" H 8600 3500 50  0001 C CNN
F 3 "http://www.ystone.com.tw/en/data/goods/IRPT/Photo%20Interrupters-Reflective%20Type.pdf" H 8600 3800 50  0001 C CNN
	1    8600 3700
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 615DD254
P 8200 3100
F 0 "#PWR?" H 8200 2950 50  0001 C CNN
F 1 "VCC" H 8215 3273 50  0000 C CNN
F 2 "" H 8200 3100 50  0001 C CNN
F 3 "" H 8200 3100 50  0001 C CNN
	1    8200 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 615DD25A
P 8200 3300
F 0 "R?" H 8268 3346 50  0000 L CNN
F 1 "200" H 8268 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 8200 3300 50  0001 C CNN
F 3 "~" H 8200 3300 50  0001 C CNN
	1    8200 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3100 8200 3200
Wire Wire Line
	8200 3400 8200 3600
Wire Wire Line
	8200 3600 8300 3600
$Comp
L power:GND #PWR?
U 1 1 615DD263
P 8200 3950
F 0 "#PWR?" H 8200 3700 50  0001 C CNN
F 1 "GND" H 8205 3777 50  0000 C CNN
F 2 "" H 8200 3950 50  0001 C CNN
F 3 "" H 8200 3950 50  0001 C CNN
	1    8200 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3950 8200 3800
Wire Wire Line
	8200 3800 8300 3800
$Comp
L Device:R_Small_US R?
U 1 1 615DD26B
P 9000 3300
F 0 "R?" H 9068 3346 50  0000 L CNN
F 1 "10K" H 9068 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 9000 3300 50  0001 C CNN
F 3 "~" H 9000 3300 50  0001 C CNN
	1    9000 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 3400 9000 3600
Wire Wire Line
	9000 3600 8900 3600
$Comp
L power:VCC #PWR?
U 1 1 615DD273
P 9000 3100
F 0 "#PWR?" H 9000 2950 50  0001 C CNN
F 1 "VCC" H 9015 3273 50  0000 C CNN
F 2 "" H 9000 3100 50  0001 C CNN
F 3 "" H 9000 3100 50  0001 C CNN
	1    9000 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 3100 9000 3200
$Comp
L power:GND #PWR?
U 1 1 615DD27A
P 9000 3950
F 0 "#PWR?" H 9000 3700 50  0001 C CNN
F 1 "GND" H 9005 3777 50  0000 C CNN
F 2 "" H 9000 3950 50  0001 C CNN
F 3 "" H 9000 3950 50  0001 C CNN
	1    9000 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 3950 9000 3800
Wire Wire Line
	9000 3800 8900 3800
Text Label 9000 3600 0    50   ~ 0
S4
$Comp
L Connector:Conn_01x06_Male J?
U 1 1 615DF18D
P 2250 3700
F 0 "J?" H 2358 4081 50  0000 C CNN
F 1 "Conn" H 2358 3990 50  0000 C CNN
F 2 "" H 2250 3700 50  0001 C CNN
F 3 "~" H 2250 3700 50  0001 C CNN
	1    2250 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 4100 2650 4000
Wire Wire Line
	2650 4000 2450 4000
Text Label 2450 3800 0    50   ~ 0
S3
Text Label 2450 3900 0    50   ~ 0
S4
$EndSCHEMATC
