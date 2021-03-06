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
L Driver_Motor:L293D U1
U 1 1 5F0201D4
P 3050 3075
F 0 "U1" H 3400 4025 50  0000 C CNN
F 1 "L293D" V 3500 2525 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket_LongPads" H 3300 2325 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/l293.pdf" H 2750 3775 50  0001 C CNN
	1    3050 3075
	1    0    0    -1  
$EndComp
$Comp
L OwnConn:GND #PWR02
U 1 1 5F02832D
P 1000 1300
F 0 "#PWR02" H 1000 1050 50  0001 C CNN
F 1 "GND" H 1005 1127 50  0000 C CNN
F 2 "" H 1000 1300 50  0001 C CNN
F 3 "" H 1000 1300 50  0001 C CNN
	1    1000 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+6V #PWR01
U 1 1 5F028D5D
P 1000 950
F 0 "#PWR01" H 1000 800 50  0001 C CNN
F 1 "+6V" H 1015 1123 50  0000 C CNN
F 2 "" H 1000 950 50  0001 C CNN
F 3 "" H 1000 950 50  0001 C CNN
	1    1000 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 950  1000 1075
Wire Notes Line
	9400 2250 9925 2250
Wire Notes Line
	9925 2250 9925 1000
Wire Notes Line
	9925 1000 9400 1000
Wire Notes Line
	9400 1000 9400 2250
Wire Notes Line
	9575 1000 9575 1100
Wire Notes Line
	9575 1100 9750 1100
Wire Notes Line
	9750 1100 9750 1000
Text Notes 9600 1075 0    39   ~ 0
USB
$Comp
L power:+6V #PWR011
U 1 1 5F0322E9
P 10125 1100
F 0 "#PWR011" H 10125 950 50  0001 C CNN
F 1 "+6V" V 10075 1175 50  0000 L CNN
F 2 "" H 10125 1100 50  0001 C CNN
F 3 "" H 10125 1100 50  0001 C CNN
	1    10125 1100
	0    1    1    0   
$EndComp
Text GLabel 9050 1100 0    50   Input ~ 0
Move_complete
Text GLabel 9050 1200 0    50   Input ~ 0
Move_step
Wire Wire Line
	9050 1100 9200 1100
Wire Wire Line
	9050 1200 9200 1200
$Comp
L OwnConn:GND #PWR09
U 1 1 5F04C5FA
P 8975 1400
F 0 "#PWR09" H 8975 1150 50  0001 C CNN
F 1 "GND" H 8825 1350 50  0000 C CNN
F 2 "" H 8975 1400 50  0001 C CNN
F 3 "" H 8975 1400 50  0001 C CNN
	1    8975 1400
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x12_Female J5
U 1 1 5F04D619
P 9400 1600
F 0 "J5" H 9250 2275 50  0000 L CNN
F 1 "Conn_01x12_Female" V 9450 1225 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x12_P2.54mm_Vertical" H 9400 1600 50  0001 C CNN
F 3 "~" H 9400 1600 50  0001 C CNN
	1    9400 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 1300 8975 1300
Wire Wire Line
	8975 1300 8975 1400
Wire Wire Line
	8975 1400 9200 1400
Connection ~ 8975 1400
$Comp
L Connector:Conn_01x12_Female J7
U 1 1 5F050505
P 9925 1600
F 0 "J7" H 9775 2275 50  0000 L CNN
F 1 "Conn_01x12_Female" V 9975 1225 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x12_P2.54mm_Vertical" H 9925 1600 50  0001 C CNN
F 3 "~" H 9925 1600 50  0001 C CNN
	1    9925 1600
	-1   0    0    -1  
$EndComp
$Comp
L OwnConn:GND #PWR012
U 1 1 5F05238A
P 10625 1225
F 0 "#PWR012" H 10625 975 50  0001 C CNN
F 1 "GND" H 10475 1175 50  0000 C CNN
F 2 "" H 10625 1225 50  0001 C CNN
F 3 "" H 10625 1225 50  0001 C CNN
	1    10625 1225
	1    0    0    -1  
$EndComp
Wire Wire Line
	10125 1200 10625 1200
Wire Wire Line
	10625 1200 10625 1225
NoConn ~ 10125 1300
Text GLabel 10175 1400 2    50   Input ~ 0
Vcc
Wire Wire Line
	10125 1400 10175 1400
Wire Wire Line
	10125 1800 10200 1800
Text GLabel 10200 1800 2    50   Input ~ 0
Photo_Sens
NoConn ~ 10125 1500
NoConn ~ 10125 1700
NoConn ~ 10125 1600
NoConn ~ 10125 1900
NoConn ~ 9200 1500
NoConn ~ 9200 1600
Text GLabel 9025 1700 0    50   Input ~ 0
Photo_Powr
Wire Wire Line
	9025 1700 9200 1700
Text GLabel 9025 1800 0    50   Input ~ 0
Motor_EN
Text GLabel 9025 1900 0    50   Input ~ 0
Motor_0
Text GLabel 9025 2000 0    50   Input ~ 0
Motor_1
Wire Wire Line
	9025 1800 9200 1800
Wire Wire Line
	9200 1900 9025 1900
Wire Wire Line
	9025 2000 9200 2000
$Comp
L power:+6V #PWR05
U 1 1 5F06366B
P 3150 1925
F 0 "#PWR05" H 3150 1775 50  0001 C CNN
F 1 "+6V" H 3165 2098 50  0000 C CNN
F 2 "" H 3150 1925 50  0001 C CNN
F 3 "" H 3150 1925 50  0001 C CNN
	1    3150 1925
	1    0    0    -1  
$EndComp
NoConn ~ 2550 3275
NoConn ~ 3550 3275
NoConn ~ 3550 3075
NoConn ~ 2550 3075
Wire Wire Line
	2550 3475 2550 3875
Wire Wire Line
	2550 3875 2850 3875
Wire Wire Line
	2850 3875 2950 3875
Connection ~ 2850 3875
Wire Wire Line
	2950 3875 3050 3875
Connection ~ 2950 3875
Wire Wire Line
	3150 3875 3250 3875
Connection ~ 3150 3875
Wire Wire Line
	3050 3875 3050 3950
Connection ~ 3050 3875
Wire Wire Line
	3050 3875 3150 3875
$Comp
L OwnConn:GND #PWR04
U 1 1 5F065F81
P 3050 3950
F 0 "#PWR04" H 3050 3700 50  0001 C CNN
F 1 "GND" H 3055 3777 50  0000 C CNN
F 2 "" H 3050 3950 50  0001 C CNN
F 3 "" H 3050 3950 50  0001 C CNN
	1    3050 3950
	1    0    0    -1  
$EndComp
Text GLabel 2550 2875 0    50   Input ~ 0
Motor_EN
Text GLabel 2550 2475 0    50   Input ~ 0
Motor_0
Text GLabel 2550 2675 0    50   Input ~ 0
Motor_1
Text GLabel 3550 2475 2    50   Input ~ 0
Motor_0_CON
Text GLabel 3550 2675 2    50   Input ~ 0
Motor_1_CON
$Comp
L Device:C_Small C2
U 1 1 5F06A5AA
P 4300 3625
F 0 "C2" H 4392 3671 50  0000 L CNN
F 1 "0.1uF" H 4392 3580 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4300 3625 50  0001 C CNN
F 3 "~" H 4300 3625 50  0001 C CNN
	1    4300 3625
	1    0    0    -1  
$EndComp
Connection ~ 3250 3875
Wire Wire Line
	3150 2075 3150 2025
Wire Wire Line
	3250 3875 4300 3875
Wire Wire Line
	4300 3875 4300 3725
Wire Wire Line
	4300 3525 4300 2025
Wire Wire Line
	4300 2025 3150 2025
Connection ~ 3150 2025
Wire Wire Line
	3150 2025 3150 1925
$Comp
L Device:C_Small C1
U 1 1 5F071EEF
P 1875 3475
F 0 "C1" H 1967 3521 50  0000 L CNN
F 1 "0.1uF" H 1967 3430 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1875 3475 50  0001 C CNN
F 3 "~" H 1875 3475 50  0001 C CNN
	1    1875 3475
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 3875 1875 3875
Wire Wire Line
	1875 3875 1875 3575
Connection ~ 2550 3875
Wire Wire Line
	1875 3375 1875 2000
Wire Wire Line
	2950 1950 2950 2000
Wire Wire Line
	1875 2000 2950 2000
Connection ~ 2950 2000
Wire Wire Line
	2950 2000 2950 2075
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 5F07BE6E
P 2950 1800
F 0 "JP1" V 3050 2075 50  0000 R CNN
F 1 "SolderJumper_2_Open" V 2925 2700 50  0000 R CNN
F 2 "SolderJumper:2pin" H 2950 1800 50  0001 C CNN
F 3 "~" H 2950 1800 50  0001 C CNN
	1    2950 1800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2950 1550 2950 1650
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 5F080A83
P 1850 5975
F 0 "J1" H 1825 5700 50  0000 C CNN
F 1 "Screw_Terminal_01x02" V 1950 5950 50  0000 C CNN
F 2 "OwnConn:screwterminal" H 1850 5975 50  0001 C CNN
F 3 "~" H 1850 5975 50  0001 C CNN
	1    1850 5975
	-1   0    0    1   
$EndComp
Wire Wire Line
	2050 5875 2175 5875
Wire Wire Line
	2175 5875 2175 5675
Wire Wire Line
	2050 5975 2150 5975
Text GLabel 2425 5975 2    50   Input ~ 0
Move_complete
$Comp
L Device:R_Small R1
U 1 1 5F08467A
P 2150 6075
F 0 "R1" H 2209 6121 50  0000 L CNN
F 1 "4k7" H 2209 6030 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 2150 6075 50  0001 C CNN
F 3 "~" H 2150 6075 50  0001 C CNN
	1    2150 6075
	1    0    0    -1  
$EndComp
Connection ~ 2150 5975
Wire Wire Line
	2150 5975 2425 5975
$Comp
L OwnConn:GND #PWR03
U 1 1 5F084C62
P 2150 6175
F 0 "#PWR03" H 2150 5925 50  0001 C CNN
F 1 "GND" H 2155 6002 50  0000 C CNN
F 2 "" H 2150 6175 50  0001 C CNN
F 3 "" H 2150 6175 50  0001 C CNN
	1    2150 6175
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 5F086F18
P 3650 5950
F 0 "J2" H 3625 5675 50  0000 C CNN
F 1 "Screw_Terminal_01x02" V 3750 5925 50  0000 C CNN
F 2 "OwnConn:screwterminal" H 3650 5950 50  0001 C CNN
F 3 "~" H 3650 5950 50  0001 C CNN
	1    3650 5950
	-1   0    0    1   
$EndComp
Wire Wire Line
	3850 5850 3975 5850
Wire Wire Line
	3975 5850 3975 5650
Wire Wire Line
	3850 5950 3950 5950
Text GLabel 3975 5650 1    50   Input ~ 0
Vcc
$Comp
L Device:R_Small R2
U 1 1 5F086F23
P 3950 6050
F 0 "R2" H 4009 6096 50  0000 L CNN
F 1 "4k7" H 4009 6005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 3950 6050 50  0001 C CNN
F 3 "~" H 3950 6050 50  0001 C CNN
	1    3950 6050
	1    0    0    -1  
$EndComp
Connection ~ 3950 5950
Wire Wire Line
	3950 5950 4225 5950
$Comp
L OwnConn:GND #PWR06
U 1 1 5F086F2B
P 3950 6150
F 0 "#PWR06" H 3950 5900 50  0001 C CNN
F 1 "GND" H 3955 5977 50  0000 C CNN
F 2 "" H 3950 6150 50  0001 C CNN
F 3 "" H 3950 6150 50  0001 C CNN
	1    3950 6150
	1    0    0    -1  
$EndComp
Text GLabel 4225 5950 2    50   Input ~ 0
Move_step
$Comp
L Connector:Screw_Terminal_01x02 J3
U 1 1 5F08F71E
P 5325 6000
F 0 "J3" H 5300 5725 50  0000 C CNN
F 1 "Screw_Terminal_01x02" V 5425 5975 50  0000 C CNN
F 2 "OwnConn:screwterminal" H 5325 6000 50  0001 C CNN
F 3 "~" H 5325 6000 50  0001 C CNN
	1    5325 6000
	-1   0    0    1   
$EndComp
Wire Wire Line
	5525 5900 5650 5900
Wire Wire Line
	5650 5900 5650 5700
Wire Wire Line
	5525 6000 5625 6000
$Comp
L Device:R_Small R3
U 1 1 5F08F729
P 5625 6100
F 0 "R3" H 5684 6146 50  0000 L CNN
F 1 "4k7" H 5684 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 5625 6100 50  0001 C CNN
F 3 "~" H 5625 6100 50  0001 C CNN
	1    5625 6100
	1    0    0    -1  
$EndComp
Connection ~ 5625 6000
Wire Wire Line
	5625 6000 5900 6000
$Comp
L OwnConn:GND #PWR07
U 1 1 5F08F731
P 5625 6200
F 0 "#PWR07" H 5625 5950 50  0001 C CNN
F 1 "GND" H 5630 6027 50  0000 C CNN
F 2 "" H 5625 6200 50  0001 C CNN
F 3 "" H 5625 6200 50  0001 C CNN
	1    5625 6200
	1    0    0    -1  
$EndComp
Text GLabel 5650 5700 2    50   Input ~ 0
Photo_Powr
Text GLabel 5900 6000 2    50   Input ~ 0
Photo_Sens
$Comp
L Connector:Screw_Terminal_01x02 J4
U 1 1 5F095001
P 7750 5175
F 0 "J4" H 7725 4900 50  0000 C CNN
F 1 "Screw_Terminal_01x02" V 7850 5150 50  0000 C CNN
F 2 "OwnConn:screwterminal" H 7750 5175 50  0001 C CNN
F 3 "~" H 7750 5175 50  0001 C CNN
	1    7750 5175
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7950 5175 8075 5175
Wire Wire Line
	8075 5175 8075 4975
Wire Wire Line
	7950 5275 8050 5275
$Comp
L Device:R_Small R4
U 1 1 5F09500C
P 8050 5375
F 0 "R4" H 8109 5421 50  0000 L CNN
F 1 "4k7" H 8109 5330 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 8050 5375 50  0001 C CNN
F 3 "~" H 8050 5375 50  0001 C CNN
	1    8050 5375
	1    0    0    -1  
$EndComp
Connection ~ 8050 5275
Wire Wire Line
	8050 5275 8325 5275
$Comp
L OwnConn:GND #PWR08
U 1 1 5F095014
P 8050 5475
F 0 "#PWR08" H 8050 5225 50  0001 C CNN
F 1 "GND" H 8055 5302 50  0000 C CNN
F 2 "" H 8050 5475 50  0001 C CNN
F 3 "" H 8050 5475 50  0001 C CNN
	1    8050 5475
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J6
U 1 1 5F09697B
P 9625 5325
F 0 "J6" H 9600 5050 50  0000 C CNN
F 1 "Screw_Terminal_01x02" V 9725 5300 50  0000 C CNN
F 2 "OwnConn:screwterminal" H 9625 5325 50  0001 C CNN
F 3 "~" H 9625 5325 50  0001 C CNN
	1    9625 5325
	-1   0    0    1   
$EndComp
Wire Wire Line
	9825 5225 9950 5225
Wire Wire Line
	9950 5225 9950 5025
Wire Wire Line
	9825 5325 9925 5325
$Comp
L Device:R_Small R6
U 1 1 5F096986
P 9925 5425
F 0 "R6" H 9984 5471 50  0000 L CNN
F 1 "4k7" H 9984 5380 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 9925 5425 50  0001 C CNN
F 3 "~" H 9925 5425 50  0001 C CNN
	1    9925 5425
	1    0    0    -1  
$EndComp
Connection ~ 9925 5325
Wire Wire Line
	9925 5325 10200 5325
$Comp
L OwnConn:GND #PWR010
U 1 1 5F09698E
P 9925 5550
F 0 "#PWR010" H 9925 5300 50  0001 C CNN
F 1 "GND" H 9930 5377 50  0000 C CNN
F 2 "" H 9925 5550 50  0001 C CNN
F 3 "" H 9925 5550 50  0001 C CNN
	1    9925 5550
	1    0    0    -1  
$EndComp
Text GLabel 8325 5275 2    50   Input ~ 0
End_down
Text GLabel 10200 5325 2    50   Input ~ 0
End_up
Text GLabel 9025 2100 0    50   Input ~ 0
End_down
Text GLabel 9025 2200 0    50   Input ~ 0
End_up
Wire Wire Line
	9025 2100 9200 2100
Wire Wire Line
	9200 2200 9025 2200
Text GLabel 10125 2200 2    50   Input ~ 0
5V_Motor_out
Text GLabel 10125 2000 2    50   Input ~ 0
5V_down
Text GLabel 2950 1550 1    50   Input ~ 0
5V_Motor_out
Text GLabel 2175 5675 1    50   Input ~ 0
Vcc
Text GLabel 10125 2100 2    50   Input ~ 0
5V_up
Text GLabel 9950 4825 1    50   Input ~ 0
5V_up
Text GLabel 8075 4775 1    50   Input ~ 0
5V_down
$Comp
L Device:R_Small R5
U 1 1 5F071F53
P 8075 4875
F 0 "R5" H 8134 4921 50  0000 L CNN
F 1 "470R" H 8134 4830 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 8075 4875 50  0001 C CNN
F 3 "~" H 8075 4875 50  0001 C CNN
	1    8075 4875
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5F072C4E
P 9950 4925
F 0 "R7" H 10009 4971 50  0000 L CNN
F 1 "470R" H 10009 4880 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 9950 4925 50  0001 C CNN
F 3 "~" H 9950 4925 50  0001 C CNN
	1    9950 4925
	1    0    0    -1  
$EndComp
Wire Wire Line
	9925 5525 9925 5550
$Comp
L Connector:Screw_Terminal_01x02 J8
U 1 1 5F07F387
P 800 1175
F 0 "J8" H 775 900 50  0000 C CNN
F 1 "Screw_Terminal_01x02" V 900 1150 50  0000 C CNN
F 2 "OwnConn:screwterminal" H 800 1175 50  0001 C CNN
F 3 "~" H 800 1175 50  0001 C CNN
	1    800  1175
	-1   0    0    1   
$EndComp
Wire Wire Line
	1000 1175 1000 1300
$Comp
L logo:Logo_Plain L1
U 1 1 5F09A7E6
P 10475 3225
F 0 "L1" H 10603 3271 50  0000 L CNN
F 1 "Logo_Plain" H 10603 3180 50  0000 L CNN
F 2 "logo:Logo_8x5" H 10475 3225 50  0001 C CNN
F 3 "" H 10475 3225 50  0001 C CNN
	1    10475 3225
	1    0    0    -1  
$EndComp
$EndSCHEMATC
