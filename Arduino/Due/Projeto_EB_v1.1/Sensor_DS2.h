/* 
 * File:   Sensor_DS2.h
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SENSOR_DS2_H
#define	SENSOR_DS2_H

float converte_DS2(int adc_value);

// UA = UV(0.003*p+0.12)
//[(UA/UV)-0.12]/0.003
//333*[(UA/UV)-0.12 ]
//UA = byte_valeu*(5V/256))
//UV =~ 5V
//333*[(byte_value/256)-0.12 ]
//byte_value*1,3175230567 - 40

signed int DS2_convert_table [256] = {-400,-387,-374,-360,-347,-334,-321,-308,-295,-281,-268,-255,-242,-229,-216
,-202,-189,-176,-163,-150,-136,-123,-110,-97,-84,-71,-57,-44,-31,-18,-5,8,22,35,48,61,74,87,101
,114,127,140,153,167,180,193,206,219,232,246,259,272,285,298,311,325,338,351,364,377,391,404,417
,430,443,456,470,483,496,509,522,535,549,562,575,588,601,614,628,641,654,667,680,694,707,720,733,746,759
,773,786,799,812,825,838,852,865,878,891,904,918,931,944,957,970,983,997,1010,1023,1036,1049,1062,1076
,1089,1102,1115,1128,1142,1155,1168,1181,1194,1207,1221,1234,1247,1260,1273,1286,1300,1313,1326,1339,1352
,1365,1379,1392,1405,1418,1431,1445,1458,1471,1484,1497,1510,1524,1537,1550,1563,1576,1589,1603,1616,1629
,1642,1655,1669,1682,1695,1708,1721,1734,1748,1761,1774,1787,1800,1813,1827,1840,1853,1866,1879,1892,1906
,1919,1932,1945,1958,1972,1985,1998,2011,2024,2037,2051,2064,2077,2090,2103,2116,2130,2143,2156,2169,2182
,2196,2209,2222,2235,2248,2261,2275,2288,2301,2314,2327,2340,2354,2367,2380,2393,2406,2419,2433,2446,2459
,2472,2485,2499,2512,2525,2538,2551,2564,2578,2591,2604,2617,2630,2643,2657,2670,2683,2696,2709,2723,2736
,2749,2762,2775,2788,2802,2815,2828,2841,2854,2867,2881,2894,2907,2920,2933,2947,2960};

float DS2_convert_table_2 [256] = {-40.0,-38.7,-37.4,-36.0,-34.7,-33.4,-32.1,-30.8,-29.5,-28.1,-26.8,-25.5,-24.2,-22.9,-21.6
,-20.2,-18.9,-17.6,-16.3,-15.0,-13.6,-12.3,-11.0,-9.7,-8.4,-7.1,-5.7,-4.4,-3.1,-1.8,-0.5,0.8,2.2,3.5,4.8,6.1,7.4,8.7,10.1
,11.4,12.7,14.0,15.3,16.7,18.0,19.3,20.6,21.9,23.2,24.6,25.9,27.2,28.5,29.8,31.1,32.5,33.8,35.1,36.4,37.7,39.1,40.4,41.7
,43.0,44.3,45.6,47.0,48.3,49.6,50.9,52.2,53.5,54.9,56.2,57.5,58.8,60.1,61.4,62.8,64.1,65.4,66.7,68.0,69.4,70.7,72.0,73.3,74.6,75.9
,77.3,78.6,79.9,81.2,82.5,83.8,85.2,86.5,87.8,89.1,90.4,91.8,93.1,94.4,95.7,97.0,98.3,99.7,101.0,102.3,103.6,104.9,106.2,107.6
,108.9,110.2,111.5,112.8,114.2,115.5,116.8,118.1,119.4,120.7,122.1,123.4,124.7,126.0,127.3,128.6,130.0,131.3,132.6,133.9,135.2
,136.5,137.9,139.2,140.5,141.8,143.1,144.5,145.8,147.1,148.4,149.7,151.0,152.4,153.7,155.0,156.3,157.6,158.9,160.3,161.6,162.9
,164.2,165.5,166.9,168.2,169.5,170.8,172.1,173.4,174.8,176.1,177.4,178.7,180.0,181.3,182.7,184.0,185.3,186.6,187.9,189.2,190.6
,191.9,1932,1945,1958,1972,1985,1998,2011,2024,2037,2051,2064,2077,2090,2103,2116,2130,2143,2156,2169,2182
,219.6,220.9,222.2,223.5,224.8,226.1,227.5,228.8,230.1,231.4,232.7,234.0,235.4,236.7,238.0,239.3,240.6,241.9,243.3,244.6,245.9
,247.2,248.5,249.9,251.2,252.5,253.8,255.1,256.4,257.8,259.1,260.4,261.7,263.0,264.3,265.7,267.0,268.3,269.6,270.9,272.3,273.6
,274.9,276.2,277.5,278.8,280.2,281.5,282.8,284.1,285.4,286.7,288.1,289.4,290.7,292.0,293.3,294.7,296.0};


#endif	/* SENSOR_DS2_H */
