#include "esp_log.h"
#include "TLS3001.h"

#ifdef USE_GAMMA_CORR

    #define TABLE_SIZE (4096u)  //12 bits

	static const uint16_t gamma_corr_table[4096] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,
	3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
	3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,
	4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
	4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
	5,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
	8,8,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
	9,9,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
	10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,12,12,
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,13,13,13,13,13,13,
	13,13,13,13,13,13,13,13,13,13,14,14,14,14,14,14,14,14,14,14,
	14,14,14,14,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,
	16,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,17,
	17,17,17,17,17,18,18,18,18,18,18,18,18,18,18,18,18,19,19,19,
	19,19,19,19,19,19,19,19,19,20,20,20,20,20,20,20,20,20,20,20,
	20,21,21,21,21,21,21,21,21,21,21,21,22,22,22,22,22,22,22,22,
	22,22,22,23,23,23,23,23,23,23,23,23,23,23,24,24,24,24,24,24,
	24,24,24,24,25,25,25,25,25,25,25,25,25,25,26,26,26,26,26,26,
	26,26,26,26,27,27,27,27,27,27,27,27,27,28,28,28,28,28,28,28,
	28,28,28,29,29,29,29,29,29,29,29,29,30,30,30,30,30,30,30,30,
	30,31,31,31,31,31,31,31,31,32,32,32,32,32,32,32,32,32,33,33,
	33,33,33,33,33,33,34,34,34,34,34,34,34,34,35,35,35,35,35,35,
	35,35,36,36,36,36,36,36,36,36,37,37,37,37,37,37,37,37,38,38,
	38,38,38,38,38,39,39,39,39,39,39,39,39,40,40,40,40,40,40,40,
	41,41,41,41,41,41,41,42,42,42,42,42,42,42,43,43,43,43,43,43,
	43,44,44,44,44,44,44,44,45,45,45,45,45,45,45,46,46,46,46,46,
	46,46,47,47,47,47,47,47,48,48,48,48,48,48,48,49,49,49,49,49,
	49,50,50,50,50,50,50,50,51,51,51,51,51,51,52,52,52,52,52,52,
	53,53,53,53,53,53,54,54,54,54,54,54,55,55,55,55,55,55,56,56,
	56,56,56,56,57,57,57,57,57,57,58,58,58,58,58,59,59,59,59,59,
	59,60,60,60,60,60,60,61,61,61,61,61,62,62,62,62,62,62,63,63,
	63,63,63,64,64,64,64,64,65,65,65,65,65,65,66,66,66,66,66,67,
	67,67,67,67,68,68,68,68,68,69,69,69,69,69,70,70,70,70,70,71,
	71,71,71,71,72,72,72,72,72,73,73,73,73,73,74,74,74,74,74,75,
	75,75,75,75,76,76,76,76,76,77,77,77,77,78,78,78,78,78,79,79,
	79,79,79,80,80,80,80,81,81,81,81,81,82,82,82,82,82,83,83,83,
	83,84,84,84,84,84,85,85,85,85,86,86,86,86,87,87,87,87,87,88,
	88,88,88,89,89,89,89,90,90,90,90,90,91,91,91,91,92,92,92,92,
	93,93,93,93,94,94,94,94,95,95,95,95,95,96,96,96,96,97,97,97,
	97,98,98,98,98,99,99,99,99,100,100,100,100,101,101,101,101,102,102,102,
	102,103,103,103,103,104,104,104,105,105,105,105,106,106,106,106,107,107,107,107,
	108,108,108,108,109,109,109,110,110,110,110,111,111,111,111,112,112,112,112,113,
	113,113,114,114,114,114,115,115,115,115,116,116,116,117,117,117,117,118,118,118,
	119,119,119,119,120,120,120,121,121,121,121,122,122,122,123,123,123,123,124,124,
	124,125,125,125,125,126,126,126,127,127,127,127,128,128,128,129,129,129,130,130,
	130,130,131,131,131,132,132,132,133,133,133,133,134,134,134,135,135,135,136,136,
	136,137,137,137,137,138,138,138,139,139,139,140,140,140,141,141,141,142,142,142,
	143,143,143,143,144,144,144,145,145,145,146,146,146,147,147,147,148,148,148,149,
	149,149,150,150,150,151,151,151,152,152,152,153,153,153,154,154,154,155,155,155,
	156,156,156,157,157,157,158,158,158,159,159,159,160,160,160,161,161,161,162,162,
	162,163,163,164,164,164,165,165,165,166,166,166,167,167,167,168,168,168,169,169,
	170,170,170,171,171,171,172,172,172,173,173,173,174,174,175,175,175,176,176,176,
	177,177,177,178,178,179,179,179,180,180,180,181,181,182,182,182,183,183,183,184,
	184,185,185,185,186,186,186,187,187,188,188,188,189,189,190,190,190,191,191,191,
	192,192,193,193,193,194,194,195,195,195,196,196,197,197,197,198,198,199,199,199,
	200,200,200,201,201,202,202,202,203,203,204,204,205,205,205,206,206,207,207,207,
	208,208,209,209,209,210,210,211,211,211,212,212,213,213,214,214,214,215,215,216,
	216,216,217,217,218,218,219,219,219,220,220,221,221,222,222,222,223,223,224,224,
	225,225,225,226,226,227,227,228,228,228,229,229,230,230,231,231,232,232,232,233,
	233,234,234,235,235,235,236,236,237,237,238,238,239,239,240,240,240,241,241,242,
	242,243,243,244,244,244,245,245,246,246,247,247,248,248,249,249,250,250,250,251,
	251,252,252,253,253,254,254,255,255,256,256,257,257,257,258,258,259,259,260,260,
	261,261,262,262,263,263,264,264,265,265,266,266,267,267,267,268,268,269,269,270,
	270,271,271,272,272,273,273,274,274,275,275,276,276,277,277,278,278,279,279,280,
	280,281,281,282,282,283,283,284,284,285,285,286,286,287,287,288,288,289,289,290,
	290,291,291,292,292,293,293,294,294,295,296,296,297,297,298,298,299,299,300,300,
	301,301,302,302,303,303,304,304,305,305,306,306,307,308,308,309,309,310,310,311,
	311,312,312,313,313,314,315,315,316,316,317,317,318,318,319,319,320,320,321,322,
	322,323,323,324,324,325,325,326,327,327,328,328,329,329,330,330,331,332,332,333,
	333,334,334,335,335,336,337,337,338,338,339,339,340,341,341,342,342,343,343,344,
	345,345,346,346,347,347,348,349,349,350,350,351,351,352,353,353,354,354,355,356,
	356,357,357,358,358,359,360,360,361,361,362,363,363,364,364,365,366,366,367,367,
	368,369,369,370,370,371,372,372,373,373,374,375,375,376,376,377,378,378,379,379,
	380,381,381,382,382,383,384,384,385,386,386,387,387,388,389,389,390,391,391,392,
	392,393,394,394,395,396,396,397,397,398,399,399,400,401,401,402,402,403,404,404,
	405,406,406,407,408,408,409,409,410,411,411,412,413,413,414,415,415,416,417,417,
	418,419,419,420,421,421,422,422,423,424,424,425,426,426,427,428,428,429,430,430,
	431,432,432,433,434,434,435,436,436,437,438,438,439,440,440,441,442,442,443,444,
	444,445,446,447,447,448,449,449,450,451,451,452,453,453,454,455,455,456,457,458,
	458,459,460,460,461,462,462,463,464,464,465,466,467,467,468,469,469,470,471,471,
	472,473,474,474,475,476,476,477,478,479,479,480,481,481,482,483,484,484,485,486,
	486,487,488,489,489,490,491,492,492,493,494,494,495,496,497,497,498,499,500,500,
	501,502,502,503,504,505,505,506,507,508,508,509,510,511,511,512,513,514,514,515,
	516,517,517,518,519,520,520,521,522,523,523,524,525,526,526,527,528,529,529,530,
	531,532,533,533,534,535,536,536,537,538,539,539,540,541,542,543,543,544,545,546,
	546,547,548,549,550,550,551,552,553,553,554,555,556,557,557,558,559,560,561,561,
	562,563,564,565,565,566,567,568,568,569,570,571,572,573,573,574,575,576,577,577,
	578,579,580,581,581,582,583,584,585,585,586,587,588,589,590,590,591,592,593,594,
	594,595,596,597,598,599,599,600,601,602,603,604,604,605,606,607,608,609,609,610,
	611,612,613,614,614,615,616,617,618,619,620,620,621,622,623,624,625,625,626,627,
	628,629,630,631,631,632,633,634,635,636,637,637,638,639,640,641,642,643,644,644,
	645,646,647,648,649,650,650,651,652,653,654,655,656,657,658,658,659,660,661,662,
	663,664,665,665,666,667,668,669,670,671,672,673,674,674,675,676,677,678,679,680,
	681,682,682,683,684,685,686,687,688,689,690,691,692,692,693,694,695,696,697,698,
	699,700,701,702,703,703,704,705,706,707,708,709,710,711,712,713,714,715,716,716,
	717,718,719,720,721,722,723,724,725,726,727,728,729,730,731,731,732,733,734,735,
	736,737,738,739,740,741,742,743,744,745,746,747,748,749,750,751,752,752,753,754,
	755,756,757,758,759,760,761,762,763,764,765,766,767,768,769,770,771,772,773,774,
	775,776,777,778,779,780,781,782,783,784,785,786,787,788,789,790,791,792,793,794,
	795,796,797,798,799,800,801,802,803,804,805,806,807,808,809,810,811,812,813,814,
	815,816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,832,833,834,
	835,836,837,838,839,841,842,843,844,845,846,847,848,849,850,851,852,853,854,855,
	856,857,858,859,860,861,862,864,865,866,867,868,869,870,871,872,873,874,875,876,
	877,878,879,881,882,883,884,885,886,887,888,889,890,891,892,893,895,896,897,898,
	899,900,901,902,903,904,905,906,908,909,910,911,912,913,914,915,916,917,919,920,
	921,922,923,924,925,926,927,928,930,931,932,933,934,935,936,937,939,940,941,942,
	943,944,945,946,947,949,950,951,952,953,954,955,956,958,959,960,961,962,963,964,
	966,967,968,969,970,971,972,974,975,976,977,978,979,980,982,983,984,985,986,987,
	988,990,991,992,993,994,995,997,998,999,1000,1001,1002,1004,1005,1006,1007,1008,1009,1011,
	1012,1013,1014,1015,1016,1018,1019,1020,1021,1022,1024,1025,1026,1027,1028,1030,1031,1032,1033,1034,
	1035,1037,1038,1039,1040,1041,1043,1044,1045,1046,1047,1049,1050,1051,1052,1053,1055,1056,1057,1058,
	1059,1061,1062,1063,1064,1066,1067,1068,1069,1070,1072,1073,1074,1075,1077,1078,1079,1080,1081,1083,
	1084,1085,1086,1088,1089,1090,1091,1093,1094,1095,1096,1097,1099,1100,1101,1102,1104,1105,1106,1107,
	1109,1110,1111,1112,1114,1115,1116,1117,1119,1120,1121,1122,1124,1125,1126,1127,1129,1130,1131,1133,
	1134,1135,1136,1138,1139,1140,1141,1143,1144,1145,1147,1148,1149,1150,1152,1153,1154,1155,1157,1158,
	1159,1161,1162,1163,1164,1166,1167,1168,1170,1171,1172,1174,1175,1176,1177,1179,1180,1181,1183,1184,
	1185,1187,1188,1189,1190,1192,1193,1194,1196,1197,1198,1200,1201,1202,1204,1205,1206,1208,1209,1210,
	1211,1213,1214,1215,1217,1218,1219,1221,1222,1223,1225,1226,1227,1229,1230,1231,1233,1234,1235,1237,
	1238,1240,1241,1242,1244,1245,1246,1248,1249,1250,1252,1253,1254,1256,1257,1258,1260,1261,1262,1264,
	1265,1267,1268,1269,1271,1272,1273,1275,1276,1278,1279,1280,1282,1283,1284,1286,1287,1289,1290,1291,
	1293,1294,1295,1297,1298,1300,1301,1302,1304,1305,1307,1308,1309,1311,1312,1314,1315,1316,1318,1319,
	1321,1322,1323,1325,1326,1328,1329,1330,1332,1333,1335,1336,1337,1339,1340,1342,1343,1344,1346,1347,
	1349,1350,1352,1353,1354,1356,1357,1359,1360,1362,1363,1364,1366,1367,1369,1370,1372,1373,1375,1376,
	1377,1379,1380,1382,1383,1385,1386,1388,1389,1390,1392,1393,1395,1396,1398,1399,1401,1402,1404,1405,
	1406,1408,1409,1411,1412,1414,1415,1417,1418,1420,1421,1423,1424,1426,1427,1428,1430,1431,1433,1434,
	1436,1437,1439,1440,1442,1443,1445,1446,1448,1449,1451,1452,1454,1455,1457,1458,1460,1461,1463,1464,
	1466,1467,1469,1470,1472,1473,1475,1476,1478,1479,1481,1482,1484,1485,1487,1488,1490,1491,1493,1494,
	1496,1498,1499,1501,1502,1504,1505,1507,1508,1510,1511,1513,1514,1516,1517,1519,1520,1522,1524,1525,
	1527,1528,1530,1531,1533,1534,1536,1538,1539,1541,1542,1544,1545,1547,1548,1550,1552,1553,1555,1556,
	1558,1559,1561,1562,1564,1566,1567,1569,1570,1572,1573,1575,1577,1578,1580,1581,1583,1585,1586,1588,
	1589,1591,1592,1594,1596,1597,1599,1600,1602,1604,1605,1607,1608,1610,1612,1613,1615,1616,1618,1620,
	1621,1623,1624,1626,1628,1629,1631,1632,1634,1636,1637,1639,1641,1642,1644,1645,1647,1649,1650,1652,
	1654,1655,1657,1658,1660,1662,1663,1665,1667,1668,1670,1672,1673,1675,1676,1678,1680,1681,1683,1685,
	1686,1688,1690,1691,1693,1695,1696,1698,1700,1701,1703,1705,1706,1708,1710,1711,1713,1715,1716,1718,
	1720,1721,1723,1725,1726,1728,1730,1731,1733,1735,1736,1738,1740,1741,1743,1745,1746,1748,1750,1752,
	1753,1755,1757,1758,1760,1762,1763,1765,1767,1769,1770,1772,1774,1775,1777,1779,1780,1782,1784,1786,
	1787,1789,1791,1792,1794,1796,1798,1799,1801,1803,1805,1806,1808,1810,1811,1813,1815,1817,1818,1820,
	1822,1824,1825,1827,1829,1831,1832,1834,1836,1838,1839,1841,1843,1845,1846,1848,1850,1852,1853,1855,
	1857,1859,1860,1862,1864,1866,1867,1869,1871,1873,1874,1876,1878,1880,1882,1883,1885,1887,1889,1890,
	1892,1894,1896,1898,1899,1901,1903,1905,1907,1908,1910,1912,1914,1915,1917,1919,1921,1923,1924,1926,
	1928,1930,1932,1934,1935,1937,1939,1941,1943,1944,1946,1948,1950,1952,1953,1955,1957,1959,1961,1963,
	1964,1966,1968,1970,1972,1974,1975,1977,1979,1981,1983,1985,1986,1988,1990,1992,1994,1996,1997,1999,
	2001,2003,2005,2007,2009,2010,2012,2014,2016,2018,2020,2022,2023,2025,2027,2029,2031,2033,2035,2037,
	2038,2040,2042,2044,2046,2048,2050,2052,2053,2055,2057,2059,2061,2063,2065,2067,2069,2070,2072,2074,
	2076,2078,2080,2082,2084,2086,2087,2089,2091,2093,2095,2097,2099,2101,2103,2105,2107,2108,2110,2112,
	2114,2116,2118,2120,2122,2124,2126,2128,2130,2132,2133,2135,2137,2139,2141,2143,2145,2147,2149,2151,
	2153,2155,2157,2159,2161,2163,2164,2166,2168,2170,2172,2174,2176,2178,2180,2182,2184,2186,2188,2190,
	2192,2194,2196,2198,2200,2202,2204,2206,2208,2210,2212,2214,2216,2218,2220,2222,2223,2225,2227,2229,
	2231,2233,2235,2237,2239,2241,2243,2245,2247,2249,2251,2253,2255,2257,2259,2261,2263,2265,2267,2269,
	2271,2273,2275,2277,2280,2282,2284,2286,2288,2290,2292,2294,2296,2298,2300,2302,2304,2306,2308,2310,
	2312,2314,2316,2318,2320,2322,2324,2326,2328,2330,2332,2334,2336,2339,2341,2343,2345,2347,2349,2351,
	2353,2355,2357,2359,2361,2363,2365,2367,2369,2371,2374,2376,2378,2380,2382,2384,2386,2388,2390,2392,
	2394,2396,2399,2401,2403,2405,2407,2409,2411,2413,2415,2417,2419,2422,2424,2426,2428,2430,2432,2434,
	2436,2438,2441,2443,2445,2447,2449,2451,2453,2455,2457,2460,2462,2464,2466,2468,2470,2472,2474,2477,
	2479,2481,2483,2485,2487,2489,2492,2494,2496,2498,2500,2502,2504,2507,2509,2511,2513,2515,2517,2519,
	2522,2524,2526,2528,2530,2532,2535,2537,2539,2541,2543,2545,2548,2550,2552,2554,2556,2558,2561,2563,
	2565,2567,2569,2572,2574,2576,2578,2580,2583,2585,2587,2589,2591,2593,2596,2598,2600,2602,2605,2607,
	2609,2611,2613,2616,2618,2620,2622,2624,2627,2629,2631,2633,2635,2638,2640,2642,2644,2647,2649,2651,
	2653,2656,2658,2660,2662,2664,2667,2669,2671,2673,2676,2678,2680,2682,2685,2687,2689,2691,2694,2696,
	2698,2700,2703,2705,2707,2710,2712,2714,2716,2719,2721,2723,2725,2728,2730,2732,2734,2737,2739,2741,
	2744,2746,2748,2750,2753,2755,2757,2760,2762,2764,2766,2769,2771,2773,2776,2778,2780,2783,2785,2787,
	2790,2792,2794,2796,2799,2801,2803,2806,2808,2810,2813,2815,2817,2820,2822,2824,2827,2829,2831,2834,
	2836,2838,2841,2843,2845,2848,2850,2852,2855,2857,2859,2862,2864,2866,2869,2871,2873,2876,2878,2880,
	2883,2885,2888,2890,2892,2895,2897,2899,2902,2904,2907,2909,2911,2914,2916,2918,2921,2923,2926,2928,
	2930,2933,2935,2937,2940,2942,2945,2947,2949,2952,2954,2957,2959,2961,2964,2966,2969,2971,2973,2976,
	2978,2981,2983,2985,2988,2990,2993,2995,2998,3000,3002,3005,3007,3010,3012,3015,3017,3019,3022,3024,
	3027,3029,3032,3034,3036,3039,3041,3044,3046,3049,3051,3054,3056,3059,3061,3063,3066,3068,3071,3073,
	3076,3078,3081,3083,3086,3088,3091,3093,3095,3098,3100,3103,3105,3108,3110,3113,3115,3118,3120,3123,
	3125,3128,3130,3133,3135,3138,3140,3143,3145,3148,3150,3153,3155,3158,3160,3163,3165,3168,3170,3173,
	3175,3178,3180,3183,3185,3188,3190,3193,3195,3198,3201,3203,3206,3208,3211,3213,3216,3218,3221,3223,
	3226,3228,3231,3234,3236,3239,3241,3244,3246,3249,3251,3254,3257,3259,3262,3264,3267,3269,3272,3274,
	3277,3280,3282,3285,3287,3290,3292,3295,3298,3300,3303,3305,3308,3311,3313,3316,3318,3321,3324,3326,
	3329,3331,3334,3336,3339,3342,3344,3347,3350,3352,3355,3357,3360,3363,3365,3368,3370,3373,3376,3378,
	3381,3384,3386,3389,3391,3394,3397,3399,3402,3405,3407,3410,3412,3415,3418,3420,3423,3426,3428,3431,
	3434,3436,3439,3442,3444,3447,3450,3452,3455,3458,3460,3463,3466,3468,3471,3474,3476,3479,3482,3484,
	3487,3490,3492,3495,3498,3500,3503,3506,3508,3511,3514,3516,3519,3522,3525,3527,3530,3533,3535,3538,
	3541,3543,3546,3549,3552,3554,3557,3560,3562,3565,3568,3571,3573,3576,3579,3582,3584,3587,3590,3592,
	3595,3598,3601,3603,3606,3609,3612,3614,3617,3620,3623,3625,3628,3631,3634,3636,3639,3642,3645,3647,
	3650,3653,3656,3658,3661,3664,3667,3669,3672,3675,3678,3681,3683,3686,3689,3692,3694,3697,3700,3703,
	3706,3708,3711,3714,3717,3720,3722,3725,3728,3731,3734,3736,3739,3742,3745,3748,3750,3753,3756,3759,
	3762,3764,3767,3770,3773,3776,3779,3781,3784,3787,3790,3793,3796,3798,3801,3804,3807,3810,3813,3815,
	3818,3821,3824,3827,3830,3833,3835,3838,3841,3844,3847,3850,3853,3855,3858,3861,3864,3867,3870,3873,
	3875,3878,3881,3884,3887,3890,3893,3896,3898,3901,3904,3907,3910,3913,3916,3919,3922,3925,3927,3930,
	3933,3936,3939,3942,3945,3948,3951,3954,3956,3959,3962,3965,3968,3971,3974,3977,3980,3983,3986,3989,
	3992,3994,3997,4000,4003,4006,4009,4012,4015,4018,4021,4024,4027,4030,4033,4036,4039,4042,4045,4047,
	4050,4053,4056,4059,4062,4065,4068,4071,4074,4077,4080,4083,4086,4089,4092,4095};
#endif

static const char * TAG = "gamma_correction";

uint16_t gamma_lookup(uint16_t index)
{
    uint16_t value;

    if (index < TABLE_SIZE)
    {
        value = gamma_corr_table[index];
    }
    else
    {
        ESP_LOGW(TAG, "index is outside lookup table range! Index:%d. Ignoring gamma correction.", index);
        value = index;
    }

    return value;

}