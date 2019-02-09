package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class DistanceScalingArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (2.00,13.50,0.00)
	// (12.00,13.50,0.00)
	
    public DistanceScalingArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public DistanceScalingArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{0.000,0.000,10.000,0.000},
				{0.209,2.086,10.000,0.000},
				{0.626,4.172,10.000,0.000},
				{1.252,6.258,10.000,0.000},
				{2.086,8.344,10.000,0.000},
				{3.129,10.430,10.000,0.000},
				{4.381,12.516,10.000,0.000},
				{5.841,14.603,10.000,0.000},
				{7.510,16.689,10.000,0.000},
				{9.387,18.775,10.000,0.000},
				{11.473,20.861,10.000,0.000},
				{13.768,22.947,10.000,0.000},
				{16.271,25.033,10.000,0.000},
				{18.983,27.119,10.000,0.000},
				{21.904,29.205,10.000,0.000},
				{25.033,31.291,10.000,0.000},
				{28.371,33.377,10.000,0.000},
				{31.917,35.463,10.000,0.000},
				{35.672,37.549,10.000,0.000},
				{39.635,39.635,10.000,0.000},
				{43.808,41.722,10.000,0.000},
				{48.188,43.808,10.000,0.000},
				{52.778,45.894,10.000,0.000},
				{57.576,47.980,10.000,0.000},
				{62.582,50.066,10.000,0.000},
				{67.797,52.152,10.000,0.000},
				{73.221,54.238,10.000,0.000},
				{78.854,56.324,10.000,0.000},
				{84.695,58.410,10.000,0.000},
				{90.744,60.496,10.000,0.000},
				{97.003,62.582,10.000,0.000},
				{103.469,64.668,10.000,0.000},
				{110.145,66.754,10.000,0.000},
				{117.029,68.840,10.000,0.000},
				{124.122,70.927,10.000,0.000},
				{131.423,73.013,10.000,0.000},
				{138.933,75.099,10.000,0.000},
				{146.651,77.185,10.000,0.000},
				{154.474,78.228,10.000,0.000},
				{162.297,78.228,10.000,0.000},
				{170.119,78.228,10.000,0.000},
				{177.942,78.228,10.000,0.000},
				{185.765,78.228,10.000,0.000},
				{193.588,78.228,10.000,0.000},
				{201.411,78.228,10.000,0.000},
				{209.233,78.228,10.000,0.000},
				{217.056,78.228,10.000,0.000},
				{224.879,78.228,10.000,0.000},
				{232.702,78.228,10.000,0.000},
				{240.525,78.228,10.000,0.000},
				{248.347,78.228,10.000,0.000},
				{256.170,78.228,10.000,0.000},
				{263.993,78.228,10.000,0.000},
				{271.816,78.228,10.000,0.000},
				{279.638,78.228,10.000,0.000},
				{287.461,78.228,10.000,0.000},
				{295.284,78.228,10.000,0.000},
				{303.107,78.228,10.000,0.000},
				{310.930,78.228,10.000,0.000},
				{318.752,78.228,10.000,0.000},
				{326.575,78.228,10.000,0.000},
				{334.398,78.228,10.000,0.000},
				{342.221,78.228,10.000,0.000},
				{350.043,78.228,10.000,0.000},
				{357.866,78.228,10.000,0.000},
				{365.689,78.228,10.000,0.000},
				{373.512,78.228,10.000,0.000},
				{381.335,78.228,10.000,0.000},
				{389.157,78.228,10.000,0.000},
				{396.980,78.228,10.000,0.000},
				{404.803,78.228,10.000,0.000},
				{412.626,78.228,10.000,0.000},
				{420.449,78.228,10.000,0.000},
				{428.271,78.228,10.000,0.000},
				{436.094,78.228,10.000,0.000},
				{443.917,78.228,10.000,0.000},
				{451.740,78.228,10.000,0.000},
				{459.562,78.228,10.000,0.000},
				{467.385,78.228,10.000,0.000},
				{475.208,78.228,10.000,0.000},
				{483.031,78.228,10.000,0.000},
				{490.854,78.228,10.000,0.000},
				{498.676,78.228,10.000,0.000},
				{506.499,78.228,10.000,0.000},
				{514.322,78.228,10.000,0.000},
				{522.145,78.228,10.000,0.000},
				{529.968,78.228,10.000,0.000},
				{537.790,78.228,10.000,0.000},
				{545.613,78.228,10.000,0.000},
				{553.436,78.228,10.000,0.000},
				{561.259,78.228,10.000,0.000},
				{569.081,78.228,10.000,0.000},
				{576.904,78.228,10.000,0.000},
				{584.727,78.228,10.000,0.000},
				{592.550,78.228,10.000,0.000},
				{600.373,78.228,10.000,0.000},
				{608.195,78.228,10.000,0.000},
				{616.018,78.228,10.000,0.000},
				{623.841,78.228,10.000,0.000},
				{631.664,78.228,10.000,0.000},
				{639.486,78.228,10.000,0.000},
				{647.309,78.228,10.000,0.000},
				{655.132,78.228,10.000,0.000},
				{662.955,78.228,10.000,0.000},
				{670.778,78.228,10.000,0.000},
				{678.600,78.228,10.000,0.000},
				{686.423,78.228,10.000,0.000},
				{694.246,78.228,10.000,0.000},
				{702.069,78.228,10.000,0.000},
				{709.892,78.228,10.000,0.000},
				{717.714,78.228,10.000,0.000},
				{725.537,78.228,10.000,0.000},
				{733.360,78.228,10.000,0.000},
				{741.183,78.228,10.000,0.000},
				{749.005,78.228,10.000,0.000},
				{756.828,78.228,10.000,0.000},
				{764.651,78.228,10.000,0.000},
				{772.474,78.228,10.000,0.000},
				{780.297,78.228,10.000,0.000},
				{788.119,78.228,10.000,0.000},
				{795.942,78.228,10.000,0.000},
				{803.765,78.228,10.000,0.000},
				{811.588,78.228,10.000,0.000},
				{819.411,78.228,10.000,0.000},
				{827.233,78.228,10.000,0.000},
				{835.056,78.228,10.000,0.000},
				{842.879,78.228,10.000,0.000},
				{850.702,78.228,10.000,0.000},
				{858.524,78.228,10.000,0.000},
				{866.347,78.228,10.000,0.000},
				{874.170,78.228,10.000,0.000},
				{881.993,78.228,10.000,0.000},
				{889.816,78.228,10.000,0.000},
				{897.638,78.228,10.000,0.000},
				{905.461,78.228,10.000,0.000},
				{913.284,78.228,10.000,0.000},
				{921.107,78.228,10.000,0.000},
				{928.929,78.228,10.000,0.000},
				{936.752,78.228,10.000,0.000},
				{944.575,78.228,10.000,0.000},
				{952.398,78.228,10.000,0.000},
				{960.221,78.228,10.000,0.000},
				{968.043,78.228,10.000,0.000},
				{975.866,78.228,10.000,0.000},
				{983.689,78.228,10.000,0.000},
				{991.512,78.228,10.000,0.000},
				{999.335,78.228,10.000,0.000},
				{1007.157,78.228,10.000,0.000},
				{1014.980,78.228,10.000,0.000},
				{1022.803,78.228,10.000,0.000},
				{1030.626,78.228,10.000,0.000},
				{1038.448,78.228,10.000,0.000},
				{1046.271,78.228,10.000,0.000},
				{1054.094,78.228,10.000,0.000},
				{1061.917,78.228,10.000,0.000},
				{1069.740,78.228,10.000,0.000},
				{1077.562,78.228,10.000,0.000},
				{1085.385,78.228,10.000,0.000},
				{1093.208,78.228,10.000,0.000},
				{1101.031,78.228,10.000,0.000},
				{1108.854,78.228,10.000,0.000},
				{1116.676,78.228,10.000,0.000},
				{1124.499,78.228,10.000,0.000},
				{1132.322,78.228,10.000,0.000},
				{1140.145,78.228,10.000,0.000},
				{1147.967,78.228,10.000,0.000},
				{1155.790,78.228,10.000,0.000},
				{1163.613,78.228,10.000,0.000},
				{1171.436,78.228,10.000,0.000},
				{1179.259,78.228,10.000,0.000},
				{1187.081,78.228,10.000,0.000},
				{1194.904,78.228,10.000,0.000},
				{1202.727,78.228,10.000,0.000},
				{1210.550,78.228,10.000,0.000},
				{1218.372,78.228,10.000,0.000},
				{1226.195,78.228,10.000,0.000},
				{1234.018,78.228,10.000,0.000},
				{1241.841,78.228,10.000,0.000},
				{1249.664,78.228,10.000,0.000},
				{1257.486,78.228,10.000,0.000},
				{1265.309,78.228,10.000,0.000},
				{1273.132,78.228,10.000,0.000},
				{1280.955,78.228,10.000,0.000},
				{1288.778,78.228,10.000,0.000},
				{1296.600,78.228,10.000,0.000},
				{1304.423,78.228,10.000,0.000},
				{1312.246,78.228,10.000,0.000},
				{1320.069,78.228,10.000,0.000},
				{1327.891,78.228,10.000,0.000},
				{1335.714,78.228,10.000,0.000},
				{1343.537,78.228,10.000,0.000},
				{1351.360,78.228,10.000,0.000},
				{1359.183,78.228,10.000,0.000},
				{1367.005,78.228,10.000,0.000},
				{1374.828,78.228,10.000,0.000},
				{1382.651,78.228,10.000,0.000},
				{1390.474,78.228,10.000,0.000},
				{1398.297,78.228,10.000,0.000},
				{1406.119,78.228,10.000,0.000},
				{1413.942,78.228,10.000,0.000},
				{1421.765,78.228,10.000,0.000},
				{1429.588,78.228,10.000,0.000},
				{1437.410,78.228,10.000,0.000},
				{1445.233,78.228,10.000,0.000},
				{1453.056,78.228,10.000,0.000},
				{1460.879,78.228,10.000,0.000},
				{1468.702,78.228,10.000,0.000},
				{1476.524,78.228,10.000,0.000},
				{1484.347,78.228,10.000,0.000},
				{1492.170,78.228,10.000,0.000},
				{1499.993,78.228,10.000,0.000},
				{1507.815,78.228,10.000,0.000},
				{1515.638,78.228,10.000,0.000},
				{1523.461,78.228,10.000,0.000},
				{1531.284,78.228,10.000,0.000},
				{1539.107,78.228,10.000,0.000},
				{1546.929,78.228,10.000,0.000},
				{1554.752,78.228,10.000,0.000},
				{1562.575,78.228,10.000,0.000},
				{1570.398,78.228,10.000,0.000},
				{1578.221,78.228,10.000,0.000},
				{1586.043,78.228,10.000,0.000},
				{1593.866,78.228,10.000,0.000},
				{1601.689,78.228,10.000,0.000},
				{1609.512,78.228,10.000,0.000},
				{1617.334,78.228,10.000,0.000},
				{1625.157,78.228,10.000,0.000},
				{1632.980,78.228,10.000,0.000},
				{1640.803,78.228,10.000,0.000},
				{1648.626,78.228,10.000,0.000},
				{1656.448,78.228,10.000,0.000},
				{1664.271,78.228,10.000,0.000},
				{1672.094,78.228,10.000,0.000},
				{1679.917,78.228,10.000,0.000},
				{1687.740,78.228,10.000,0.000},
				{1695.562,78.228,10.000,0.000},
				{1703.385,78.228,10.000,0.000},
				{1711.208,78.228,10.000,0.000},
				{1719.031,78.228,10.000,0.000},
				{1726.853,78.228,10.000,0.000},
				{1734.676,78.228,10.000,0.000},
				{1742.499,78.228,10.000,0.000},
				{1750.322,78.228,10.000,0.000},
				{1758.145,78.228,10.000,0.000},
				{1765.967,78.228,10.000,0.000},
				{1773.790,78.228,10.000,0.000},
				{1781.613,78.228,10.000,0.000},
				{1789.436,78.228,10.000,0.000},
				{1797.258,78.228,10.000,0.000},
				{1805.081,78.228,10.000,0.000},
				{1812.904,78.228,10.000,0.000},
				{1820.727,78.228,10.000,0.000},
				{1828.550,78.228,10.000,0.000},
				{1836.372,78.228,10.000,0.000},
				{1844.195,78.228,10.000,0.000},
				{1852.018,78.228,10.000,0.000},
				{1859.841,78.228,10.000,0.000},
				{1867.664,78.228,10.000,0.000},
				{1875.486,78.228,10.000,0.000},
				{1883.309,78.228,10.000,0.000},
				{1891.132,78.228,10.000,0.000},
				{1898.955,78.228,10.000,0.000},
				{1906.777,78.228,10.000,0.000},
				{1914.600,78.228,10.000,0.000},
				{1922.423,78.228,10.000,0.000},
				{1930.246,78.228,10.000,0.000},
				{1938.069,78.228,10.000,0.000},
				{1945.891,78.228,10.000,0.000},
				{1953.714,78.228,10.000,0.000},
				{1961.537,78.228,10.000,0.000},
				{1969.360,78.228,10.000,0.000},
				{1977.183,78.228,10.000,0.000},
				{1985.005,78.228,10.000,0.000},
				{1992.828,78.228,10.000,0.000},
				{2000.651,78.228,10.000,0.000},
				{2008.474,78.228,10.000,0.000},
				{2016.296,78.228,10.000,0.000},
				{2024.119,78.228,10.000,0.000},
				{2031.942,78.228,10.000,0.000},
				{2039.765,78.228,10.000,0.000},
				{2047.588,78.228,10.000,0.000},
				{2055.410,78.228,10.000,0.000},
				{2063.233,78.228,10.000,0.000},
				{2071.056,78.228,10.000,0.000},
				{2078.879,78.228,10.000,0.000},
				{2086.701,78.228,10.000,0.000},
				{2094.524,78.228,10.000,0.000},
				{2102.347,78.228,10.000,0.000},
				{2110.170,78.228,10.000,0.000},
				{2117.993,78.228,10.000,0.000},
				{2125.815,78.228,10.000,0.000},
				{2133.638,78.228,10.000,0.000},
				{2141.461,78.228,10.000,0.000},
				{2149.284,78.228,10.000,0.000},
				{2157.107,78.228,10.000,0.000},
				{2164.929,78.228,10.000,0.000},
				{2172.752,78.228,10.000,0.000},
				{2180.575,78.228,10.000,0.000},
				{2188.398,78.228,10.000,0.000},
				{2196.220,78.228,10.000,0.000},
				{2204.043,78.228,10.000,0.000},
				{2211.866,78.228,10.000,0.000},
				{2219.689,78.228,10.000,0.000},
				{2227.512,78.228,10.000,0.000},
				{2235.334,78.228,10.000,0.000},
				{2243.157,78.228,10.000,0.000},
				{2250.980,78.228,10.000,0.000},
				{2258.803,78.228,10.000,0.000},
				{2266.626,78.228,10.000,0.000},
				{2274.448,78.228,10.000,0.000},
				{2282.271,78.228,10.000,0.000},
				{2290.094,78.228,10.000,0.000},
				{2297.917,78.228,10.000,0.000},
				{2305.739,78.228,10.000,0.000},
				{2313.562,78.228,10.000,0.000},
				{2321.385,78.228,10.000,0.000},
				{2329.208,78.228,10.000,0.000},
				{2337.031,78.228,10.000,0.000},
				{2344.853,78.228,10.000,0.000},
				{2352.676,78.228,10.000,0.000},
				{2360.499,78.228,10.000,0.000},
				{2368.322,78.228,10.000,0.000},
				{2376.144,78.228,10.000,0.000},
				{2383.967,78.228,10.000,0.000},
				{2391.790,78.228,10.000,0.000},
				{2399.613,78.228,10.000,0.000},
				{2407.436,78.228,10.000,0.000},
				{2415.258,78.228,10.000,0.000},
				{2423.081,78.228,10.000,0.000},
				{2430.904,78.228,10.000,0.000},
				{2438.727,78.228,10.000,0.000},
				{2446.550,78.228,10.000,0.000},
				{2454.372,78.228,10.000,0.000},
				{2462.195,78.228,10.000,0.000},
				{2470.018,78.228,10.000,0.000},
				{2477.632,76.142,10.000,0.000},
				{2485.038,74.056,10.000,0.000},
				{2492.235,71.970,10.000,0.000},
				{2499.223,69.884,10.000,0.000},
				{2506.003,67.797,10.000,0.000},
				{2512.574,65.711,10.000,0.000},
				{2518.936,63.625,10.000,0.000},
				{2525.090,61.539,10.000,0.000},
				{2531.036,59.453,10.000,0.000},
				{2536.772,57.367,10.000,0.000},
				{2542.300,55.281,10.000,0.000},
				{2547.620,53.195,10.000,0.000},
				{2552.731,51.109,10.000,0.000},
				{2557.633,49.023,10.000,0.000},
				{2562.327,46.937,10.000,0.000},
				{2566.812,44.851,10.000,0.000},
				{2571.088,42.765,10.000,0.000},
				{2575.156,40.678,10.000,0.000},
				{2579.015,38.592,10.000,0.000},
				{2582.666,36.506,10.000,0.000},
				{2586.108,34.420,10.000,0.000},
				{2589.341,32.334,10.000,0.000},
				{2592.366,30.248,10.000,0.000},
				{2595.182,28.162,10.000,0.000},
				{2597.790,26.076,10.000,0.000},
				{2600.189,23.990,10.000,0.000},
				{2602.379,21.904,10.000,0.000},
				{2604.361,19.818,10.000,0.000},
				{2606.134,17.732,10.000,0.000},
				{2607.699,15.646,10.000,0.000},
				{2609.055,13.559,10.000,0.000},
				{2610.202,11.473,10.000,0.000},
				{2611.141,9.387,10.000,0.000},
				{2611.871,7.301,10.000,0.000},
				{2612.393,5.215,10.000,0.000}		};

}