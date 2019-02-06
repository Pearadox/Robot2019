package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class SpeedTestingArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (2.00,13.50,0.00)
	// (5.00,16.50,89.99)
	// (2.00,19.50,179.98)
	
    public SpeedTestingArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public SpeedTestingArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{0.000,0.000,10.000,0.000},
				{0.065,0.652,10.000,0.000},
				{0.196,1.304,10.000,0.000},
				{0.391,1.956,10.000,0.000},
				{0.652,2.608,10.000,0.000},
				{0.978,3.259,10.000,0.000},
				{1.369,3.911,10.000,0.000},
				{1.825,4.563,10.000,0.000},
				{2.347,5.215,10.000,0.000},
				{2.934,5.867,10.000,-0.010},
				{3.585,6.519,10.000,-0.010},
				{4.303,7.171,10.000,-0.010},
				{5.085,7.823,10.000,-0.020},
				{5.932,8.475,10.000,-0.030},
				{6.845,9.127,10.000,-0.030},
				{7.823,9.778,10.000,-0.040},
				{8.866,10.430,10.000,-0.060},
				{9.974,11.082,10.000,-0.070},
				{11.147,11.734,10.000,-0.090},
				{12.386,12.386,10.000,-0.110},
				{13.690,13.038,10.000,-0.130},
				{15.059,13.690,10.000,-0.160},
				{16.493,14.342,10.000,-0.190},
				{17.992,14.994,10.000,-0.230},
				{19.557,15.646,10.000,-0.270},
				{21.187,16.297,10.000,-0.320},
				{22.882,16.949,10.000,-0.370},
				{24.642,17.601,10.000,-0.430},
				{26.467,18.253,10.000,-0.490},
				{28.358,18.905,10.000,-0.570},
				{30.313,19.557,10.000,-0.650},
				{32.334,20.209,10.000,-0.730},
				{34.420,20.861,10.000,-0.830},
				{36.572,21.513,10.000,-0.940},
				{38.788,22.165,10.000,-1.060},
				{41.070,22.816,10.000,-1.180},
				{43.416,23.468,10.000,-1.320},
				{45.828,24.120,10.000,-1.480},
				{48.273,24.446,10.000,-1.640},
				{50.718,24.446,10.000,-1.810},
				{53.162,24.446,10.000,-1.990},
				{55.607,24.446,10.000,-2.180},
				{58.052,24.446,10.000,-2.390},
				{60.496,24.446,10.000,-2.600},
				{62.941,24.446,10.000,-2.820},
				{65.385,24.446,10.000,-3.050},
				{67.830,24.446,10.000,-3.290},
				{70.275,24.446,10.000,-3.540},
				{72.719,24.446,10.000,-3.800},
				{75.164,24.446,10.000,-4.070},
				{77.609,24.446,10.000,-4.350},
				{80.053,24.446,10.000,-4.650},
				{82.498,24.446,10.000,-4.960},
				{84.942,24.446,10.000,-5.270},
				{87.387,24.446,10.000,-5.610},
				{89.832,24.446,10.000,-5.950},
				{92.276,24.446,10.000,-6.310},
				{94.721,24.446,10.000,-6.680},
				{97.165,24.446,10.000,-7.060},
				{99.610,24.446,10.000,-7.460},
				{102.055,24.446,10.000,-7.870},
				{104.499,24.446,10.000,-8.300},
				{106.944,24.446,10.000,-8.740},
				{109.389,24.446,10.000,-9.200},
				{111.833,24.446,10.000,-9.670},
				{114.278,24.446,10.000,-10.160},
				{116.722,24.446,10.000,-10.670},
				{119.167,24.446,10.000,-11.190},
				{121.612,24.446,10.000,-11.740},
				{124.056,24.446,10.000,-12.300},
				{126.501,24.446,10.000,-12.880},
				{128.946,24.446,10.000,-13.480},
				{131.390,24.446,10.000,-14.100},
				{133.835,24.446,10.000,-14.740},
				{136.279,24.446,10.000,-15.400},
				{138.724,24.446,10.000,-16.080},
				{141.169,24.446,10.000,-16.780},
				{143.613,24.446,10.000,-17.510},
				{146.058,24.446,10.000,-18.260},
				{148.503,24.446,10.000,-19.030},
				{150.947,24.446,10.000,-19.820},
				{153.392,24.446,10.000,-20.640},
				{155.836,24.446,10.000,-21.480},
				{158.281,24.446,10.000,-22.340},
				{160.726,24.446,10.000,-23.230},
				{163.170,24.446,10.000,-24.150},
				{165.615,24.446,10.000,-25.080},
				{168.059,24.446,10.000,-26.040},
				{170.504,24.446,10.000,-27.030},
				{172.949,24.446,10.000,-28.040},
				{175.393,24.446,10.000,-29.060},
				{177.838,24.446,10.000,-30.120},
				{180.283,24.446,10.000,-31.190},
				{182.727,24.446,10.000,-32.280},
				{185.172,24.446,10.000,-33.390},
				{187.616,24.446,10.000,-34.520},
				{190.061,24.446,10.000,-35.670},
				{192.506,24.446,10.000,-36.830},
				{194.950,24.446,10.000,-38.000},
				{197.395,24.446,10.000,-39.190},
				{199.840,24.446,10.000,-40.380},
				{202.284,24.446,10.000,-41.590},
				{204.729,24.446,10.000,-42.800},
				{207.173,24.446,10.000,-44.010},
				{209.618,24.446,10.000,-45.220},
				{212.063,24.446,10.000,-46.440},
				{214.507,24.446,10.000,-47.650},
				{216.952,24.446,10.000,-48.860},
				{219.396,24.446,10.000,-50.060},
				{221.841,24.446,10.000,-51.250},
				{224.286,24.446,10.000,-52.430},
				{226.730,24.446,10.000,-53.600},
				{229.175,24.446,10.000,-54.760},
				{231.620,24.446,10.000,-55.900},
				{234.064,24.446,10.000,-57.020},
				{236.509,24.446,10.000,-58.120},
				{238.953,24.446,10.000,-59.210},
				{241.398,24.446,10.000,-60.270},
				{243.843,24.446,10.000,-61.320},
				{246.287,24.446,10.000,-62.340},
				{248.732,24.446,10.000,-63.340},
				{251.177,24.446,10.000,-64.310},
				{253.621,24.446,10.000,-65.260},
				{256.066,24.446,10.000,-66.190},
				{258.510,24.446,10.000,-67.090},
				{260.955,24.446,10.000,-67.970},
				{263.400,24.446,10.000,-68.830},
				{265.844,24.446,10.000,-69.660},
				{268.289,24.446,10.000,-70.470},
				{270.734,24.446,10.000,-71.260},
				{273.178,24.446,10.000,-72.020},
				{275.623,24.446,10.000,-72.760},
				{278.067,24.446,10.000,-73.470},
				{280.512,24.446,10.000,-74.170},
				{282.957,24.446,10.000,-74.840},
				{285.401,24.446,10.000,-75.500},
				{287.846,24.446,10.000,-76.130},
				{290.290,24.446,10.000,-76.740},
				{292.735,24.446,10.000,-77.330},
				{295.180,24.446,10.000,-77.900},
				{297.624,24.446,10.000,-78.460},
				{300.069,24.446,10.000,-79.000},
				{302.514,24.446,10.000,-79.510},
				{304.958,24.446,10.000,-80.010},
				{307.403,24.446,10.000,-80.500},
				{309.847,24.446,10.000,-80.970},
				{312.292,24.446,10.000,-81.420},
				{314.737,24.446,10.000,-81.850},
				{317.181,24.446,10.000,-82.280},
				{319.626,24.446,10.000,-82.680},
				{322.071,24.446,10.000,-83.070},
				{324.515,24.446,10.000,-83.450},
				{326.960,24.446,10.000,-83.820},
				{329.404,24.446,10.000,-84.170},
				{331.849,24.446,10.000,-84.510},
				{334.294,24.446,10.000,-84.840},
				{336.738,24.446,10.000,-85.150},
				{339.183,24.446,10.000,-85.450},
				{341.627,24.446,10.000,-85.740},
				{344.072,24.446,10.000,-86.020},
				{346.517,24.446,10.000,-86.290},
				{348.961,24.446,10.000,-86.550},
				{351.406,24.446,10.000,-86.790},
				{353.851,24.446,10.000,-87.030},
				{356.295,24.446,10.000,-87.260},
				{358.740,24.446,10.000,-87.470},
				{361.184,24.446,10.000,-87.680},
				{363.629,24.446,10.000,-87.880},
				{366.074,24.446,10.000,-88.060},
				{368.518,24.446,10.000,-88.240},
				{370.963,24.446,10.000,-88.410},
				{373.408,24.446,10.000,-88.570},
				{375.852,24.446,10.000,-88.720},
				{378.297,24.446,10.000,-88.860},
				{380.741,24.446,10.000,-89.000},
				{383.121,23.794,10.000,-89.120},
				{385.435,23.142,10.000,-89.230},
				{387.684,22.491,10.000,-89.330},
				{389.868,21.839,10.000,-89.420},
				{391.987,21.187,10.000,-89.500},
				{394.040,20.535,10.000,-89.570},
				{396.028,19.883,10.000,-89.640},
				{397.952,19.231,10.000,-89.700},
				{399.809,18.579,10.000,-89.750},
				{401.602,17.927,10.000,-89.790},
				{403.330,17.275,10.000,-89.830},
				{404.992,16.623,10.000,-89.860},
				{406.589,15.972,10.000,-89.890},
				{408.121,15.320,10.000,-89.920},
				{409.588,14.668,10.000,-89.940},
				{410.990,14.016,10.000,-89.950},
				{412.326,13.364,10.000,-89.960},
				{413.597,12.712,10.000,-89.970},
				{414.803,12.060,10.000,-89.980},
				{415.944,11.408,10.000,-89.990},
				{417.020,10.756,10.000,-89.990},
				{418.030,10.104,10.000,-89.990},
				{418.975,9.453,10.000,-89.990},
				{418.975,9.453,10.000,-89.990},
				{419.790,8.149,10.000,-89.990},
				{420.605,8.149,10.000,-89.990},
				{421.420,8.149,10.000,-90.000},
				{422.235,8.149,10.000,-90.000},
				{423.050,8.149,10.000,-90.010},
				{423.865,8.149,10.000,-90.010},
				{424.679,8.149,10.000,-90.020},
				{425.494,8.149,10.000,-90.030},
				{426.309,8.149,10.000,-90.040},
				{427.124,8.149,10.000,-90.040},
				{427.939,8.149,10.000,-90.060},
				{428.754,8.149,10.000,-90.070},
				{429.569,8.149,10.000,-90.080},
				{430.383,8.149,10.000,-90.090},
				{431.198,8.149,10.000,-90.110},
				{432.013,8.149,10.000,-90.120},
				{432.828,8.149,10.000,-90.140},
				{433.643,8.149,10.000,-90.160},
				{434.458,8.149,10.000,-90.170},
				{435.273,8.149,10.000,-90.190},
				{436.088,8.149,10.000,-90.210},
				{436.902,8.149,10.000,-90.230},
				{437.717,8.149,10.000,-90.250},
				{438.532,8.149,10.000,-90.280},
				{439.347,8.149,10.000,-90.300},
				{440.162,8.149,10.000,-90.330},
				{440.977,8.149,10.000,-90.350},
				{441.792,8.149,10.000,-90.380},
				{442.607,8.149,10.000,-90.400},
				{443.421,8.149,10.000,-90.430},
				{444.236,8.149,10.000,-90.460},
				{445.051,8.149,10.000,-90.490},
				{445.866,8.149,10.000,-90.520},
				{446.681,8.149,10.000,-90.550},
				{447.496,8.149,10.000,-90.590},
				{448.311,8.149,10.000,-90.620},
				{449.126,8.149,10.000,-90.660},
				{449.940,8.149,10.000,-90.690},
				{450.755,8.149,10.000,-90.730},
				{451.570,8.149,10.000,-90.770},
				{452.385,8.149,10.000,-90.800},
				{453.200,8.149,10.000,-90.840},
				{454.015,8.149,10.000,-90.880},
				{454.830,8.149,10.000,-90.920},
				{455.645,8.149,10.000,-90.970},
				{456.459,8.149,10.000,-91.010},
				{457.274,8.149,10.000,-91.050},
				{458.089,8.149,10.000,-91.100},
				{458.904,8.149,10.000,-91.150},
				{459.719,8.149,10.000,-91.190},
				{460.534,8.149,10.000,-91.240},
				{461.349,8.149,10.000,-91.290},
				{462.164,8.149,10.000,-91.340},
				{462.978,8.149,10.000,-91.390},
				{463.793,8.149,10.000,-91.440},
				{464.608,8.149,10.000,-91.500},
				{465.423,8.149,10.000,-91.550},
				{466.238,8.149,10.000,-91.600},
				{467.053,8.149,10.000,-91.660},
				{467.868,8.149,10.000,-91.720},
				{468.683,8.149,10.000,-91.780},
				{469.497,8.149,10.000,-91.830},
				{470.312,8.149,10.000,-91.890},
				{471.127,8.149,10.000,-91.960},
				{471.942,8.149,10.000,-92.020},
				{472.757,8.149,10.000,-92.080},
				{473.572,8.149,10.000,-92.150},
				{474.387,8.149,10.000,-92.210},
				{475.202,8.149,10.000,-92.280},
				{476.016,8.149,10.000,-92.340},
				{476.831,8.149,10.000,-92.410},
				{477.646,8.149,10.000,-92.480},
				{478.461,8.149,10.000,-92.550},
				{479.276,8.149,10.000,-92.620},
				{480.091,8.149,10.000,-92.700},
				{480.906,8.149,10.000,-92.770},
				{481.721,8.149,10.000,-92.850},
				{482.535,8.149,10.000,-92.920},
				{483.350,8.149,10.000,-93.000},
				{484.165,8.149,10.000,-93.080},
				{484.980,8.149,10.000,-93.160},
				{485.795,8.149,10.000,-93.240},
				{486.610,8.149,10.000,-93.320},
				{487.425,8.149,10.000,-93.400},
				{488.239,8.149,10.000,-93.490},
				{489.054,8.149,10.000,-93.570},
				{489.869,8.149,10.000,-93.660},
				{490.684,8.149,10.000,-93.750},
				{491.499,8.149,10.000,-93.840},
				{492.314,8.149,10.000,-93.930},
				{493.129,8.149,10.000,-94.020},
				{493.944,8.149,10.000,-94.110},
				{494.758,8.149,10.000,-94.210},
				{495.573,8.149,10.000,-94.300},
				{496.388,8.149,10.000,-94.400},
				{497.203,8.149,10.000,-94.500},
				{498.018,8.149,10.000,-94.590},
				{498.833,8.149,10.000,-94.690},
				{499.648,8.149,10.000,-94.800},
				{500.463,8.149,10.000,-94.900},
				{501.277,8.149,10.000,-95.000},
				{502.092,8.149,10.000,-95.110},
				{502.907,8.149,10.000,-95.220},
				{503.722,8.149,10.000,-95.320},
				{504.537,8.149,10.000,-95.430},
				{505.352,8.149,10.000,-95.550},
				{506.167,8.149,10.000,-95.660},
				{506.982,8.149,10.000,-95.770},
				{507.796,8.149,10.000,-95.890},
				{508.611,8.149,10.000,-96.000},
				{509.426,8.149,10.000,-96.120},
				{510.241,8.149,10.000,-96.240},
				{511.056,8.149,10.000,-96.360},
				{511.871,8.149,10.000,-96.490},
				{512.686,8.149,10.000,-96.610},
				{513.501,8.149,10.000,-96.740},
				{514.315,8.149,10.000,-96.860},
				{515.130,8.149,10.000,-96.990},
				{515.945,8.149,10.000,-97.120},
				{516.760,8.149,10.000,-97.250},
				{517.575,8.149,10.000,-97.390},
				{518.390,8.149,10.000,-97.520},
				{519.205,8.149,10.000,-97.660},
				{520.020,8.149,10.000,-97.800},
				{520.834,8.149,10.000,-97.940},
				{521.649,8.149,10.000,-98.080},
				{522.464,8.149,10.000,-98.220},
				{523.279,8.149,10.000,-98.370},
				{524.094,8.149,10.000,-98.510},
				{524.909,8.149,10.000,-98.660},
				{525.724,8.149,10.000,-98.810},
				{526.539,8.149,10.000,-98.970},
				{527.353,8.149,10.000,-99.120},
				{528.168,8.149,10.000,-99.270},
				{528.983,8.149,10.000,-99.430},
				{529.798,8.149,10.000,-99.590},
				{530.613,8.149,10.000,-99.750},
				{531.428,8.149,10.000,-99.910},
				{532.243,8.149,10.000,-100.080},
				{533.058,8.149,10.000,-100.250},
				{533.872,8.149,10.000,-100.410},
				{534.687,8.149,10.000,-100.580},
				{535.502,8.149,10.000,-100.760},
				{536.317,8.149,10.000,-100.930},
				{537.132,8.149,10.000,-101.110},
				{537.947,8.149,10.000,-101.280},
				{538.762,8.149,10.000,-101.460},
				{539.577,8.149,10.000,-101.650},
				{540.391,8.149,10.000,-101.830},
				{541.206,8.149,10.000,-102.020},
				{542.021,8.149,10.000,-102.210},
				{542.836,8.149,10.000,-102.400},
				{543.651,8.149,10.000,-102.590},
				{544.466,8.149,10.000,-102.780},
				{545.281,8.149,10.000,-102.980},
				{546.095,8.149,10.000,-103.180},
				{546.910,8.149,10.000,-103.380},
				{547.725,8.149,10.000,-103.580},
				{548.540,8.149,10.000,-103.790},
				{549.355,8.149,10.000,-104.000},
				{550.170,8.149,10.000,-104.210},
				{550.985,8.149,10.000,-104.420},
				{551.800,8.149,10.000,-104.630},
				{552.614,8.149,10.000,-104.850},
				{553.429,8.149,10.000,-105.070},
				{554.244,8.149,10.000,-105.290},
				{555.059,8.149,10.000,-105.510},
				{555.874,8.149,10.000,-105.740},
				{556.689,8.149,10.000,-105.970},
				{557.504,8.149,10.000,-106.200},
				{558.319,8.149,10.000,-106.430},
				{559.133,8.149,10.000,-106.670},
				{559.948,8.149,10.000,-106.910},
				{560.763,8.149,10.000,-107.150},
				{561.578,8.149,10.000,-107.390},
				{562.393,8.149,10.000,-107.640},
				{563.208,8.149,10.000,-107.890},
				{564.023,8.149,10.000,-108.140},
				{564.838,8.149,10.000,-108.390},
				{565.652,8.149,10.000,-108.650},
				{566.467,8.149,10.000,-108.900},
				{567.282,8.149,10.000,-109.170},
				{568.097,8.149,10.000,-109.430},
				{568.912,8.149,10.000,-109.700},
				{569.727,8.149,10.000,-109.960},
				{570.542,8.149,10.000,-110.240},
				{571.357,8.149,10.000,-110.510},
				{572.171,8.149,10.000,-110.790},
				{572.986,8.149,10.000,-111.070},
				{573.801,8.149,10.000,-111.350},
				{574.616,8.149,10.000,-111.630},
				{575.431,8.149,10.000,-111.920},
				{576.246,8.149,10.000,-112.210},
				{577.061,8.149,10.000,-112.500},
				{577.876,8.149,10.000,-112.800},
				{578.690,8.149,10.000,-113.100},
				{579.505,8.149,10.000,-113.400},
				{580.320,8.149,10.000,-113.700},
				{581.135,8.149,10.000,-114.010},
				{581.950,8.149,10.000,-114.310},
				{582.765,8.149,10.000,-114.620},
				{583.580,8.149,10.000,-114.940},
				{584.395,8.149,10.000,-115.260},
				{585.209,8.149,10.000,-115.570},
				{586.024,8.149,10.000,-115.900},
				{586.839,8.149,10.000,-116.220},
				{587.654,8.149,10.000,-116.550},
				{588.469,8.149,10.000,-116.880},
				{589.284,8.149,10.000,-117.210},
				{590.099,8.149,10.000,-117.540},
				{590.914,8.149,10.000,-117.880},
				{591.728,8.149,10.000,-118.220},
				{592.543,8.149,10.000,-118.560},
				{593.358,8.149,10.000,-118.910},
				{594.173,8.149,10.000,-119.250},
				{594.988,8.149,10.000,-119.600},
				{595.803,8.149,10.000,-119.950},
				{596.618,8.149,10.000,-120.310},
				{597.433,8.149,10.000,-120.670},
				{598.247,8.149,10.000,-121.020},
				{599.062,8.149,10.000,-121.390},
				{599.877,8.149,10.000,-121.750},
				{600.692,8.149,10.000,-122.110},
				{601.507,8.149,10.000,-122.480},
				{602.322,8.149,10.000,-122.850},
				{603.137,8.149,10.000,-123.220},
				{603.952,8.149,10.000,-123.600},
				{604.766,8.149,10.000,-123.970},
				{605.581,8.149,10.000,-124.350},
				{606.396,8.149,10.000,-124.730},
				{607.211,8.149,10.000,-125.110},
				{608.026,8.149,10.000,-125.490},
				{608.841,8.149,10.000,-125.880},
				{609.656,8.149,10.000,-126.260},
				{610.470,8.149,10.000,-126.650},
				{611.285,8.149,10.000,-127.040},
				{612.100,8.149,10.000,-127.430},
				{612.915,8.149,10.000,-127.820},
				{613.730,8.149,10.000,-128.220},
				{614.545,8.149,10.000,-128.610},
				{615.360,8.149,10.000,-129.010},
				{616.175,8.149,10.000,-129.410},
				{616.989,8.149,10.000,-129.800},
				{617.804,8.149,10.000,-130.200},
				{618.619,8.149,10.000,-130.600},
				{619.434,8.149,10.000,-131.000},
				{620.249,8.149,10.000,-131.410},
				{621.064,8.149,10.000,-131.810},
				{621.879,8.149,10.000,-132.210},
				{622.694,8.149,10.000,-132.610},
				{623.508,8.149,10.000,-133.020},
				{624.323,8.149,10.000,-133.420},
				{625.138,8.149,10.000,-133.830},
				{625.953,8.149,10.000,-134.230},
				{626.768,8.149,10.000,-134.640},
				{627.583,8.149,10.000,-135.040},
				{628.398,8.149,10.000,-135.450},
				{629.213,8.149,10.000,-135.850},
				{630.027,8.149,10.000,-136.260},
				{630.842,8.149,10.000,-136.660},
				{631.657,8.149,10.000,-137.070},
				{632.472,8.149,10.000,-137.470},
				{633.287,8.149,10.000,-137.870},
				{634.102,8.149,10.000,-138.280},
				{634.917,8.149,10.000,-138.680},
				{635.732,8.149,10.000,-139.080},
				{636.546,8.149,10.000,-139.480},
				{637.361,8.149,10.000,-139.880},
				{638.176,8.149,10.000,-140.280},
				{638.991,8.149,10.000,-140.680},
				{639.806,8.149,10.000,-141.070},
				{640.621,8.149,10.000,-141.470},
				{641.436,8.149,10.000,-141.860},
				{642.251,8.149,10.000,-142.260},
				{643.065,8.149,10.000,-142.650},
				{643.880,8.149,10.000,-143.040},
				{644.695,8.149,10.000,-143.430},
				{645.510,8.149,10.000,-143.810},
				{646.325,8.149,10.000,-144.200},
				{647.140,8.149,10.000,-144.580},
				{647.955,8.149,10.000,-144.970},
				{648.770,8.149,10.000,-145.350},
				{649.584,8.149,10.000,-145.730},
				{650.399,8.149,10.000,-146.100},
				{651.214,8.149,10.000,-146.480},
				{652.029,8.149,10.000,-146.850},
				{652.844,8.149,10.000,-147.220},
				{653.659,8.149,10.000,-147.590},
				{654.474,8.149,10.000,-147.960},
				{655.289,8.149,10.000,-148.320},
				{656.103,8.149,10.000,-148.690},
				{656.918,8.149,10.000,-149.050},
				{657.733,8.149,10.000,-149.400},
				{658.548,8.149,10.000,-149.760},
				{659.363,8.149,10.000,-150.110},
				{660.178,8.149,10.000,-150.460},
				{660.993,8.149,10.000,-150.810},
				{661.808,8.149,10.000,-151.160},
				{662.622,8.149,10.000,-151.500},
				{663.437,8.149,10.000,-151.840},
				{664.252,8.149,10.000,-152.180},
				{665.067,8.149,10.000,-152.520},
				{665.882,8.149,10.000,-152.850},
				{666.697,8.149,10.000,-153.190},
				{667.512,8.149,10.000,-153.510},
				{668.326,8.149,10.000,-153.840},
				{669.141,8.149,10.000,-154.160},
				{669.956,8.149,10.000,-154.480},
				{670.771,8.149,10.000,-154.800},
				{671.586,8.149,10.000,-155.120},
				{672.401,8.149,10.000,-155.430},
				{673.216,8.149,10.000,-155.740},
				{674.031,8.149,10.000,-156.050},
				{674.845,8.149,10.000,-156.350},
				{675.660,8.149,10.000,-156.660},
				{676.475,8.149,10.000,-156.960},
				{677.290,8.149,10.000,-157.250},
				{678.105,8.149,10.000,-157.550},
				{678.920,8.149,10.000,-157.840},
				{679.735,8.149,10.000,-158.130},
				{680.550,8.149,10.000,-158.420},
				{681.364,8.149,10.000,-158.700},
				{682.179,8.149,10.000,-158.980},
				{682.994,8.149,10.000,-159.260},
				{683.809,8.149,10.000,-159.530},
				{684.624,8.149,10.000,-159.810},
				{685.439,8.149,10.000,-160.080},
				{686.254,8.149,10.000,-160.350},
				{687.069,8.149,10.000,-160.610},
				{687.883,8.149,10.000,-160.870},
				{688.698,8.149,10.000,-161.140},
				{689.513,8.149,10.000,-161.390},
				{690.328,8.149,10.000,-161.650},
				{691.143,8.149,10.000,-161.900},
				{691.958,8.149,10.000,-162.150},
				{692.773,8.149,10.000,-162.400},
				{693.588,8.149,10.000,-162.640},
				{694.402,8.149,10.000,-162.890},
				{695.217,8.149,10.000,-163.130},
				{696.032,8.149,10.000,-163.360},
				{696.847,8.149,10.000,-163.600},
				{697.662,8.149,10.000,-163.830},
				{698.477,8.149,10.000,-164.060},
				{699.292,8.149,10.000,-164.290},
				{700.107,8.149,10.000,-164.510},
				{700.921,8.149,10.000,-164.740},
				{701.736,8.149,10.000,-164.960},
				{702.551,8.149,10.000,-165.180},
				{703.366,8.149,10.000,-165.390},
				{704.181,8.149,10.000,-165.610},
				{704.996,8.149,10.000,-165.820},
				{705.811,8.149,10.000,-166.030},
				{706.626,8.149,10.000,-166.240},
				{707.440,8.149,10.000,-166.440},
				{708.255,8.149,10.000,-166.640},
				{709.070,8.149,10.000,-166.840},
				{709.885,8.149,10.000,-167.040},
				{710.700,8.149,10.000,-167.240},
				{711.515,8.149,10.000,-167.430},
				{712.330,8.149,10.000,-167.620},
				{713.145,8.149,10.000,-167.810},
				{713.959,8.149,10.000,-168.000},
				{714.774,8.149,10.000,-168.190},
				{715.589,8.149,10.000,-168.370},
				{716.404,8.149,10.000,-168.550},
				{717.219,8.149,10.000,-168.730},
				{718.034,8.149,10.000,-168.910},
				{718.849,8.149,10.000,-169.080},
				{719.664,8.149,10.000,-169.260},
				{720.478,8.149,10.000,-169.430},
				{721.293,8.149,10.000,-169.600},
				{722.108,8.149,10.000,-169.770},
				{722.923,8.149,10.000,-169.930},
				{723.738,8.149,10.000,-170.100},
				{724.553,8.149,10.000,-170.260},
				{725.368,8.149,10.000,-170.420},
				{726.182,8.149,10.000,-170.580},
				{726.997,8.149,10.000,-170.730},
				{727.812,8.149,10.000,-170.890},
				{728.627,8.149,10.000,-171.040},
				{729.442,8.149,10.000,-171.190},
				{730.257,8.149,10.000,-171.340},
				{731.072,8.149,10.000,-171.490},
				{731.887,8.149,10.000,-171.640},
				{732.701,8.149,10.000,-171.780},
				{733.516,8.149,10.000,-171.920},
				{734.331,8.149,10.000,-172.070},
				{735.146,8.149,10.000,-172.210},
				{735.961,8.149,10.000,-172.340},
				{736.776,8.149,10.000,-172.480},
				{737.591,8.149,10.000,-172.610},
				{738.406,8.149,10.000,-172.750},
				{739.220,8.149,10.000,-172.880},
				{740.035,8.149,10.000,-173.010},
				{740.850,8.149,10.000,-173.140},
				{741.665,8.149,10.000,-173.260},
				{742.480,8.149,10.000,-173.390},
				{743.295,8.149,10.000,-173.510},
				{744.110,8.149,10.000,-173.630},
				{744.925,8.149,10.000,-173.760},
				{745.739,8.149,10.000,-173.870},
				{746.554,8.149,10.000,-173.990},
				{747.369,8.149,10.000,-174.110},
				{748.184,8.149,10.000,-174.220},
				{748.999,8.149,10.000,-174.340},
				{749.814,8.149,10.000,-174.450},
				{750.629,8.149,10.000,-174.560},
				{751.444,8.149,10.000,-174.670},
				{752.258,8.149,10.000,-174.780},
				{753.073,8.149,10.000,-174.880},
				{753.888,8.149,10.000,-174.990},
				{754.703,8.149,10.000,-175.090},
				{755.518,8.149,10.000,-175.200},
				{756.333,8.149,10.000,-175.300},
				{757.148,8.149,10.000,-175.400},
				{757.963,8.149,10.000,-175.500},
				{758.777,8.149,10.000,-175.590},
				{759.592,8.149,10.000,-175.690},
				{760.407,8.149,10.000,-175.780},
				{761.222,8.149,10.000,-175.880},
				{762.037,8.149,10.000,-175.970},
				{762.852,8.149,10.000,-176.060},
				{763.667,8.149,10.000,-176.150},
				{764.482,8.149,10.000,-176.240},
				{765.296,8.149,10.000,-176.330},
				{766.111,8.149,10.000,-176.410},
				{766.926,8.149,10.000,-176.500},
				{767.741,8.149,10.000,-176.580},
				{768.556,8.149,10.000,-176.670},
				{769.371,8.149,10.000,-176.750},
				{770.186,8.149,10.000,-176.830},
				{771.001,8.149,10.000,-176.910},
				{771.815,8.149,10.000,-176.990},
				{772.630,8.149,10.000,-177.060},
				{773.445,8.149,10.000,-177.140},
				{774.260,8.149,10.000,-177.210},
				{775.075,8.149,10.000,-177.290},
				{775.890,8.149,10.000,-177.360},
				{776.705,8.149,10.000,-177.430},
				{777.520,8.149,10.000,-177.500},
				{778.334,8.149,10.000,-177.570},
				{779.149,8.149,10.000,-177.640},
				{779.964,8.149,10.000,-177.710},
				{780.779,8.149,10.000,-177.770},
				{781.594,8.149,10.000,-177.840},
				{782.409,8.149,10.000,-177.900},
				{783.224,8.149,10.000,-177.960},
				{784.039,8.149,10.000,-178.030},
				{784.853,8.149,10.000,-178.090},
				{785.668,8.149,10.000,-178.150},
				{786.483,8.149,10.000,-178.210},
				{787.298,8.149,10.000,-178.260},
				{788.113,8.149,10.000,-178.320},
				{788.928,8.149,10.000,-178.380},
				{789.743,8.149,10.000,-178.430},
				{790.557,8.149,10.000,-178.480},
				{791.372,8.149,10.000,-178.540},
				{792.187,8.149,10.000,-178.590},
				{793.002,8.149,10.000,-178.640},
				{793.817,8.149,10.000,-178.690},
				{794.632,8.149,10.000,-178.740},
				{795.447,8.149,10.000,-178.790},
				{796.262,8.149,10.000,-178.830},
				{797.076,8.149,10.000,-178.880},
				{797.891,8.149,10.000,-178.920},
				{798.706,8.149,10.000,-178.970},
				{799.521,8.149,10.000,-179.010},
				{800.336,8.149,10.000,-179.050},
				{801.151,8.149,10.000,-179.090},
				{801.966,8.149,10.000,-179.130},
				{802.781,8.149,10.000,-179.170},
				{803.595,8.149,10.000,-179.210},
				{804.410,8.149,10.000,-179.250},
				{805.225,8.149,10.000,-179.280},
				{806.040,8.149,10.000,-179.320},
				{806.855,8.149,10.000,-179.350},
				{807.670,8.149,10.000,-179.390},
				{808.485,8.149,10.000,-179.420},
				{809.300,8.149,10.000,-179.450},
				{810.114,8.149,10.000,-179.480},
				{810.929,8.149,10.000,-179.510},
				{811.744,8.149,10.000,-179.540},
				{812.559,8.149,10.000,-179.570},
				{813.374,8.149,10.000,-179.600},
				{814.189,8.149,10.000,-179.620},
				{815.004,8.149,10.000,-179.650},
				{815.819,8.149,10.000,-179.670},
				{816.633,8.149,10.000,-179.700},
				{817.448,8.149,10.000,-179.720},
				{818.263,8.149,10.000,-179.740},
				{819.078,8.149,10.000,-179.760},
				{819.893,8.149,10.000,-179.780},
				{820.708,8.149,10.000,-179.800},
				{821.523,8.149,10.000,-179.820},
				{822.338,8.149,10.000,-179.830},
				{823.152,8.149,10.000,-179.850},
				{823.967,8.149,10.000,-179.860},
				{824.782,8.149,10.000,-179.880},
				{825.597,8.149,10.000,-179.890},
				{826.412,8.149,10.000,-179.900},
				{827.227,8.149,10.000,-179.920},
				{828.042,8.149,10.000,-179.930},
				{828.857,8.149,10.000,-179.940},
				{829.671,8.149,10.000,-179.940},
				{830.486,8.149,10.000,-179.950},
				{831.301,8.149,10.000,-179.960},
				{832.116,8.149,10.000,-179.960},
				{832.866,7.497,10.000,-179.970},
				{833.550,6.845,10.000,-179.970},
				{834.170,6.193,10.000,-179.980},
				{834.724,5.541,10.000,-179.980},
				{835.213,4.889,10.000,-179.980},
				{835.636,4.237,10.000,-179.980},
				{835.995,3.585,10.000,-179.980},
				{836.288,2.934,10.000,-179.980},
				{836.516,2.282,10.000,-179.980},
				{836.679,1.630,10.000,-179.980},
				{836.777,0.978,10.000,-179.980}		};

}