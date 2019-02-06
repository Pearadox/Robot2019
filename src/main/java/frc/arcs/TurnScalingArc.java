package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class TurnScalingArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (2.00,13.50,0.00)
	// (5.00,16.50,89.99)
	
    public TurnScalingArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public TurnScalingArc(boolean flipped) {
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
				{378.232,23.794,10.000,-88.860},
				{380.546,23.142,10.000,-88.990},
				{382.795,22.491,10.000,-89.100},
				{384.979,21.839,10.000,-89.210},
				{387.097,21.187,10.000,-89.300},
				{389.151,20.535,10.000,-89.390},
				{391.139,19.883,10.000,-89.470},
				{393.062,19.231,10.000,-89.540},
				{394.920,18.579,10.000,-89.600},
				{396.713,17.927,10.000,-89.660},
				{398.440,17.275,10.000,-89.710},
				{400.103,16.623,10.000,-89.750},
				{401.700,15.972,10.000,-89.790},
				{403.232,15.320,10.000,-89.830},
				{404.699,14.668,10.000,-89.860},
				{406.100,14.016,10.000,-89.880},
				{407.437,13.364,10.000,-89.910},
				{408.708,12.712,10.000,-89.920},
				{409.914,12.060,10.000,-89.940},
				{411.055,11.408,10.000,-89.950},
				{412.130,10.756,10.000,-89.960},
				{413.141,10.104,10.000,-89.970},
				{414.086,9.453,10.000,-89.980},
				{414.966,8.801,10.000,-89.980},
				{415.781,8.149,10.000,-89.990},
				{416.531,7.497,10.000,-89.990},
				{417.215,6.845,10.000,-89.990},
				{417.834,6.193,10.000,-89.990},
				{418.389,5.541,10.000,-89.990},
				{418.877,4.889,10.000,-89.990},
				{419.301,4.237,10.000,-89.990},
				{419.660,3.585,10.000,-89.990},
				{419.953,2.934,10.000,-89.990},
				{420.181,2.282,10.000,-89.990},
				{420.344,1.630,10.000,-89.990}		};

}