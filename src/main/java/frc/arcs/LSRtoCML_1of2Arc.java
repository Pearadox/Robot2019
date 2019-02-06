package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class LSRtoCML_1of2Arc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (1.70,2.30,0.00)
	// (16.70,4.30,45.00)
	
    public LSRtoCML_1of2Arc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public LSRtoCML_1of2Arc(boolean flipped) {
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
				{2.934,5.867,10.000,0.000},
				{3.585,6.519,10.000,0.000},
				{4.303,7.171,10.000,0.000},
				{5.085,7.823,10.000,0.010},
				{5.932,8.475,10.000,0.010},
				{6.845,9.127,10.000,0.010},
				{7.823,9.778,10.000,0.010},
				{8.866,10.430,10.000,0.020},
				{9.974,11.082,10.000,0.020},
				{11.147,11.734,10.000,0.030},
				{12.386,12.386,10.000,0.040},
				{13.690,13.038,10.000,0.040},
				{15.059,13.690,10.000,0.050},
				{16.493,14.342,10.000,0.060},
				{17.992,14.994,10.000,0.080},
				{19.557,15.646,10.000,0.090},
				{21.187,16.297,10.000,0.110},
				{22.882,16.949,10.000,0.120},
				{24.642,17.601,10.000,0.140},
				{26.467,18.253,10.000,0.160},
				{28.358,18.905,10.000,0.190},
				{30.313,19.557,10.000,0.210},
				{32.334,20.209,10.000,0.240},
				{34.420,20.861,10.000,0.270},
				{36.572,21.513,10.000,0.310},
				{38.788,22.165,10.000,0.340},
				{41.070,22.816,10.000,0.380},
				{43.416,23.468,10.000,0.420},
				{45.828,24.120,10.000,0.470},
				{48.306,24.772,10.000,0.520},
				{50.848,25.424,10.000,0.570},
				{53.456,26.076,10.000,0.630},
				{56.128,26.728,10.000,0.690},
				{58.866,27.380,10.000,0.750},
				{61.670,28.032,10.000,0.820},
				{64.538,28.684,10.000,0.890},
				{67.472,29.335,10.000,0.970},
				{70.470,29.987,10.000,1.050},
				{73.534,30.639,10.000,1.130},
				{76.663,31.291,10.000,1.220},
				{79.858,31.943,10.000,1.320},
				{83.117,32.595,10.000,1.420},
				{86.442,33.247,10.000,1.520},
				{89.832,33.899,10.000,1.630},
				{93.287,34.551,10.000,1.740},
				{96.807,35.203,10.000,1.860},
				{100.392,35.854,10.000,1.980},
				{104.043,36.506,10.000,2.110},
				{107.759,37.158,10.000,2.240},
				{111.540,37.810,10.000,2.380},
				{115.386,38.462,10.000,2.520},
				{119.297,39.114,10.000,2.670},
				{123.274,39.766,10.000,2.820},
				{127.316,40.418,10.000,2.980},
				{131.423,41.070,10.000,3.140},
				{135.595,41.722,10.000,3.310},
				{139.832,42.373,10.000,3.480},
				{144.135,43.025,10.000,3.660},
				{148.503,43.677,10.000,3.840},
				{152.935,44.329,10.000,4.020},
				{157.434,44.981,10.000,4.210},
				{161.997,45.633,10.000,4.400},
				{166.625,46.285,10.000,4.600},
				{171.319,46.937,10.000,4.800},
				{176.078,47.589,10.000,5.010},
				{180.902,48.240,10.000,5.210},
				{185.791,48.892,10.000,5.420},
				{190.746,49.544,10.000,5.640},
				{195.765,50.196,10.000,5.850},
				{200.850,50.848,10.000,6.070},
				{206.000,51.500,10.000,6.290},
				{211.215,52.152,10.000,6.510},
				{216.496,52.804,10.000,6.740},
				{221.841,53.456,10.000,6.960},
				{227.252,54.108,10.000,7.190},
				{232.728,54.759,10.000,7.420},
				{238.269,55.411,10.000,7.650},
				{243.875,56.063,10.000,7.870},
				{249.547,56.715,10.000,8.100},
				{255.284,57.367,10.000,8.330},
				{261.085,58.019,10.000,8.550},
				{266.952,58.671,10.000,8.780},
				{272.885,59.323,10.000,9.000},
				{278.882,59.975,10.000,9.220},
				{284.945,60.627,10.000,9.440},
				{291.073,61.278,10.000,9.650},
				{297.266,61.930,10.000,9.860},
				{303.524,62.582,10.000,10.070},
				{309.847,63.234,10.000,10.270},
				{316.236,63.886,10.000,10.470},
				{322.690,64.538,10.000,10.660},
				{329.209,65.190,10.000,10.850},
				{335.793,65.842,10.000,11.030},
				{342.442,66.494,10.000,11.210},
				{349.157,67.146,10.000,11.380},
				{355.937,67.797,10.000,11.540},
				{362.782,68.449,10.000,11.690},
				{369.692,69.101,10.000,11.840},
				{376.667,69.753,10.000,11.970},
				{383.708,70.405,10.000,12.100},
				{390.813,71.057,10.000,12.220},
				{397.984,71.709,10.000,12.320},
				{405.220,72.361,10.000,12.420},
				{412.521,73.013,10.000,12.500},
				{419.888,73.665,10.000,12.580},
				{427.320,74.316,10.000,12.640},
				{434.816,74.968,10.000,12.690},
				{442.378,75.620,10.000,12.720},
				{450.006,76.272,10.000,12.740},
				{457.698,76.924,10.000,12.750},
				{465.456,77.576,10.000,12.740},
				{473.278,78.228,10.000,12.710},
				{481.166,78.880,10.000,12.670},
				{489.120,79.532,10.000,12.610},
				{497.138,80.184,10.000,12.540},
				{505.221,80.835,10.000,12.440},
				{513.370,81.487,10.000,12.330},
				{521.584,82.139,10.000,12.190},
				{529.863,82.791,10.000,12.040},
				{538.208,83.443,10.000,11.860},
				{546.617,84.095,10.000,11.670},
				{555.092,84.747,10.000,11.450},
				{563.632,85.399,10.000,11.200},
				{572.237,86.051,10.000,10.930},
				{580.907,86.703,10.000,10.640},
				{589.642,87.354,10.000,10.320},
				{598.443,88.006,10.000,9.970},
				{607.309,88.658,10.000,9.590},
				{616.240,89.310,10.000,9.180},
				{625.236,89.962,10.000,8.750},
				{634.297,90.614,10.000,8.280},
				{643.424,91.266,10.000,7.780},
				{652.616,91.918,10.000,7.240},
				{661.871,92.557,10.000,6.670},
				{671.062,91.906,10.000,6.080},
				{680.187,91.254,10.000,5.460},
				{689.248,90.602,10.000,4.810},
				{698.243,89.950,10.000,4.140},
				{707.172,89.298,10.000,3.440},
				{716.037,88.646,10.000,2.720},
				{724.836,87.994,10.000,1.980},
				{733.571,87.342,10.000,1.210},
				{742.240,86.690,10.000,0.430},
				{750.843,86.038,10.000,-0.380},
				{759.382,85.387,10.000,-1.210},
				{767.856,84.735,10.000,-2.050},
				{776.264,84.083,10.000,-2.910},
				{784.607,83.431,10.000,-3.780},
				{792.885,82.779,10.000,-4.670},
				{801.098,82.127,10.000,-5.570},
				{809.245,81.475,10.000,-6.480},
				{817.327,80.823,10.000,-7.400},
				{825.345,80.171,10.000,-8.320},
				{833.297,79.519,10.000,-9.250},
				{841.183,78.868,10.000,-10.180},
				{849.005,78.216,10.000,-11.120},
				{856.761,77.564,10.000,-12.050},
				{864.452,76.912,10.000,-12.980},
				{872.078,76.260,10.000,-13.910},
				{879.639,75.608,10.000,-14.830},
				{887.135,74.956,10.000,-15.750},
				{894.565,74.304,10.000,-16.660},
				{901.931,73.652,10.000,-17.560},
				{909.231,73.000,10.000,-18.440},
				{916.465,72.349,10.000,-19.320},
				{923.635,71.697,10.000,-20.180},
				{930.740,71.045,10.000,-21.030},
				{937.779,70.393,10.000,-21.860},
				{944.753,69.741,10.000,-22.680},
				{951.662,69.089,10.000,-23.480},
				{958.506,68.437,10.000,-24.260},
				{965.284,67.785,10.000,-25.030},
				{971.997,67.133,10.000,-25.780},
				{978.646,66.482,10.000,-26.510},
				{985.229,65.830,10.000,-27.220},
				{991.746,65.178,10.000,-27.910},
				{998.199,64.526,10.000,-28.580},
				{1004.586,63.874,10.000,-29.240},
				{1010.909,63.222,10.000,-29.870},
				{1017.166,62.570,10.000,-30.490},
				{1023.357,61.918,10.000,-31.090},
				{1029.484,61.266,10.000,-31.670},
				{1035.545,60.614,10.000,-32.230},
				{1041.542,59.963,10.000,-32.780},
				{1047.473,59.311,10.000,-33.310},
				{1053.339,58.659,10.000,-33.810},
				{1059.139,58.007,10.000,-34.310},
				{1064.875,57.355,10.000,-34.780},
				{1070.545,56.703,10.000,-35.240},
				{1076.150,56.051,10.000,-35.690},
				{1081.690,55.399,10.000,-36.110},
				{1087.165,54.747,10.000,-36.530},
				{1092.574,54.095,10.000,-36.920},
				{1097.919,53.444,10.000,-37.310},
				{1103.198,52.792,10.000,-37.680},
				{1108.412,52.140,10.000,-38.030},
				{1113.561,51.488,10.000,-38.370},
				{1118.644,50.836,10.000,-38.700},
				{1123.663,50.184,10.000,-39.020},
				{1128.616,49.532,10.000,-39.320},
				{1133.504,48.880,10.000,-39.610},
				{1138.327,48.228,10.000,-39.890},
				{1143.084,47.576,10.000,-40.160},
				{1147.777,46.925,10.000,-40.420},
				{1152.404,46.273,10.000,-40.670},
				{1156.966,45.621,10.000,-40.900},
				{1161.463,44.969,10.000,-41.130},
				{1165.895,44.317,10.000,-41.350},
				{1170.261,43.665,10.000,-41.560},
				{1174.563,43.013,10.000,-41.760},
				{1178.799,42.361,10.000,-41.950},
				{1182.970,41.709,10.000,-42.130},
				{1187.075,41.057,10.000,-42.300},
				{1191.116,40.406,10.000,-42.470},
				{1195.091,39.754,10.000,-42.620},
				{1199.001,39.102,10.000,-42.770},
				{1202.846,38.450,10.000,-42.920},
				{1206.626,37.798,10.000,-43.050},
				{1210.341,37.146,10.000,-43.180},
				{1213.990,36.494,10.000,-43.300},
				{1217.575,35.842,10.000,-43.420},
				{1221.094,35.190,10.000,-43.530},
				{1224.547,34.538,10.000,-43.640},
				{1227.936,33.887,10.000,-43.730},
				{1231.260,33.235,10.000,-43.830},
				{1234.518,32.583,10.000,-43.920},
				{1237.711,31.931,10.000,-44.000},
				{1240.839,31.279,10.000,-44.080},
				{1243.902,30.627,10.000,-44.150},
				{1246.899,29.975,10.000,-44.220},
				{1249.831,29.323,10.000,-44.290},
				{1252.698,28.671,10.000,-44.350},
				{1255.500,28.019,10.000,-44.400},
				{1258.237,27.368,10.000,-44.460},
				{1260.909,26.716,10.000,-44.510},
				{1263.515,26.064,10.000,-44.550},
				{1266.056,25.412,10.000,-44.600},
				{1268.532,24.760,10.000,-44.640},
				{1270.943,24.108,10.000,-44.670},
				{1273.289,23.456,10.000,-44.710},
				{1275.569,22.804,10.000,-44.740},
				{1277.784,22.152,10.000,-44.770},
				{1279.934,21.501,10.000,-44.790},
				{1282.019,20.849,10.000,-44.820},
				{1284.039,20.197,10.000,-44.840},
				{1285.994,19.545,10.000,-44.860},
				{1287.883,18.893,10.000,-44.880},
				{1289.707,18.241,10.000,-44.890},
				{1291.466,17.589,10.000,-44.910},
				{1293.160,16.937,10.000,-44.920},
				{1294.788,16.285,10.000,-44.930},
				{1296.351,15.633,10.000,-44.940},
				{1297.850,14.982,10.000,-44.950},
				{1299.283,14.330,10.000,-44.960},
				{1300.650,13.678,10.000,-44.970},
				{1301.953,13.026,10.000,-44.970},
				{1303.190,12.374,10.000,-44.980},
				{1304.362,11.722,10.000,-44.980},
				{1305.469,11.070,10.000,-44.990},
				{1306.511,10.418,10.000,-44.990},
				{1307.488,9.766,10.000,-44.990},
				{1308.399,9.114,10.000,-44.990},
				{1309.246,8.463,10.000,-45.000},
				{1310.027,7.811,10.000,-45.000},
				{1310.743,7.159,10.000,-45.000},
				{1311.393,6.507,10.000,-45.000},
				{1311.979,5.855,10.000,-45.000},
				{1312.499,5.203,10.000,-45.000},
				{1312.954,4.551,10.000,-45.000},
				{1313.344,3.899,10.000,-45.000},
				{1313.669,3.247,10.000,-45.000},
				{1313.928,2.595,10.000,-45.000},
				{1314.123,1.944,10.000,-45.000},
				{1314.252,1.292,10.000,-45.000}		};

}