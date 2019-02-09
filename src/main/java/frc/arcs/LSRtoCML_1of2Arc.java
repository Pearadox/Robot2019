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
				{16.271,25.033,10.000,0.010},
				{18.983,27.119,10.000,0.010},
				{21.904,29.205,10.000,0.010},
				{25.033,31.291,10.000,0.010},
				{28.371,33.377,10.000,0.020},
				{31.917,35.463,10.000,0.020},
				{35.672,37.549,10.000,0.030},
				{39.635,39.635,10.000,0.040},
				{43.808,41.722,10.000,0.040},
				{48.188,43.808,10.000,0.050},
				{52.778,45.894,10.000,0.060},
				{57.576,47.980,10.000,0.080},
				{62.582,50.066,10.000,0.090},
				{67.797,52.152,10.000,0.110},
				{73.221,54.238,10.000,0.120},
				{78.854,56.324,10.000,0.140},
				{84.695,58.410,10.000,0.160},
				{90.744,60.496,10.000,0.190},
				{97.003,62.582,10.000,0.210},
				{103.469,64.668,10.000,0.240},
				{110.145,66.754,10.000,0.270},
				{117.029,68.840,10.000,0.310},
				{124.122,70.927,10.000,0.340},
				{131.423,73.013,10.000,0.380},
				{138.933,75.099,10.000,0.420},
				{146.651,77.185,10.000,0.470},
				{154.578,79.271,10.000,0.520},
				{162.714,81.357,10.000,0.570},
				{171.058,83.443,10.000,0.630},
				{179.611,85.529,10.000,0.690},
				{188.373,87.615,10.000,0.750},
				{197.343,89.701,10.000,0.820},
				{206.521,91.787,10.000,0.890},
				{215.909,93.873,10.000,0.970},
				{225.505,95.959,10.000,1.050},
				{235.309,98.046,10.000,1.130},
				{245.322,100.132,10.000,1.220},
				{255.544,102.218,10.000,1.320},
				{265.975,104.304,10.000,1.420},
				{276.614,106.390,10.000,1.520},
				{287.461,108.476,10.000,1.630},
				{298.517,110.562,10.000,1.740},
				{309.782,112.648,10.000,1.860},
				{321.256,114.734,10.000,1.980},
				{332.938,116.820,10.000,2.110},
				{344.828,118.906,10.000,2.240},
				{356.928,120.992,10.000,2.380},
				{369.235,123.078,10.000,2.520},
				{381.752,125.165,10.000,2.670},
				{394.477,127.251,10.000,2.820},
				{407.411,129.337,10.000,2.980},
				{420.553,131.423,10.000,3.140},
				{433.904,133.509,10.000,3.310},
				{447.463,135.595,10.000,3.480},
				{461.231,137.681,10.000,3.660},
				{475.208,139.767,10.000,3.840},
				{489.393,141.853,10.000,4.020},
				{503.787,143.939,10.000,4.210},
				{518.390,146.025,10.000,4.400},
				{533.201,148.111,10.000,4.600},
				{548.221,150.197,10.000,4.800},
				{563.449,152.284,10.000,5.010},
				{578.886,154.370,10.000,5.210},
				{594.532,156.456,10.000,5.420},
				{610.386,158.542,10.000,5.640},
				{626.449,160.628,10.000,5.850},
				{642.720,162.714,10.000,6.070},
				{659.200,164.800,10.000,6.290},
				{675.889,166.886,10.000,6.510},
				{692.786,168.972,10.000,6.740},
				{709.892,171.058,10.000,6.960},
				{727.206,173.144,10.000,7.190},
				{744.729,175.230,10.000,7.420},
				{762.461,177.316,10.000,7.650},
				{780.401,179.403,10.000,7.870},
				{798.550,181.489,10.000,8.100},
				{816.907,183.575,10.000,8.330},
				{835.473,185.661,10.000,8.550},
				{854.248,187.747,10.000,8.780},
				{873.231,189.833,10.000,9.000},
				{892.423,191.919,10.000,9.220},
				{911.824,194.005,10.000,9.440},
				{931.433,196.091,10.000,9.650},
				{951.251,198.177,10.000,9.860},
				{971.277,200.263,10.000,10.070},
				{991.512,202.349,10.000,10.270},
				{1011.955,204.435,10.000,10.470},
				{1032.607,206.521,10.000,10.660},
				{1053.468,208.608,10.000,10.850},
				{1074.538,210.694,10.000,11.030},
				{1095.816,212.780,10.000,11.210},
				{1117.302,214.866,10.000,11.380},
				{1138.997,216.952,10.000,11.540},
				{1160.901,219.038,10.000,11.690},
				{1183.014,221.124,10.000,11.840},
				{1205.335,223.210,10.000,11.970},
				{1227.864,225.296,10.000,12.100},
				{1250.602,227.382,10.000,12.220},
				{1273.549,229.468,10.000,12.320},
				{1296.705,231.554,10.000,12.420},
				{1320.069,233.640,10.000,12.500},
				{1343.641,235.727,10.000,12.580},
				{1367.423,237.813,10.000,12.640},
				{1391.412,239.899,10.000,12.690},
				{1415.611,241.985,10.000,12.720},
				{1440.018,244.071,10.000,12.740},
				{1464.634,246.157,10.000,12.750},
				{1489.458,248.243,10.000,12.740},
				{1514.491,250.329,10.000,12.710},
				{1539.732,252.415,10.000,12.670},
				{1565.183,254.501,10.000,12.610},
				{1590.841,256.587,10.000,12.540},
				{1616.709,258.673,10.000,12.440},
				{1642.785,260.759,10.000,12.330},
				{1669.069,262.846,10.000,12.190},
				{1695.562,264.932,10.000,12.040},
				{1722.264,267.018,10.000,11.860},
				{1749.174,269.104,10.000,11.670},
				{1776.293,271.190,10.000,11.450},
				{1803.621,273.276,10.000,11.200},
				{1831.157,275.362,10.000,10.930},
				{1858.902,277.448,10.000,10.640},
				{1886.855,279.534,10.000,10.320},
				{1915.017,281.620,10.000,9.970},
				{1943.388,283.706,10.000,9.590},
				{1971.967,285.792,10.000,9.180},
				{2000.755,287.878,10.000,8.750},
				{2029.752,289.965,10.000,8.280},
				{2058.957,292.051,10.000,7.780},
				{2088.370,294.137,10.000,7.240},
				{2117.989,296.184,10.000,6.670},
				{2147.399,294.098,10.000,6.080},
				{2176.600,292.012,10.000,5.460},
				{2205.592,289.926,10.000,4.810},
				{2234.376,287.840,10.000,4.140},
				{2262.952,285.753,10.000,3.440},
				{2291.318,283.667,10.000,2.720},
				{2319.476,281.581,10.000,1.980},
				{2347.426,279.495,10.000,1.210},
				{2375.167,277.409,10.000,0.430},
				{2402.699,275.323,10.000,-0.380},
				{2430.023,273.237,10.000,-1.210},
				{2457.138,271.151,10.000,-2.050},
				{2484.044,269.065,10.000,-2.910},
				{2510.742,266.979,10.000,-3.780},
				{2537.232,264.893,10.000,-4.670},
				{2563.512,262.807,10.000,-5.570},
				{2589.584,260.721,10.000,-6.480},
				{2615.448,258.634,10.000,-7.400},
				{2641.103,256.548,10.000,-8.320},
				{2666.549,254.462,10.000,-9.250},
				{2691.786,252.376,10.000,-10.180},
				{2716.815,250.290,10.000,-11.120},
				{2741.636,248.204,10.000,-12.050},
				{2766.248,246.118,10.000,-12.980},
				{2790.651,244.032,10.000,-13.910},
				{2814.845,241.946,10.000,-14.830},
				{2838.831,239.860,10.000,-15.750},
				{2862.609,237.774,10.000,-16.660},
				{2886.178,235.688,10.000,-17.560},
				{2909.538,233.602,10.000,-18.440},
				{2932.689,231.516,10.000,-19.320},
				{2955.632,229.429,10.000,-20.180},
				{2978.367,227.343,10.000,-21.030},
				{3000.892,225.257,10.000,-21.860},
				{3023.209,223.171,10.000,-22.680},
				{3045.318,221.085,10.000,-23.480},
				{3067.218,218.999,10.000,-24.260},
				{3088.909,216.913,10.000,-25.030},
				{3110.392,214.827,10.000,-25.780},
				{3131.666,212.741,10.000,-26.510},
				{3152.731,210.655,10.000,-27.220},
				{3173.588,208.569,10.000,-27.910},
				{3194.237,206.483,10.000,-28.580},
				{3214.676,204.397,10.000,-29.240},
				{3234.907,202.310,10.000,-29.870},
				{3254.930,200.224,10.000,-30.490},
				{3274.744,198.138,10.000,-31.090},
				{3294.349,196.052,10.000,-31.670},
				{3313.745,193.966,10.000,-32.230},
				{3332.933,191.880,10.000,-32.780},
				{3351.913,189.794,10.000,-33.310},
				{3370.684,187.708,10.000,-33.810},
				{3389.246,185.622,10.000,-34.310},
				{3407.599,183.536,10.000,-34.780},
				{3425.744,181.450,10.000,-35.240},
				{3443.681,179.364,10.000,-35.690},
				{3461.408,177.278,10.000,-36.110},
				{3478.928,175.191,10.000,-36.530},
				{3496.238,173.105,10.000,-36.920},
				{3513.340,171.019,10.000,-37.310},
				{3530.233,168.933,10.000,-37.680},
				{3546.918,166.847,10.000,-38.030},
				{3563.394,164.761,10.000,-38.370},
				{3579.662,162.675,10.000,-38.700},
				{3595.721,160.589,10.000,-39.020},
				{3611.571,158.503,10.000,-39.320},
				{3627.213,156.417,10.000,-39.610},
				{3642.646,154.331,10.000,-39.890},
				{3657.870,152.245,10.000,-40.160},
				{3672.886,150.159,10.000,-40.420},
				{3687.693,148.072,10.000,-40.670},
				{3702.292,145.986,10.000,-40.900},
				{3716.682,143.900,10.000,-41.130},
				{3730.863,141.814,10.000,-41.350},
				{3744.836,139.728,10.000,-41.560},
				{3758.600,137.642,10.000,-41.760},
				{3772.156,135.556,10.000,-41.950},
				{3785.503,133.470,10.000,-42.130},
				{3798.641,131.384,10.000,-42.300},
				{3811.571,129.298,10.000,-42.470},
				{3824.292,127.212,10.000,-42.620},
				{3836.805,125.126,10.000,-42.770},
				{3849.109,123.040,10.000,-42.920},
				{3861.204,120.953,10.000,-43.050},
				{3873.091,118.867,10.000,-43.180},
				{3884.769,116.781,10.000,-43.300},
				{3896.238,114.695,10.000,-43.420},
				{3907.499,112.609,10.000,-43.530},
				{3918.552,110.523,10.000,-43.640},
				{3929.395,108.437,10.000,-43.730},
				{3940.031,106.351,10.000,-43.830},
				{3950.457,104.265,10.000,-43.920},
				{3960.675,102.179,10.000,-44.000},
				{3970.684,100.093,10.000,-44.080},
				{3980.485,98.007,10.000,-44.150},
				{3990.077,95.921,10.000,-44.220},
				{3999.460,93.835,10.000,-44.290},
				{4008.635,91.748,10.000,-44.350},
				{4017.601,89.662,10.000,-44.400},
				{4026.359,87.576,10.000,-44.460},
				{4034.908,85.490,10.000,-44.510},
				{4043.248,83.404,10.000,-44.550},
				{4051.380,81.318,10.000,-44.600},
				{4059.303,79.232,10.000,-44.640},
				{4067.018,77.146,10.000,-44.670},
				{4074.524,75.060,10.000,-44.710},
				{4081.821,72.974,10.000,-44.740},
				{4088.910,70.888,10.000,-44.770},
				{4095.790,68.802,10.000,-44.790},
				{4102.462,66.716,10.000,-44.820},
				{4108.925,64.629,10.000,-44.840},
				{4115.179,62.543,10.000,-44.860},
				{4121.225,60.457,10.000,-44.880},
				{4127.062,58.371,10.000,-44.890},
				{4132.691,56.285,10.000,-44.910},
				{4138.110,54.199,10.000,-44.920},
				{4143.322,52.113,10.000,-44.930},
				{4148.324,50.027,10.000,-44.940},
				{4153.119,47.941,10.000,-44.950},
				{4157.704,45.855,10.000,-44.960},
				{4162.081,43.769,10.000,-44.970},
				{4166.249,41.683,10.000,-44.970},
				{4170.209,39.597,10.000,-44.980},
				{4173.960,37.510,10.000,-44.980},
				{4177.502,35.424,10.000,-44.990},
				{4180.836,33.338,10.000,-44.990},
				{4183.961,31.252,10.000,-44.990},
				{4186.878,29.166,10.000,-44.990},
				{4189.586,27.080,10.000,-45.000},
				{4192.085,24.994,10.000,-45.000},
				{4194.376,22.908,10.000,-45.000},
				{4196.458,20.822,10.000,-45.000},
				{4198.332,18.736,10.000,-45.000},
				{4199.997,16.650,10.000,-45.000},
				{4201.453,14.564,10.000,-45.000},
				{4202.701,12.478,10.000,-45.000},
				{4203.740,10.391,10.000,-45.000},
				{4204.571,8.305,10.000,-45.000},
				{4205.193,6.219,10.000,-45.000},
				{4205.606,4.133,10.000,-45.000}		};

}