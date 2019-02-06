package frc.arcs;

import com.team319.follower.SrxMotionProfile;
import com.team319.follower.SrxTrajectory;

public class CMRtoLSRArc extends SrxTrajectory {
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (17.00,12.60,180.00)
	// (1.50,2.60,183.14)
	
    public CMRtoLSRArc() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	
    public CMRtoLSRArc(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}

	public boolean highGear = true;

	double[][] centerPoints = {
				{-0.000,-0.000,10.000,-360.000},
				{-0.065,-0.652,10.000,-360.000},
				{-0.196,-1.304,10.000,-360.000},
				{-0.391,-1.956,10.000,-360.000},
				{-0.652,-2.608,10.000,-360.000},
				{-0.978,-3.259,10.000,-360.000},
				{-1.369,-3.911,10.000,-360.000},
				{-1.825,-4.563,10.000,-360.000},
				{-2.347,-5.215,10.000,-360.000},
				{-2.934,-5.867,10.000,-360.000},
				{-3.585,-6.519,10.000,-360.000},
				{-4.303,-7.171,10.000,-360.000},
				{-5.085,-7.823,10.000,-360.010},
				{-5.932,-8.475,10.000,-360.010},
				{-6.845,-9.127,10.000,-360.010},
				{-7.823,-9.778,10.000,-360.010},
				{-8.866,-10.430,10.000,-360.020},
				{-9.974,-11.082,10.000,-360.020},
				{-11.147,-11.734,10.000,-360.030},
				{-12.386,-12.386,10.000,-360.040},
				{-13.690,-13.038,10.000,-360.040},
				{-15.059,-13.690,10.000,-360.050},
				{-16.493,-14.342,10.000,-360.060},
				{-17.992,-14.994,10.000,-360.070},
				{-19.557,-15.646,10.000,-360.090},
				{-21.187,-16.297,10.000,-360.100},
				{-22.882,-16.949,10.000,-360.120},
				{-24.642,-17.601,10.000,-360.140},
				{-26.467,-18.253,10.000,-360.160},
				{-28.358,-18.905,10.000,-360.180},
				{-30.313,-19.557,10.000,-360.210},
				{-32.334,-20.209,10.000,-360.240},
				{-34.420,-20.861,10.000,-360.270},
				{-36.572,-21.513,10.000,-360.300},
				{-38.788,-22.165,10.000,-360.340},
				{-41.070,-22.816,10.000,-360.380},
				{-43.416,-23.468,10.000,-360.420},
				{-45.828,-24.120,10.000,-360.470},
				{-48.306,-24.772,10.000,-360.520},
				{-50.848,-25.424,10.000,-360.580},
				{-53.456,-26.076,10.000,-360.640},
				{-56.128,-26.728,10.000,-360.700},
				{-58.866,-27.380,10.000,-360.770},
				{-61.670,-28.032,10.000,-360.840},
				{-64.538,-28.684,10.000,-360.920},
				{-67.472,-29.335,10.000,-361.000},
				{-70.470,-29.987,10.000,-361.090},
				{-73.534,-30.639,10.000,-361.190},
				{-76.663,-31.291,10.000,-361.290},
				{-79.858,-31.943,10.000,-361.390},
				{-83.117,-32.595,10.000,-361.510},
				{-86.442,-33.247,10.000,-361.630},
				{-89.832,-33.899,10.000,-361.750},
				{-93.287,-34.551,10.000,-361.890},
				{-96.807,-35.203,10.000,-362.030},
				{-100.392,-35.854,10.000,-362.180},
				{-104.043,-36.506,10.000,-362.340},
				{-107.759,-37.158,10.000,-362.500},
				{-111.540,-37.810,10.000,-362.680},
				{-115.386,-38.462,10.000,-362.860},
				{-119.297,-39.114,10.000,-363.060},
				{-123.274,-39.766,10.000,-363.260},
				{-127.316,-40.418,10.000,-363.470},
				{-131.423,-41.070,10.000,-363.690},
				{-135.595,-41.722,10.000,-363.930},
				{-139.832,-42.373,10.000,-364.170},
				{-144.135,-43.025,10.000,-364.430},
				{-148.503,-43.677,10.000,-364.700},
				{-152.935,-44.329,10.000,-364.980},
				{-157.434,-44.981,10.000,-365.270},
				{-161.997,-45.633,10.000,-365.570},
				{-166.625,-46.285,10.000,-365.890},
				{-171.319,-46.937,10.000,-366.220},
				{-176.078,-47.589,10.000,-366.570},
				{-180.902,-48.240,10.000,-366.930},
				{-185.791,-48.892,10.000,-367.300},
				{-190.746,-49.544,10.000,-367.690},
				{-195.765,-50.196,10.000,-368.100},
				{-200.850,-50.848,10.000,-368.520},
				{-206.000,-51.500,10.000,-368.960},
				{-211.215,-52.152,10.000,-369.410},
				{-216.496,-52.804,10.000,-369.890},
				{-221.841,-53.456,10.000,-370.380},
				{-227.252,-54.108,10.000,-370.880},
				{-232.728,-54.759,10.000,-371.410},
				{-238.269,-55.411,10.000,-371.960},
				{-243.875,-56.063,10.000,-372.520},
				{-249.547,-56.715,10.000,-373.100},
				{-255.284,-57.367,10.000,-373.710},
				{-261.085,-58.019,10.000,-374.330},
				{-266.952,-58.671,10.000,-374.970},
				{-272.885,-59.323,10.000,-375.640},
				{-278.882,-59.975,10.000,-376.320},
				{-284.945,-60.627,10.000,-377.020},
				{-291.073,-61.278,10.000,-377.750},
				{-297.266,-61.930,10.000,-378.490},
				{-303.524,-62.582,10.000,-379.250},
				{-309.847,-63.234,10.000,-380.040},
				{-316.236,-63.886,10.000,-380.840},
				{-322.690,-64.538,10.000,-381.660},
				{-329.209,-65.190,10.000,-382.500},
				{-335.793,-65.842,10.000,-383.350},
				{-342.442,-66.494,10.000,-384.230},
				{-349.157,-67.146,10.000,-385.120},
				{-355.937,-67.797,10.000,-386.020},
				{-362.782,-68.449,10.000,-386.940},
				{-369.692,-69.101,10.000,-387.870},
				{-376.667,-69.753,10.000,-388.810},
				{-383.708,-70.405,10.000,-389.760},
				{-390.813,-71.057,10.000,-390.720},
				{-397.984,-71.709,10.000,-391.690},
				{-405.220,-72.361,10.000,-392.660},
				{-412.521,-73.013,10.000,-393.630},
				{-419.888,-73.665,10.000,-394.610},
				{-427.320,-74.316,10.000,-395.590},
				{-434.816,-74.968,10.000,-396.560},
				{-442.378,-75.620,10.000,-397.540},
				{-450.006,-76.272,10.000,-398.500},
				{-457.698,-76.924,10.000,-399.460},
				{-465.456,-77.576,10.000,-400.410},
				{-473.278,-78.228,10.000,-401.350},
				{-481.166,-78.880,10.000,-402.280},
				{-489.120,-79.532,10.000,-403.200},
				{-497.138,-80.184,10.000,-404.100},
				{-505.221,-80.835,10.000,-404.980},
				{-513.370,-81.487,10.000,-405.850},
				{-521.584,-82.139,10.000,-406.690},
				{-529.863,-82.791,10.000,-407.520},
				{-538.208,-83.443,10.000,-408.330},
				{-546.617,-84.095,10.000,-409.120},
				{-555.092,-84.747,10.000,-409.880},
				{-563.632,-85.399,10.000,-410.620},
				{-572.237,-86.051,10.000,-411.340},
				{-580.907,-86.703,10.000,-412.030},
				{-589.642,-87.354,10.000,-412.700},
				{-598.443,-88.006,10.000,-413.340},
				{-607.309,-88.658,10.000,-413.960},
				{-616.240,-89.310,10.000,-414.550},
				{-625.236,-89.962,10.000,-415.120},
				{-634.297,-90.614,10.000,-415.660},
				{-643.424,-91.266,10.000,-416.170},
				{-652.616,-91.918,10.000,-416.660},
				{-661.873,-92.570,10.000,-417.120},
				{-671.195,-93.222,10.000,-417.560},
				{-680.582,-93.873,10.000,-417.970},
				{-690.035,-94.525,10.000,-418.350},
				{-699.552,-95.177,10.000,-418.700},
				{-709.135,-95.829,10.000,-419.030},
				{-718.783,-96.481,10.000,-419.340},
				{-728.497,-97.133,10.000,-419.610},
				{-738.275,-97.785,10.000,-419.860},
				{-748.054,-97.785,10.000,-420.080},
				{-757.832,-97.785,10.000,-420.270},
				{-767.611,-97.785,10.000,-420.430},
				{-777.389,-97.785,10.000,-420.570},
				{-787.168,-97.785,10.000,-420.680},
				{-796.946,-97.785,10.000,-420.760},
				{-806.725,-97.785,10.000,-420.810},
				{-816.503,-97.785,10.000,-420.840},
				{-826.282,-97.785,10.000,-420.840},
				{-836.060,-97.785,10.000,-420.810},
				{-845.838,-97.785,10.000,-420.760},
				{-855.617,-97.785,10.000,-420.680},
				{-865.395,-97.785,10.000,-420.570},
				{-875.174,-97.785,10.000,-420.440},
				{-884.952,-97.785,10.000,-420.270},
				{-894.731,-97.785,10.000,-420.080},
				{-904.444,-97.133,10.000,-419.870},
				{-914.092,-96.481,10.000,-419.630},
				{-923.675,-95.829,10.000,-419.360},
				{-933.193,-95.177,10.000,-419.060},
				{-942.645,-94.525,10.000,-418.740},
				{-952.033,-93.873,10.000,-418.400},
				{-961.355,-93.222,10.000,-418.030},
				{-970.612,-92.570,10.000,-417.630},
				{-979.804,-91.918,10.000,-417.210},
				{-988.930,-91.266,10.000,-416.760},
				{-997.992,-90.614,10.000,-416.290},
				{-1006.988,-89.962,10.000,-415.800},
				{-1015.919,-89.310,10.000,-415.280},
				{-1024.785,-88.658,10.000,-414.730},
				{-1033.585,-88.006,10.000,-414.160},
				{-1042.321,-87.354,10.000,-413.570},
				{-1050.991,-86.703,10.000,-412.950},
				{-1059.596,-86.051,10.000,-412.310},
				{-1068.136,-85.399,10.000,-411.650},
				{-1076.611,-84.747,10.000,-410.970},
				{-1085.020,-84.095,10.000,-410.260},
				{-1093.364,-83.443,10.000,-409.540},
				{-1101.644,-82.791,10.000,-408.790},
				{-1109.857,-82.139,10.000,-408.020},
				{-1118.006,-81.487,10.000,-407.230},
				{-1126.090,-80.835,10.000,-406.430},
				{-1134.108,-80.184,10.000,-405.610},
				{-1142.061,-79.532,10.000,-404.770},
				{-1149.949,-78.880,10.000,-403.920},
				{-1157.772,-78.228,10.000,-403.050},
				{-1165.530,-77.576,10.000,-402.180},
				{-1173.222,-76.924,10.000,-401.290},
				{-1180.849,-76.272,10.000,-400.390},
				{-1188.411,-75.620,10.000,-399.480},
				{-1195.908,-74.968,10.000,-398.570},
				{-1203.340,-74.316,10.000,-397.660},
				{-1210.706,-73.665,10.000,-396.740},
				{-1218.007,-73.013,10.000,-395.810},
				{-1225.244,-72.361,10.000,-394.890},
				{-1232.414,-71.709,10.000,-393.970},
				{-1239.520,-71.057,10.000,-393.050},
				{-1246.561,-70.405,10.000,-392.140},
				{-1253.536,-69.753,10.000,-391.230},
				{-1260.446,-69.101,10.000,-390.330},
				{-1267.291,-68.449,10.000,-389.440},
				{-1274.071,-67.797,10.000,-388.560},
				{-1280.785,-67.146,10.000,-387.690},
				{-1287.435,-66.494,10.000,-386.830},
				{-1294.019,-65.842,10.000,-385.990},
				{-1300.538,-65.190,10.000,-385.160},
				{-1306.992,-64.538,10.000,-384.340},
				{-1313.380,-63.886,10.000,-383.540},
				{-1319.704,-63.234,10.000,-382.760},
				{-1325.962,-62.582,10.000,-382.000},
				{-1332.155,-61.930,10.000,-381.250},
				{-1338.283,-61.278,10.000,-380.520},
				{-1344.345,-60.627,10.000,-379.810},
				{-1350.343,-59.975,10.000,-379.120},
				{-1356.275,-59.323,10.000,-378.440},
				{-1362.142,-58.671,10.000,-377.790},
				{-1367.944,-58.019,10.000,-377.150},
				{-1373.681,-57.367,10.000,-376.540},
				{-1379.352,-56.715,10.000,-375.940},
				{-1384.959,-56.063,10.000,-375.360},
				{-1390.500,-55.411,10.000,-374.800},
				{-1395.976,-54.759,10.000,-374.260},
				{-1401.387,-54.108,10.000,-373.730},
				{-1406.732,-53.456,10.000,-373.230},
				{-1412.012,-52.804,10.000,-372.740},
				{-1417.228,-52.152,10.000,-372.270},
				{-1422.378,-51.500,10.000,-371.810},
				{-1427.462,-50.848,10.000,-371.380},
				{-1432.482,-50.196,10.000,-370.950},
				{-1437.437,-49.544,10.000,-370.550},
				{-1442.326,-48.892,10.000,-370.160},
				{-1447.150,-48.240,10.000,-369.780},
				{-1451.909,-47.589,10.000,-369.420},
				{-1456.602,-46.937,10.000,-369.080},
				{-1461.231,-46.285,10.000,-368.750},
				{-1465.794,-45.633,10.000,-368.430},
				{-1470.292,-44.981,10.000,-368.130},
				{-1474.725,-44.329,10.000,-367.840},
				{-1479.093,-43.677,10.000,-367.560},
				{-1483.395,-43.025,10.000,-367.290},
				{-1487.633,-42.373,10.000,-367.040},
				{-1491.805,-41.722,10.000,-366.800},
				{-1495.912,-41.070,10.000,-366.560},
				{-1499.954,-40.418,10.000,-366.340},
				{-1503.930,-39.766,10.000,-366.130},
				{-1507.842,-39.114,10.000,-365.930},
				{-1511.688,-38.462,10.000,-365.740},
				{-1515.469,-37.810,10.000,-365.560},
				{-1519.185,-37.158,10.000,-365.390},
				{-1522.835,-36.506,10.000,-365.230},
				{-1526.421,-35.854,10.000,-365.080},
				{-1529.941,-35.203,10.000,-364.930},
				{-1533.396,-34.551,10.000,-364.790},
				{-1536.786,-33.899,10.000,-364.660},
				{-1540.111,-33.247,10.000,-364.540},
				{-1543.370,-32.595,10.000,-364.430},
				{-1546.564,-31.943,10.000,-364.320},
				{-1549.693,-31.291,10.000,-364.220},
				{-1552.757,-30.639,10.000,-364.120},
				{-1555.756,-29.987,10.000,-364.030},
				{-1558.690,-29.335,10.000,-363.950},
				{-1561.558,-28.684,10.000,-363.870},
				{-1564.361,-28.032,10.000,-363.800},
				{-1567.099,-27.380,10.000,-363.740},
				{-1569.772,-26.728,10.000,-363.670},
				{-1572.380,-26.076,10.000,-363.620},
				{-1574.922,-25.424,10.000,-363.560},
				{-1577.399,-24.772,10.000,-363.520},
				{-1579.811,-24.120,10.000,-363.470},
				{-1582.158,-23.468,10.000,-363.430},
				{-1584.440,-22.816,10.000,-363.390},
				{-1586.656,-22.165,10.000,-363.360},
				{-1588.807,-21.513,10.000,-363.330},
				{-1590.893,-20.861,10.000,-363.300},
				{-1592.914,-20.209,10.000,-363.280},
				{-1594.870,-19.557,10.000,-363.260},
				{-1596.761,-18.905,10.000,-363.240},
				{-1598.586,-18.253,10.000,-363.220},
				{-1600.346,-17.601,10.000,-363.200},
				{-1602.041,-16.949,10.000,-363.190},
				{-1603.671,-16.297,10.000,-363.180},
				{-1605.235,-15.646,10.000,-363.170},
				{-1606.735,-14.994,10.000,-363.160},
				{-1608.169,-14.342,10.000,-363.160},
				{-1609.538,-13.690,10.000,-363.150},
				{-1610.842,-13.038,10.000,-363.150},
				{-1612.080,-12.386,10.000,-363.150},
				{-1613.254,-11.734,10.000,-363.140},
				{-1614.362,-11.082,10.000,-363.140},
				{-1615.405,-10.430,10.000,-363.140},
				{-1616.383,-9.778,10.000,-363.140},
				{-1617.295,-9.127,10.000,-363.140},
				{-1618.143,-8.475,10.000,-363.140},
				{-1618.925,-7.823,10.000,-363.140},
				{-1619.642,-7.171,10.000,-363.140},
				{-1620.294,-6.519,10.000,-363.140},
				{-1620.881,-5.867,10.000,-363.140},
				{-1621.402,-5.215,10.000,-363.140},
				{-1621.859,-4.563,10.000,-363.140},
				{-1622.250,-3.911,10.000,-363.140},
				{-1622.576,-3.259,10.000,-363.140},
				{-1622.836,-2.608,10.000,-363.140},
				{-1623.032,-1.956,10.000,-363.140},
				{-1623.162,-1.304,10.000,-363.140}		};

}