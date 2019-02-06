
package frc.robot.pathfollowing;

import static java.util.Arrays.asList;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.trajectory.WaypointSequence.Waypoint;
import com.team319.trajectory.AbstractBobPathCreator;
import com.team319.trajectory.BobPath;
import com.team319.trajectory.SrxTranslatorConfig;

public class BobPathCreator extends AbstractBobPathCreator {

	private static double robotWidthInFeet = 27.6 / 12.0;
	private static double robotLengthInFeet = 33.6 / 12.0;

	// This point and points like it can be used when there are common starting locatons for the robot
	// Remember that paths should be generated from the center point of the robot
	private static Waypoint startingPoint = new Waypoint(robotLengthInFeet / 2.0, 45.5 / 12.0, 0, 0, 0);

	private static Waypoint L2R = new Waypoint(robotLengthInFeet/2.0 + 1.1, 10.1, 0, 0, 0);
	private static Waypoint CMR = new Waypoint(robotLengthInFeet/2.0 + 15.6, 12.6, Math.toRadians(-180), 0, 0);  // only use for reverse
	
	
	private SrxTranslatorConfig config = new SrxTranslatorConfig();

	private BobPathCreator() {
		config.max_acc = 8.0; // Maximum acceleration in FPS
		config.max_vel = 12.0; // Maximum velocity in FPS
		config.wheel_dia_inches = 6.0;
		config.scale_factor = 1.0; // Used to adjust for a gear ratio and or distance tuning
		config.encoder_ticks_per_rev = 128; // Count of ticks on your encoder
		config.robotLength = 33.6; // Robot length in inches, used for drawing the robot
		config.robotWidth = 27.6; // Robot width in inches, used for drawing the robot
		config.highGear = true;
	}

	/**
	 * Use this method to generate team paths. You can create more methods like this one to organize your path, 
	 * just make sure to add the method call to the returned list in getArcs()
	 * @return the list of team paths to generate
	 */
	private List<BobPath> generateTeamArcs() {
		// Create a path with the name of "Example", this will generate a file named ExampleArc
		BobPath exampleArc = new BobPath(config, "Example");
		// Set the first point to the starating point, this be done with any of the addWaypoint methods
		// positive X is forward, positive Y is left, units are in feet and degrees
		exampleArc.addWaypoint(startingPoint);
		// Add the next point that 3 ft forward, and doesn't turn, it also has a max speed of 5 FPS, 
		// it will arrive at this location going 2 FPS
		exampleArc.addWaypointRelative(3, 0, 0, 2, 5);
		// Add the next point to be an additional 5 feet forward and 5 feet to the left with max speed of 2 FPS,
		// it  will arrive at this locaton going 0 FPS 
		exampleArc.addWaypointRelative(5, 5, 0, 0, 2);

		BobPath R2toCMR = new BobPath(config, "R2toCMR");  // L2 Right to cargo middle right
		R2toCMR.addWaypoint(L2R);
		R2toCMR.addWaypointRelative(14.5, 2.5, 0);

		BobPath CMRtoLSR = new BobPath(config, "CMRtoLSR", true);  // Cargo middle right to loading station right
		CMRtoLSR.addWaypoint(CMR);
		CMRtoLSR.addWaypointRelative(2, 1, Math.toRadians(0));
		 
		return asList(exampleArc, R2toCMR, CMRtoLSR); // return asList(path1, path2, path3, ...);
	}
    
	public static void main(String[] args) {
		new BobPathCreator().generatePaths();
	}
	
	@Override
	protected List<BobPath> getArcs() {
		List<BobPath> paths = new ArrayList<>();
		paths.addAll(getConfigArcs());
		paths.addAll(generateTeamArcs());
		return paths;
	}
	
	/**
	 * Generate the configuration arcs, distance, turning, and speed
	 * DistanceScaling - This path will run 3 feet forward and stop. To tune this
	 * adjust the scaling factor until the robot stops at exactly 3 feet.
	 * TurnScaling - This path will run 3 feet forward and 3 feet to the left, this will 
	 * end at 90 degrees. This path can be used when tuning your heading loop for arc mode.
	 * SpeedTesting - This path will drive 3 feet forward and 3 feet to the left at 3 FPS,
	 * then drive another 3 feed forward and 3 feet to the left. This path will end with 
	 * the robot 6 feet to the left of it's starting position facing the oppostite direction.
	 */
	private List<BobPath> getConfigArcs() {
		BobPath distanceScaling = new BobPath(config, "DistanceScaling");
		distanceScaling.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		distanceScaling.addWaypointRelative(3, 0, 0, 0, 3);

		BobPath turnScaling = new BobPath(config, "TurnScaling");
		turnScaling.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		turnScaling.addWaypointRelative(3, 3, 89.99, 0, 3);

		BobPath speedTesting = new BobPath(config, "SpeedTesting");
		speedTesting.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		speedTesting.addWaypointRelative(3, 3, 89.99, 1, 3);
		speedTesting.addWaypointRelative(-3, 3, 89.99, 0, 1);

		return asList(distanceScaling, turnScaling, speedTesting);
	}
}