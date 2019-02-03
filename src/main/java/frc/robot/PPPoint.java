// pure pursuit trajectory point

package frc.robot;

public class PPPoint {

    double x, y, distance, curvature;

    public PPPoint(double x, double y, double distance, double curvature) {
        this.x = x;
        this.y = y;
        this.distance = distance;
        this.curvature = curvature;
    }

}