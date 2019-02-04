// pure pursuit trajectory point

package frc.robot;

public class PPPoint {

    public double x, y, distance, curvature;
    public double velocity = 0;

    public PPPoint(double x, double y, double distance, double curvature) {
        this.x = x;
        this.y = y;
        this.distance = distance;
        this.curvature = curvature;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double calculateDistance(double otherX, double otherY) {
        return Math.sqrt((x-otherX)*(x-otherX)+(y-otherY)*(y-otherY));
    }
}