package frc.robot.utilities;

public class Vector {

    public double i, j;

    public Vector(double i, double j) {
        this.i = i;
        this.j = j;
    }

    //  dot product, this vector is first, other vector is second
    public double dp(Vector v) {
        return this.mag() * v.mag() * Math.sin(v.head() - this.head());
    }

    //  returns subtracted vector, this vector is first, other vector is second
    public Vector subtract(Vector v) {
        return new Vector(i-v.i, j-v.j);
    }

    //  returns added vector
    public Vector add(Vector v) {
        return new Vector(i+v.i, j+v.j);
    }

    //  magnitude
    public double mag() {
        return Math.sqrt(i*i + j*j);
    }

    //  heading, in radians
    public double head() {
        return Math.tanh(j/i);
    }

}