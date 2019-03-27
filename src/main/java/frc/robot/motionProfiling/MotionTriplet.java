package frc.robot.motionProfiling;

import frc.robot.helpers.Triplet;

public class MotionTriplet extends Triplet<Double> {
    public MotionTriplet(Double position, Double velocity, Double acceleration) {
        super(position, velocity, acceleration);
    }

    public double getPosition() { return a; }
    public double getVelocity() { return b; }
    public double getAcceleration() { return c; } 
}