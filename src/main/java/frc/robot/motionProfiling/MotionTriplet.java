package frc.robot.motionProfiling;

import frc.robot.helpers.Triplet;

public class MotionTriplet extends Triplet<Double> {
    /**
     * Create a Triplet specifically that contains data about position, velocity and acceleration
     * @param position : double
     * @param velocity : double
     * @param acceleration : double
     */
    public MotionTriplet(Double position, Double velocity, Double acceleration) {
        super(position, velocity, acceleration);
    }

    /**
     * Return the position / value a
     * @return : double
     */
    public double getPosition() { return this.getFirst(); }
    /**
     * Return the velocity / value b
     * @return : double
     */
    public double getVelocity() { return this.getSecond(); }
    /**
     * Return the acceleration / value c
     * @return : double
     */
    public double getAcceleration() { return this.getThird(); }
}