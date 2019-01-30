package frc.robot.motionProfiling;

public class MotionTriplet {
    public double velocity;
    public double position;
    public double acceleration;

    public MotionTriplet(double velocity, double position, double acceleration) {
        this.velocity = velocity;
        this.position = position;
        this.acceleration = acceleration;
    }
}