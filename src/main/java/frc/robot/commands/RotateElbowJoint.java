package frc.robot.commands;

import java.util.ArrayList;

import frc.robot.helpers.RotationMotor;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.Point;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class RotateElbowJoint {

    MotionProfiler elbowMotionProfiler;
    ArmSubsystem arm;

    private boolean isRunning = false;
    public double targetHoldAbsoluteAngle = 180;

    public RotateElbowJoint() {
        arm = ArmSubsystem.getInstance();
        this.targetHoldAbsoluteAngle = arm.getHomeDegrees(Motor.ELBOW_JOINT);

        elbowMotionProfiler = new MotionProfiler();
    }


    public boolean isFinished() {
        return this.isRunning == false;
    }

    public void restartMotionProfile(double angle, MotionProfiler shoulderProfile, double maxVelocity, double maxAcceleration) {
        RotationMotor elbowMotor = arm.getRotationMotor(Motor.ELBOW_JOINT);
        double initRadians = this.targetHoldAbsoluteAngle / 180.0 * Math.PI;
        double endRadians =  angle / 180.0 * Math.PI;
        ArrayList<Point> shoulderPositions = shoulderProfile.getPositionFunction();
        elbowMotionProfiler.setVelocityPoints(MotionProfiler.generateTrapezoidalProfile(initRadians, endRadians, maxVelocity, maxAcceleration), initRadians);
        ArrayList<Point> elbowPositions = elbowMotionProfiler.getPositionFunction();
        ArrayList<Point> newElbowPositions = new ArrayList<>();
        System.out.println(initRadians + " " + endRadians);
        System.out.println("initial elbow: " + (elbowPositions.get(0).y / Math.PI * 180) + ", initial shoulder: " + (shoulderPositions.get(0).y / Math.PI * 180));
        System.out.println("final elbow: " + (elbowPositions.get(elbowPositions.size() - 1).y / Math.PI * 180) + ", final shoulder: " + (shoulderPositions.get(shoulderPositions.size() - 1).y / Math.PI * 180));

        // adjust the elbow positions based on shoulder positions
        if (elbowPositions.size() >= shoulderPositions.size()) {
            for (int i = 0; i < elbowPositions.size(); i++) {
                double targetPos = 0;
                if (i < shoulderPositions.size()) targetPos = shoulderPositions.get(i).y;
                else targetPos = shoulderPositions.get(shoulderPositions.size() - 1).y;

                newElbowPositions.add(new Point(elbowPositions.get(i).x, this.radiansRelativeToShoulder(elbowPositions.get(i).y, targetPos)));
            }
        }
        else {
            for (int i = 0; i < shoulderPositions.size(); i++) {
                double targetPos = shoulderPositions.get(i).y;
                if (i < elbowPositions.size()) newElbowPositions.add(new Point(shoulderPositions.get(i).x, this.radiansRelativeToShoulder(elbowPositions.get(i).y, targetPos)));
                else newElbowPositions.add(new Point(shoulderPositions.get(i).x, this.radiansRelativeToShoulder(elbowPositions.get(elbowPositions.size() - 1).y, targetPos)));
            }
        }

        System.out.println("initial relative: " + (newElbowPositions.get(0).y / Math.PI * 180) + ", final relative:  " + (newElbowPositions.get(newElbowPositions.size() - 1).y / Math.PI * 180));
        this.targetHoldAbsoluteAngle = elbowPositions.get(elbowPositions.size() - 1).y / Math.PI * 180.0;

        elbowMotionProfiler.setPositionPoints(newElbowPositions);
        
        elbowMotionProfiler.restartMotionProfile();
        this.isRunning = true;
    }

    public void execute() {
        if (this.isRunning) {
            elbowMotionProfiler.startMotionProfile();
            MotionTriplet elbowTriplet = elbowMotionProfiler.updateMotionProfile();
            if (elbowMotionProfiler.getState() == MotionProfileState.RUNNING && elbowTriplet != null) {
                // set position relative to shoulder joint
                double elbowAbsoluteDegrees = elbowTriplet.position / (2 * Math.PI) * 360;
                arm.setDegrees(Motor.ELBOW_JOINT, elbowAbsoluteDegrees);
                System.out.println("Target: " + elbowAbsoluteDegrees + ", current: " + arm.getDegrees(Motor.ELBOW_JOINT));
            }
            else if (elbowMotionProfiler.getState() == MotionProfileState.FINISHED) {
                if (elbowMotionProfiler.getPositionFunction().size() > 0) {
                    double position = elbowMotionProfiler.getPositionFunction().get(elbowMotionProfiler.getPositionFunction().size() - 1).y;

                    arm.setDegrees(Motor.ELBOW_JOINT, degreesRelativeToShoulder(this.targetHoldAbsoluteAngle));
                }
            }
        }
    }

    private double radiansRelativeToShoulder(double absoluteRadians, double shoulderRadians) {
        return absoluteRadians - shoulderRadians + arm.getHomeDegrees(Motor.SHOULDER_JOINT) / 360.0 * (Math.PI * 2);
    }
    private double degreesRelativeToShoulder(double absoluteDegrees) {
       return absoluteDegrees - arm.getDegrees(Motor.SHOULDER_JOINT) + arm.getHomeDegrees(Motor.SHOULDER_JOINT);
    }
}
