package frc.robot.commands;

import java.util.ArrayList;

import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.FunctionGenerator;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.Point;
import frc.robot.motionProfiling.TrapezoidalFunctions;
import frc.robot.util.physics.PhysicsWorld;

public class CoupledLiftProfiler {
    public static double clawMaxVelocity = Math.PI / 3;
    public static double clawMaxAcceleration = Math.PI / 2;
    public static double rackMaxVelocity = 2.5; // inches / s
    public static double rackMaxAcceleration = 1.5; // inches / s^2
    public static Pair<MotionProfiler> generateProfiles(double initClaw, double endClaw, double initRack, double endRack) {
        // define geometric variables
        ArrayList<Point> clawShape = TrapezoidalFunctions.generateTrapezoidalProfile(initClaw, endClaw, clawMaxVelocity, clawMaxAcceleration);
        ArrayList<Point> rackShape = TrapezoidalFunctions.generateTrapezoidalProfile(initRack, endRack, rackMaxVelocity, rackMaxAcceleration);

        MotionProfiler clawProfiler = new MotionProfiler();
        MotionProfiler rackProfiler = new MotionProfiler();

        clawProfiler.setVelocityPoints(FunctionGenerator.getLinearInterpolation(clawShape, 0.01), initClaw);
        rackProfiler.setVelocityPoints(FunctionGenerator.getLinearInterpolation(rackShape, 0.01), initRack);

        return new Pair<>(clawProfiler, rackProfiler);
    }

    public static Pair<MotionProfiler> getHabClimbProfiles(double initClaw, double endClaw, double initRack, double endRack) {
        // generate profiles
        ArrayList<Point> clawShape = TrapezoidalFunctions.generateTrapezoidalProfile(initClaw, endClaw, clawMaxVelocity, clawMaxAcceleration);
        ArrayList<Point> rackShape = TrapezoidalFunctions.generateTrapezoidalProfile(initRack, endRack, rackMaxVelocity, rackMaxAcceleration);
        double clawTime = TrapezoidalFunctions.totalTimeOfShape(clawShape);
        double rackTime = TrapezoidalFunctions.totalTimeOfShape(rackShape);

        MotionProfiler clawProfile = new MotionProfiler();
        MotionProfiler rackProfile = new MotionProfiler();

        if (clawTime <= rackTime) {
            // solve based on rack
            System.out.println("Solve with fixed rack heights");
            rackProfile.setVelocityPoints(FunctionGenerator.getLinearInterpolation(rackShape, 0.01), initRack);
            ArrayList<Point> clawPoints = new ArrayList<>();
            double give = 15.0 / 180.0 * Math.PI;
            for (Point position: rackProfile.getPositionFunction()) {
                double angle = PhysicsWorld.getInstance().solveClimberClawAngleForHeight(position.y, endRack);
                clawPoints.add(new Point(position.x, angle - give * position.x / rackTime));
            }

            clawProfile.setPositionPoints(clawPoints);
        }
        else {
            // solve based on claw
            System.out.println("Solve with fixed claw angles");
            clawProfile.setVelocityPoints(FunctionGenerator.getLinearInterpolation(clawShape, 0.01), initRack);
            ArrayList<Point> rackPoints = new ArrayList<>();
            for (Point position: clawProfile.getPositionFunction()) {
                double height = PhysicsWorld.getInstance().solveClimberRackHeightForAngle(position.y, endRack);
                rackPoints.add(new Point(position.x, height));
            }
            rackProfile.setPositionPoints(rackPoints, initClaw);
        }

        return new Pair<>(clawProfile, rackProfile);
    }
}