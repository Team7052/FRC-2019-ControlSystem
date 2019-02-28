package frc.robot.commands.arm;

import frc.robot.PhysicsConstants;
import frc.robot.PhysicsWorld;
import frc.robot.Robot;
import frc.robot.helpers.Pair;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ArmControllerCommand extends CommandGroup {
    public enum ArmState {
        IDLE, PROFILING_POSITIONS, HOLDING, PROFILING_SEQUENCE
    }
    RotateShoulderJoint shoulderCommand;
    RotateElbowJoint elbowCommand;
    RotateWristJoint wristCommand;
    ArmSubsystem arm;
    CoupledArmProfiler coupledArmProfiler;

    PhysicsWorld physicsWorld;

    String currentProfile = "none";
    public ArmState state;

    public ArmControllerCommand() {
        this.init(new PhysicsWorld());
    }

    public ArmControllerCommand(PhysicsWorld physicsWorld) {
        this.init(physicsWorld);
    }

    private void init(PhysicsWorld physicsWorld) {
        arm = ArmSubsystem.getInstance();
        shoulderCommand = new RotateShoulderJoint();
        elbowCommand = new RotateElbowJoint();
        wristCommand = new RotateWristJoint();

        coupledArmProfiler = new CoupledArmProfiler(shoulderCommand, elbowCommand);

        this.physicsWorld = physicsWorld;
        this.state = ArmState.IDLE;
    }


    protected boolean isFinished() {
        return false;
    }

    public boolean motionProfilesRunning() {
        return shoulderCommand.isRunning() || elbowCommand.isRunning();
    }

    public void execute() {
        shoulderCommand.execute();
        elbowCommand.execute();
        wristCommand.execute();

        if (Robot.oi.button_A() && !currentProfile.equals("Home")) {
            currentProfile = "Home";
            this.setAngles(radians(25), radians(180));

        }
        else if (Robot.oi.button_X() && !currentProfile.equals("Lower Hatch")) {
            currentProfile = "Lower Hatch";
            //this.setAngles(radians(45), radians(90));
            this.setDistances(20, 21);
        }
        else if (Robot.oi.button_B() && !currentProfile.equals("Mid Hatch")) {
            currentProfile = "Mid Hatch";
            //this.setAngles(radians(60), radians(160));
            this.setDistances(18,47);
        }
        else if (Robot.oi.button_Y() && !currentProfile.equals("High Hatch")) {
            currentProfile = "High Hatch";
           // this.setAngles(radians(150), radians(170));
            this.setDistances(9, 75);
        }
        else if (Robot.oi.button_L1() && !motionProfilesRunning()) {
            currentProfile = "Pull out";
            this.setPullOutMotion();
        }
        else if (Robot.oi.button_L2() && !motionProfilesRunning()) {
            currentProfile = "Pull up";
            this.setPullUpMotion();
        }

        this.physicsWorld.updateWorld(0, arm.getDegrees(ArmSubsystem.Motor.SHOULDER_JOINT), arm.getAbsoluteElbowDegrees(), true);
    }

    public double radians(double degrees) {
        return degrees / 180.0 * Math.PI;
    }

    public void setDeltaAngles(double shoulderDelta, double elbowDelta) {
        /*double shoulderDegrees = arm.getDegrees(Motor.SHOULDER_JOINT);
        double elbowDegrees = arm.getDegrees(Motor.ELBOW_JOINT);

        shoulderCommand.restartMotionProfile(shoulderDegrees + shoulderDelta, Math.PI / 2, Math.PI / 2);
        elbowCommand.restartMotionProfile(elbowCommand.targetHoldAbsoluteAngle + elbowDelta, shoulderCommand.shoulderMotionProfiler, Math.PI / 2, Math.PI / 2);*/
    }

    public void setPullOutMotion() {
        ArrayList<Pair<Double>> deltas = new ArrayList<>();
        deltas.add(new Pair<Double>(0.0, -2.0));
        deltas.add(new Pair<Double>(-4.0, 0.0));
        ArrayList<Pair<Double>> newAngles = this.physicsWorld.displaceSequentially(deltas);
        //newAngles.add(new Pair<Double>(radians(30), radians(180)));
        //this.generateSequence(newAngles);
        System.out.println(newAngles);
    }

    public void setPullUpMotion() {
        ArrayList<Pair<Double>> deltas = new ArrayList<>();
        deltas.add(new Pair<Double>(0.0, 4.0));
        ArrayList<Pair<Double>> newAngles = this.physicsWorld.displaceSequentially(deltas);
        //newAngles.add(new Pair<Double>(radians(30), radians(180)));
        //this.generateSequence(newAngles);
        System.out.println(newAngles);
    }


    public void setAngles(double shoulderAngle, double elbowAngle) {
        this.coupledArmProfiler.generateProfiles(shoulderAngle, elbowAngle);
    }

    public void setDistances(double l, double h) {
        Pair<Double> angles = this.physicsWorld.inverseKinematics(l + PhysicsConstants.baseWidth - PhysicsConstants.backToArm + PhysicsConstants.thickness - PhysicsConstants.hand, PhysicsConstants.armHeight + PhysicsConstants.baseHeight - h);
        this.setAngles(angles.a, angles.b);
    }


    public void generateSequence(ArrayList<Pair<Double>> newAngles) {
        this.coupledArmProfiler.generateSequentialProfiles(newAngles);
    }
}