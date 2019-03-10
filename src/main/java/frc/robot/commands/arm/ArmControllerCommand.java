package frc.robot.commands.arm;

import frc.robot.PhysicsConstants;
import frc.robot.PhysicsWorld;
import frc.robot.Robot;
import frc.robot.helpers.Pair;
import frc.robot.subsystems.ArmSubsystem;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ArmControllerCommand extends CommandGroup {
    public enum ArmState {
        IDLE, PROFILING_POSITIONS, HOLDING, PROFILING_SEQUENCE
    }
    public RotateShoulderJoint shoulderCommand;
    public RotateElbowJoint elbowCommand;
    public RotateWristJoint wristCommand;
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

        coupledArmProfiler = new CoupledArmProfiler(shoulderCommand, elbowCommand, wristCommand);

        this.physicsWorld = physicsWorld;
        this.state = ArmState.IDLE;
    }


    protected boolean isFinished() {
        return false;
    }

    public boolean motionProfilesRunning() {
        return shoulderCommand.isRunning() || elbowCommand.isRunning() || wristCommand.isRunning();
    }

    public boolean wristEnabled = false;

    boolean controlByLengthAndHeight = false;
    double current_l;
    double current_h;

    public void execute() {
        shoulderCommand.execute();
        elbowCommand.execute();
        wristCommand.execute();

        if (Robot.oi.button_A() && !currentProfile.equals("Home")) {
            currentProfile = "Home";
            controlByLengthAndHeight = false;
            this.setAngles(radians(25), radians(180));
            this.wristCommand.enableWrist();

        }
        else if (Robot.oi.button_X() && !currentProfile.equals("Lower Hatch")) {
            currentProfile = "Lower Hatch";
            controlByLengthAndHeight = true;
            //this.setAngles(radians(45), radians(90));
            current_l = 16;
            current_h = 26;
            this.setDistances(current_l, current_h);
            this.wristCommand.enableWrist();
        }
        else if (Robot.oi.button_B() && !currentProfile.equals("Mid Hatch")) {
            currentProfile = "Mid Hatch";
            //this.setAngles(radians(60), radians(160));
            controlByLengthAndHeight = true;
            current_l = 14;
            current_h = 52;
            this.setDistances(current_l, current_h);
            this.wristCommand.enableWrist();
        }
        else if (Robot.oi.button_Y() && !currentProfile.equals("High Hatch")) {
            currentProfile = "High Hatch";
            //this.setAngles(radians(150), radians(170));
            controlByLengthAndHeight = true;
            current_l = 2;
            current_h = 78;
            this.setDistances(current_l, current_h);
            this.wristCommand.enableWrist();
            //this.setAngles(radians(235), radians(180));
        }
        else if (Robot.oi.button_L1() && !motionProfilesRunning()) {
            currentProfile = "Pull out";
            if (this.controlByLengthAndHeight) {
                current_h -= 1.5;
                this.setDistances(current_l, current_h);
                wristCommand.disableWrist();
            }
        }
        else if (Robot.oi.button_L2() && !currentProfile.equals("Flip")) {
            currentProfile = "Flip";
            if (this.controlByLengthAndHeight) {
                current_h -= 1.5;
                this.setAngles(radians(235), radians(180), radians(330));
                wristCommand.enableWrist();
            }
        }
        else if (Robot.oi.button_R2() && !currentProfile.equals("Loading")) {
            currentProfile = "Loading";
            current_l = 16;
            current_h = 20;
            this.setDistances(current_l, current_h);
            controlByLengthAndHeight = true;
            this.wristCommand.enableWrist();
        }
        else if (Robot.oi.dPad_DOWN() && !motionProfilesRunning()) {
            if (controlByLengthAndHeight) {
                currentProfile = "move";
                current_h -= 1;
                this.setDistances(current_l, current_h);
                this.wristCommand.enableWrist();
            }
        }
        else if (Robot.oi.dPad_UP() && !motionProfilesRunning()) {
            currentProfile = "move";
            if (controlByLengthAndHeight) {
                current_h += 4;
                this.setDistances(current_l, current_h);
                this.wristCommand.enableWrist(); 
            }
        }

        this.physicsWorld.updateWorld(0, arm.getDegrees(ArmSubsystem.Motor.SHOULDER_JOINT), arm.getAbsoluteElbowDegrees(), true);
    }

    public double radians(double degrees) {
        return degrees / 180.0 * Math.PI;
    }

    public void setPullOutMotion() {
        ArrayList<Pair<Double>> deltas = new ArrayList<>();
        deltas.add(new Pair<Double>(0.0, -2.0));
        deltas.add(new Pair<Double>(-2.0, 0.0));
        ArrayList<Pair<Double>> newAngles = this.physicsWorld.displaceSequentially(deltas);
        //newAngles.add(new Pair<Double>(radians(30), radians(180)));
        this.generateSequence(newAngles);
    }

    public void setPullUpMotion() {
        ArrayList<Pair<Double>> deltas = new ArrayList<>();
        deltas.add(new Pair<Double>(0.0, 2.0));
        ArrayList<Pair<Double>> newAngles = this.physicsWorld.displaceSequentially(deltas);
        //newAngles.add(new Pair<Double>(radians(30), radians(180)));
        this.generateSequence(newAngles);
    }

    public void setAngles(double shoulderAngle, double elbowAngle) {
        this.coupledArmProfiler.generateProfilesWhenReady(shoulderAngle, elbowAngle, Math.PI);
    }

    public void setAngles(double shoulderAngle, double elbowAngle, double wristAngle) {
        this.coupledArmProfiler.generateProfilesWhenReady(shoulderAngle, elbowAngle, wristAngle);
    }


    public void setDistances(double l, double h) {
        this.setDistances(l, h, Math.PI);
    }

    public void setDistances(double l, double h, double wristRadians) {
        Pair<Double> angles = this.physicsWorld.inverseKinematics(l + PhysicsConstants.backToArm + PhysicsConstants.thickness - PhysicsConstants.hand, PhysicsConstants.armHeight + PhysicsConstants.baseHeight - h);
        this.setAngles(angles.a, angles.b, wristRadians);
    }

    public void generateSequence(ArrayList<Pair<Double>> newAngles) {
        this.coupledArmProfiler.generateSequentialProfilesWhenReady(newAngles);
    }
}