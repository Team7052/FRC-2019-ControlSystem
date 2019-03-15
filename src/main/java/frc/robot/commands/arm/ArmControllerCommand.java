package frc.robot.commands.arm;

import frc.robot.PhysicsWorld;
import frc.robot.Robot;
import frc.robot.helpers.Triplet;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.sequencing.Sequence;
import frc.robot.states.ArmSuperState.ArmState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ArmControllerCommand extends CommandGroup implements Runnable {
    ArmSubsystem arm;
    JointController shoulderController;
    JointController elbowController;
    JointController wristController;

    Thread t;
    ScheduledExecutorService timer;

    public ArmControllerCommand() {
        arm = ArmSubsystem.getInstance();
        shoulderController = new JointController(Motor.SHOULDER_JOINT);
        elbowController = new JointController(Motor.ELBOW_JOINT);
        wristController = new JointController(Motor.WRIST_JOINT);

        t = new Thread(this, "Arm Controller Thread");
        timer = Executors.newSingleThreadScheduledExecutor();
        timer.scheduleAtFixedRate(this, 0, 10, TimeUnit.MILLISECONDS);
        t.start();
    }

    ArmState shouldSetNewState;

    protected boolean isFinished() {
        return false;
    }

    @Override
    public void run() {
        if (shouldSetNewState != null) {
            Triplet<Sequence<MotionTriplet>> triplet = arm.setState(shouldSetNewState);
            this.setControllers(triplet);
            shouldSetNewState = null;
        }
    }

    @Override
    public void execute() {
        if (Robot.oi.button_A()) {
           shouldSetNewState = ArmState.home;
        }
        else if (Robot.oi.button_R2()) {
            shouldSetNewState = ArmState.intakeHatch;
        }
        else if (Robot.oi.button_X()) {
            shouldSetNewState = ArmState.lowRocketHatch;
        }
        else if (Robot.oi.button_B()) {
            shouldSetNewState = ArmState.midRocketHatch;
        }
        else if (Robot.oi.button_Y()) {
            shouldSetNewState = ArmState.highRocketHatch;
        }
        else if (Robot.oi.button_L3()) {
            shouldSetNewState = ArmState.intakeCargo;
        }
        else if (Robot.oi.button_R3()) {
            shouldSetNewState = ArmState.lowRocketCargo;
        }

        this.shoulderController.execute();
        this.elbowController.execute();
        this.wristController.execute();
        /*
        if (Robot.oi.button_L1() && !motionProfilesRunning()) {
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
        */
        PhysicsWorld.getInstance().updateWorld(0, arm.getDegrees(Motor.SHOULDER_JOINT), arm.getAbsoluteDegrees(Motor.ELBOW_JOINT), true);

    }

    private void setControllers(Triplet<Sequence<MotionTriplet>> triplet) {
        if (triplet != null) {
            shoulderController.jointSequence = triplet.a;
            elbowController.jointSequence = triplet.b;
            wristController.jointSequence = triplet.c;
        }
    }
}