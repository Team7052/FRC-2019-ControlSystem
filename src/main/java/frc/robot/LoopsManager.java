package frc.robot;

import frc.robot.commands.ClawCommand;
import frc.robot.commands.RackCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.arm.ArmControllerCommand;
import frc.robot.networking.Network;
import frc.robot.states.ArmSuperState;
import frc.robot.states.ClimberSuperState;
import frc.robot.states.substates.ArmState;
import frc.robot.states.substates.ClimberState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;
import frc.robot.subsystems.Climber;
import frc.robot.util.loops.Loop;
import frc.robot.util.physics.PhysicsWorld;

import frc.robot.commands.LiftCommand;

public class LoopsManager {
    ArmSuperState armSuperState;
    ArmSubsystem arm;

    ArmControllerCommand armCommand;
    TankDriveCommand driveCommand;
    RackCommand rackCommand;
    ClawCommand clawCommand;
    LiftCommand liftCommand;
    
    public LoopsManager() {
        armSuperState = ArmSuperState.getInstance();

        arm = ArmSubsystem.getInstance();
        armCommand = new ArmControllerCommand();
        driveCommand = new TankDriveCommand();
        rackCommand = new RackCommand();
        clawCommand = new ClawCommand();
        liftCommand = new LiftCommand();

        ClimberSuperState.getInstance().setClawDelegate(clawCommand);
        ClimberSuperState.getInstance().setRackDelegate(rackCommand);
    }
    public Loop hardwareLoop = new Loop() {
        @Override
        public synchronized void onUpdate() {
             armCommand.execute();
             driveCommand.execute();
             //rackCommand.execute();
             //clawCommand.execute();
             //liftCommand.execute();
             //System.out.println(Climber.getInstance().getClaw().getDegrees() + " " + Climber.getInstance().getClaw().getPosition());
        }
    };

    public Loop stateManagerLoop = new Loop() {
        @Override
        public synchronized void onUpdate() {
            if (Robot.oi.button_A()) {
                armSuperState.setState(ArmState.home);
            }
            else if (Robot.oi.button_R2()) {
                armSuperState.setState(ArmState.intakeHatch);
            }
            else if (Robot.oi.button_X()) {
                armSuperState.setState(ArmState.lowRocketHatch);
            }
            else if (Robot.oi.button_B()) {
                armSuperState.setState(ArmState.midRocketHatch);
            }
            else if (Robot.oi.button_Y()) {
                armSuperState.setState(ArmState.highRocketHatch);
            }
            else if (Robot.oi.button_L3()) {
                armSuperState.setState(ArmState.intakeCargo);
            }
            else if (Robot.oi.button_R3()) {
                armSuperState.setState(ArmState.lowRocketCargo);
            }
            else if (Robot.oi.dPad_DOWN()) {
                armSuperState.setState(ArmState.lowerArm);
            }
            else if (Robot.oi.dPad_UP()) {
                armSuperState.setState(ArmState.raiseArm);
            }

            if (Robot.oi.button_L1()) {
                ClimberSuperState.getInstance().setState(ClimberState.hab2Climb);
            }
            else if (Robot.oi.dPad_LEFT()) {
                ClimberSuperState.getInstance().setState(ClimberState.home);
            }
            else if (Robot.oi.dPad_RIGHT()) {
                ClimberSuperState.getInstance().setState(ClimberState.hab3Climb);
            }

            /*if (Robot.oi.button_A()) {
                ClimberSuperState.getInstance().setState(ClimberState.hab3Climb);
            }*/
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
            */

            armSuperState.update();
            ClimberSuperState.getInstance().update();
        }
    };

    public Loop networkLoop = new Loop() {
        @Override
        public synchronized void onUpdate() {
            Network.getInstance().sendDataToServer();
        }
    };

    public Loop physicsWorldLoop = new Loop() {
        @Override
        public synchronized void onUpdate() {
            PhysicsWorld.getInstance().updateArmKinematics(arm.getDegrees(Motor.SHOULDER_JOINT), arm.getAbsoluteDegrees(Motor.ELBOW_JOINT), arm.getAbsoluteDegrees(Motor.WRIST_JOINT), true);
            PhysicsWorld.getInstance().updateClimberKinematics(Climber.getInstance().getClaw().getDegrees() / 180.0 * Math.PI, Climber.getInstance().getLeg().getLinearPosition());
        }
    };
}