package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.FollowSplineCommand;
import frc.robot.commands.RackCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.arm.ArmControllerCommand;
import frc.robot.motionProfiling.Point;
import frc.robot.networking.Network;
import frc.robot.states.substates.ArmState;
import frc.robot.states.substates.ClimberState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;
import frc.robot.subsystems.Climber;
import frc.robot.util.loops.Loop;
import frc.robot.util.physics.PhysicsWorld;

import frc.robot.commands.LiftCommand;

public class LoopsManager {
    ArmSubsystem arm;
    Climber climber;

    ArmControllerCommand armCommand;
    TankDriveCommand driveCommand;
    RackCommand rackCommand;
    ClawCommand clawCommand;
    LiftCommand liftCommand;
    FollowSplineCommand autoCommand;
    
    public LoopsManager() {
        arm = ArmSubsystem.getInstance();
        climber = Climber.getInstance();
        armCommand = new ArmControllerCommand();
        driveCommand = new TankDriveCommand();
        rackCommand = new RackCommand();
        clawCommand = new ClawCommand();
        liftCommand = new LiftCommand();
        
        Point[] path = {
            new Point (0, 0), new Point (27,56.5), new Point (45, 81.5), new Point (60, 89.5)
        };
        autoCommand = new FollowSplineCommand(new ArrayList<>(Arrays.asList(path)), 3.5);

        climber.getSuperState().setClawDelegate(clawCommand);
        climber.getSuperState().setRackDelegate(rackCommand);
    }

    public Loop hardwareLoop = new Loop() {
        @Override
        public synchronized void onUpdate() {
            armCommand.execute();
            driveCommand.execute();
            rackCommand.execute();
            clawCommand.execute();
            //liftCommand.execute();
        }
    };

    public Loop stateManagerLoop = new Loop() {
        @Override
        public synchronized void onUpdate() {
            if (Robot.oi.button_A()) {
                arm.getSuperState().setState(ArmState.home);
            }
            else if (Robot.oi.button_R2()) {
                arm.getSuperState().setState(ArmState.intakeHatch);
            }
            else if (Robot.oi.button_X()) {
                arm.getSuperState().setState(ArmState.lowRocketHatch);
            }
            else if (Robot.oi.button_B()) {
                arm.getSuperState().setState(ArmState.midRocketHatch);
            }
            else if (Robot.oi.button_Y()) {
                arm.getSuperState().setState(ArmState.highRocketHatch);
            }
            else if (Robot.oi.button_L3()) {
                arm.getSuperState().setState(ArmState.intakeCargo);
            }
            else if (Robot.oi.button_R3()) {
                arm.getSuperState().setState(ArmState.lowRocketCargo);
            }
            else if (Robot.oi.dPad_DOWN()) {
                arm.getSuperState().setState(ArmState.lowerArm);
            }
            else if (Robot.oi.dPad_UP()) {
                arm.getSuperState().setState(ArmState.raiseArm);
            }

            if (Robot.oi2 != null) {
                /*if (Robot.oi2.button_X()) {
                    climber.getSuperState().setState(ClimberState.hab2Climb);
                }
                else if (Robot.oi2.button_A()) {
                    climber.getSuperState().setState(ClimberState.home);
                }
                else if (Robot.oi.button_Y()) {
                    climber.getSuperState().setState(ClimberState.hab3Climb);
                }*/
            }

            

            arm.getSuperState().update();
            climber.getSuperState().update();
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
            PhysicsWorld.getInstance().updateClimberKinematics(Climber.getInstance().getClaw().getDegrees() / 180.0 * Math.PI, Climber.getInstance().getRack().getLinearPosition());
        }
    };

    public Loop autoLoop = new Loop() {
        double prev = 0;
        @Override
        public void onStart() {
        }
        @Override
        public void onUpdate() {
            if (prev == 0) prev = Timer.getFPGATimestamp();
            double timestamp = Timer.getFPGATimestamp();
            autoCommand.execute();

            if (timestamp - prev > 1.5) {
                arm.getSuperState().setState(ArmState.highRocketHatch);
            }
            armCommand.execute();
            arm.getSuperState().update();
        }
    };
}