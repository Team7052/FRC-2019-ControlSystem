package frc.robot.commands;


import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.motionProfiling.FunctionGenerator;
import frc.robot.motionProfiling.FunctionSet;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.Point;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class RotateShoulderJoint extends Command {

    public int targetDegrees = 90;
    MotionProfiler motionProfiler;

    ArmSubsystem arm;
    public RotateShoulderJoint() {
        arm = ArmSubsystem.getInstance();
        requires(arm);
        
        motionProfiler = new MotionProfiler();
        FunctionSet f1 = new FunctionSet((x) -> Math.PI * x, 0, 0.5, 0.01);
        FunctionSet f2 = new FunctionSet((x) -> -Math.PI * x + Math.PI, 0.51, 1.0, 0.01);

        motionProfiler.setVelocityPoints(FunctionGenerator.generate(f1, f2));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    double motorSpeed = 0;

    boolean prevPressed = false;
    int initialPosition = 0;

    ArrayList<Point> errorFunction = new ArrayList<>();

    @Override
    protected void execute() {
        super.execute();
        MotionTriplet triplet = motionProfiler.updateMotionProfile(1.0);
        int quadratureRawTarget, position;

        if (motionProfiler.getState() == MotionProfileState.RUNNING) {
            
        }
        
        
        int target = initialPosition + quadratureRawTarget;
        
        

        /*if (Robot.oi.button_A()) {
            if (!motionProfiler.running && !prevPressed) {
                motionProfiler.startMotionProfile();
                this.initialPosition = arm.getPosition(Motor.SHOULDER_JOINT);
            }
            
            if (triplet != null) {
                System.out.println(position % 4096 + " " +  target % 4096 + " ");
                arm.setPosition(Motor.SHOULDER_JOINT, target);
            }
            else {
                arm.setCurrent(Motor.SHOULDER_JOINT, arm.getPosition(Motor.SHOULDER_JOINT));
            }

            prevPressed = true;
        }
        else {
            prevPressed = false;
            motionProfiler.stopMotionProfile();
            arm.stop(Motor.SHOULDER_JOINT);
        }*/
    }
}