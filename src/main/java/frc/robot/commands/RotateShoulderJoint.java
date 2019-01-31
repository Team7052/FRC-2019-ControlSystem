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

        ArrayList<Point> points = new ArrayList<>();

        points.add(new Point(0,0));
        points.add(new Point(0.2, 1));
        points.add(new Point(0.5, 1));
        points.add(new Point(0.8, 0));

        ArrayList<Point> interpolatedPoints = motionProfiler.getLinearInterpolation(points, 0.001);

        motionProfiler.setVelocityPoints(interpolatedPoints);
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
        MotionTriplet triplet = motionProfiler.updateMotionProfile(0.8);
        if (motionProfiler.getState() == MotionProfileState.RUNNING && triplet != null) {
           arm.setPosition(Motor.SHOULDER_JOINT, (int) (arm.homePosition + triplet.position / (2 * Math.PI) * 4096));
        }
        else if (motionProfiler.getState() == MotionProfileState.FINISHED) {
            if (motionProfiler.getPositionFunction().size() > 0) {
                double position = motionProfiler.getPositionFunction().get(motionProfiler.getPositionFunction().size() - 1).y;
                arm.setPosition(Motor.SHOULDER_JOINT, (int) (arm.homePosition + position / (2 * Math.PI) * 4096));
            }
        }
        
        
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