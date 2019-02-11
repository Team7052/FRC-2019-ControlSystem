package frc.robot.commands;

import java.util.ArrayList;

import javax.swing.plaf.ColorUIResource;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.motionProfiling.FunctionGenerator;
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

public class DriveTenM extends Command {
    double time = 0;
    // declare subsystem variable
    DriveTrain driveTrain;
    MotionProfiler motionProfiler;
    Encoder leftEncoder;
    Encoder rightEncoder;
    
    public DriveTenM() {
        super("Drive Ten M");
        // get drive train subsystem instance
        driveTrain = DriveTrain.getInstance();
        
        //required for each command to know which subsystems it will be using
        requires(driveTrain);
        

        ArrayList<Point> points = new ArrayList<>();

        points.add(new Point(0, 0));
        points.add(new Point(2, 2.5));
        points.add(new Point(6, 2.5));
        points.add(new Point(8, 0));

        motionProfiler = new MotionProfiler();
        ArrayList<Point> interpolatedPoints = motionProfiler.getLinearInterpolation(points,0.01);

        motionProfiler.setVelocityPoints(interpolatedPoints);

        rightEncoder = new Encoder(0, 1, false, EncodingType.k4X);
        leftEncoder = new Encoder(3, 4, false, EncodingType.k4X);





    }

    @Override
    protected void initialize() {
        super.initialize();
    }

    @Override
    protected void execute() {
        super.execute();
        
  if(motionProfiler.getState() == MotionProfileState.IDLE){
    leftEncoder.reset();
    rightEncoder.reset();
    
}
        if (time == 0) {
            time = Timer.getFPGATimestamp();
        }
        double currentTime = Timer.getFPGATimestamp();

        double timePassed = currentTime - time;

  double leftAngularDistance = -leftEncoder.get();
  double rightAngularDistance = rightEncoder.get();

  double currentLeftDisplacement = leftAngularDistance/4277;
  double currentRightDisplacement = rightAngularDistance/4277;


  System.out.println("Left Encoder Velocity: " +leftEncoder.getRate());
  System.out.println("Right Encoder Velocity: " +rightEncoder.getRate());
  



  double kv = 0.5; //needs to be tuned
  double kp = 4.5; //needs to be tuned
  double velocity = 0;
  double displacement = 0;
  double leftDifference = 0;
  double rightDifference = 0;
  motionProfiler.startMotionProfile();

  MotionTriplet triplet = motionProfiler.updateMotionProfile(8.0);
  if (motionProfiler.getState() == MotionProfileState.RUNNING && triplet != null) {
    
    velocity = triplet.velocity;
    displacement = triplet.position;
    leftDifference = displacement - currentLeftDisplacement;
    rightDifference = displacement - currentRightDisplacement;
  }
  double leftSpeed = (velocity * kv) + (leftDifference* kp);
  double rightSpeed = (velocity * kv) + (rightDifference* kp);

  if(timePassed<0.1){
    driveTrain.setLeftGroupSpeed(velocity*kv);
    driveTrain.setRightGroupSpeed(velocity*kv);
  }
  else{
    driveTrain.setLeftGroupSpeed(leftSpeed);
    driveTrain.setRightGroupSpeed(rightSpeed);
  }
  


    }

    public double getSlope(double x1, double y1, double x2, double y2){
        double slope = (y2-y1)/(x2-x1);
        return slope;
    }

    public double getyIntercept(double x1, double y1, double x2, double y2){
        double slope = getSlope(x1, y1, x2, y2);
        double yint = y2-(slope)*(x2);

        return yint;
    }


    @Override
    protected boolean isFinished() {
        return false;
    }



}