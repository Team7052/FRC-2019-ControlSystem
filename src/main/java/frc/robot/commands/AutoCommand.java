package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.Point;

public class AutoCommand extends Command {
    double time = 0;
    // declare subsystem variable
    DriveTrain driveTrain;
    MotionProfiler motionProfiler;
    Encoder leftEncoder;
    Encoder rightEncoder;
    
    public AutoCommand() {
        super("Auto Command");
        // get drive train subsystem instance
        driveTrain = DriveTrain.getInstance();
        motionProfiler = new MotionProfiler();
        
        //required for each command to know which subsystems it will be using
        requires(driveTrain);
        


        

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

        double desiredTime = 5.0;

  double leftAngularDistance = -leftEncoder.get();
  double rightAngularDistance = rightEncoder.get();


  //Add (x, y) displacement points
  Point[] newPoints1 = {
    new Point (0, 0), new Point (2, 1), new Point (4, 4), new Point (6, 2)
  };
  double xs[] = {0, 2, 4, 6};
  double ys[] = {0, 1, 4, 2};
  ArrayList<Point> points = new ArrayList<>(Arrays.asList(newPoints1));

  //Begin cubic interpolation
  ArrayList<Point> tangents = motionProfiler.calcTangents(points);
  ArrayList<Point> newPoints = new ArrayList<Point>();

        for (int i = 0; i < newPoints.size(); i++) {
            ArrayList<Point> otherP = motionProfiler.calcFinalPoints(xs, i, ys, tangents);
            newPoints.addAll(otherP);
        }

        double kp = 0; //needs to be tuned
        double leftVelocity = 0;
        double rightVelocity = 0;
        double displacement = 0;
        double leftDifference = 0;
        double rightDifference = 0;
        int desiredPoint = 0;
        double leftTheo = 0;
        double rightTheo = 0;
        double numPoints = newPoints.size();      

  if(timePassed<=desiredTime){
    desiredPoint = (int) ((timePassed / desiredTime) * numPoints);
                //closest entered point to desirecpoint
                int i = motionProfiler.closest(xs, newPoints.get(desiredPoint).x);
                //System.out.println(i);
                boolean concaveUp = motionProfiler.concaveUp(xs, i, ys, tangents);
               // System.out.println("Concave Up at " + newPoints.get(desiredPoint).x + ": " + concaveUp);
                leftTheo = motionProfiler.getLeftSum(desiredPoint, newPoints);
                rightTheo = motionProfiler.getRightSum(desiredPoint, newPoints);
                leftVelocity = motionProfiler.getLeftSlope(desiredPoint, newPoints, concaveUp);
                rightVelocity = motionProfiler.getRightSlope(desiredPoint, newPoints, concaveUp);

                double leftSpeed = (leftVelocity) + (leftDifference * kp);
                double rightSpeed = (rightVelocity) + (rightDifference * kp);

                System.out.println(newPoints.get(desiredPoint).x+ ": " + Math.round((leftSpeed) * 100.0) / 100.0 + " " + Math.round((rightSpeed) * 100.0) / 100.0);
    
  }

  	//driveTrain.setLeftGroupSpeed(leftSpeed*0.3);
		//driveTrain.setRightGroupSpeed(rightSpeed * 0.3);
  


    }




    @Override
    protected boolean isFinished() {
        return false;
    }



}