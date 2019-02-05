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
        points.add(new Point(3, 2.5));
        points.add(new Point(5, 0));

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
        if (time == 0) {
            time = Timer.getFPGATimestamp();
        }
        double currentTime = Timer.getFPGATimestamp();

        double timePassed = currentTime - time;
        // your code goes here:
    /*double x = Robot.oi.axisLeft_X();
    double v = Robot.oi.axisTrigger_R2();
    double w = Robot.oi.axisTrigger_L2();
    double forwardSpeed = v;
    double backwardSpeed = -w;
    double leftSpeed = forwardSpeed+backwardSpeed;
    double rightSpeed = forwardSpeed+backwardSpeed;
    */
    //line1: y=1.25x
    //line2: y=2.5
    //line3: y=-1.25x +6.25
    /*double point1x = 0;
    double point1y = 0;
    double point2x = 4;
    double point2y = 2.5;
    double point3x = 6;
    double point3y = 2.5;
    double point4x = 10;
    double point4y = 0;

    double velocity = 0;
    double constant = 2.55;

    if(timePassed>=point1x && timePassed<point2x){
        velocity = getSlope(point1x, point1y, point2x, point2y)*timePassed +getyIntercept(point1x, point1y, point2x, point2y);
    }
    if(timePassed>= point2x && timePassed<point3x){
        velocity = getSlope(point2x, point2y, point3x, point3y)*timePassed + getyIntercept(point2x, point2y, point3x, point3y);
    }
    if(timePassed>=point3x && timePassed<=point4x){
        velocity = getSlope(point3x, point3y, point4x, point4y)*timePassed +getyIntercept(point3x, point3y, point4x, point4y);
    }

    double speed = velocity/constant;

  driveTrain.setLeftGroupSpeed(speed);
  driveTrain.setRightGroupSpeed(speed/(1.045)); */
  System.out.println("Left Encoder Count: " +leftEncoder.get());
  System.out.println("Right Encoder Count: " +rightEncoder.get());


  double constant = 2.55;
  double speed = 0;
  motionProfiler.startMotionProfile();
  MotionTriplet triplet = motionProfiler.updateMotionProfile(5.0);
  if (motionProfiler.getState() == MotionProfileState.RUNNING && triplet != null) {
    double velocity = triplet.velocity;
    speed = velocity / constant;
  }
  
  //driveTrain.setLeftGroupSpeed(speed);
  //driveTrain.setRightGroupSpeed(speed/(1.04));



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