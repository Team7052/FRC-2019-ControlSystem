package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.motionProfiling.FunctionGenerator;

public class DriveTenM extends Command {
    double time = 0;
    // declare subsystem variable
    DriveTrain driveTrain;
    
    public DriveTenM() {
        super("Drive Ten M");
        // get drive train subsystem instance
        driveTrain = DriveTrain.getInstance();
        
        //required for each command to know which subsystems it will be using
		requires(driveTrain);
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
    double point1x = 0;
    double point1y = 0;
    double point2x = 2;
    double point2y = 2.5;
    double point3x = 3;
    double point3y = 2.5;
    double point4x = 5;
    double point4y = 0;

    double velocity = 0;
    double constant = 3.19486;

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

  driveTrain.setLeftGroupSpeed(speed*1.05);
  driveTrain.setRightGroupSpeed(speed);

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