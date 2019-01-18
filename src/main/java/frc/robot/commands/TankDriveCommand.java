package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class TankDriveCommand extends Command {
    double currentSpeedLeft = 0;
	double currentSpeedRight = 0;
    // declare subsystem variable
    DriveTrain driveTrain;
    public TankDriveCommand() {
        super("Tank Drive Command");
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

        // your code goes here:
    double x = Robot.oi.axisLeft_X();
    double v = Robot.oi.axisTrigger_R2();
    double w = Robot.oi.axisTrigger_L2();
    double forwardSpeed = v;
    double backwardSpeed = -w;
    double leftSpeed = forwardSpeed+backwardSpeed;
    double rightSpeed = forwardSpeed+backwardSpeed;

    if (x > 0.1) {
			rightSpeed = (rightSpeed*(1+x));
		}
		else if (x < -0.1) {
			leftSpeed = (leftSpeed *(1-x));
		}

		if(0.2> Math.abs(v+w)){
			//System.out.println("y is between -0.3 and 0.3");
			if(x<-0.3){
				//System.out.println("turn left");
				rightSpeed = 0.5*x;
				leftSpeed = 0.5*-x;
			}
			else if(x>0.3){
				//System.out.println("turn right");
				leftSpeed = 0.5*-x;
				rightSpeed = 0.5*x;
			}
		}
		double finalLeftSpeed = bufferSpeedLeft(this.currentSpeedLeft, leftSpeed);
		double finalRightSpeed = bufferSpeedLeft(this.currentSpeedRight, rightSpeed);
		driveTrain.setLeftGroupSpeed(finalLeftSpeed);
        driveTrain.setRightGroupSpeed(finalRightSpeed);
		this.currentSpeedLeft = finalLeftSpeed;
		this.currentSpeedRight = finalRightSpeed;
	
    }

    public double bufferSpeedLeft(double currentSpeedLeft, double desiredSpeedLeft){
		double speedIncrement=desiredSpeedLeft;
		double threshold = 0.08;
		if(Math.abs(desiredSpeedLeft-currentSpeedLeft)>threshold){
			if(desiredSpeedLeft>currentSpeedLeft){
				speedIncrement=currentSpeedLeft+threshold;
			}
			if(desiredSpeedLeft<currentSpeedLeft){
				speedIncrement=currentSpeedLeft-threshold;
			}
		}
		if(Math.abs(desiredSpeedLeft-currentSpeedLeft)<0.1){
			speedIncrement=desiredSpeedLeft;
		}

        return speedIncrement;
    }
    @Override
    protected boolean isFinished() {
        return false;
    }

}