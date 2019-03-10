package frc.robot.commands;


import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;


public class TankDriveCommand extends Command {
    // declare subsystem variable
	DriveTrain driveTrain;
	double deadBand = 0.1;
	double kp = 0.00001;
    
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
		double v = Robot.oi.axisTrigger_R2();
		double w = Robot.oi.axisTrigger_L2();
		double x = Robot.oi.axisRight_X();
		double y = Robot.oi.axisLeft_Y();
		double rightSpeed = 0;
		double leftSpeed = 0;
		double leftTarget = 0;
		double rightTarget = 0;
		double difference = 0;
		double theta = 0;
		Encoder leftEncoder = driveTrain.getLeftEncoder();
		Encoder rightEncoder = driveTrain.getRightEncoder();
<<<<<<< HEAD
		//System.out.println("Left: "+leftEncoder.get());
		//System.out.println("Right: "+rightEncoder.get());

		double multiplier = 0.5;
=======
		System.out.println("Left: "+leftEncoder.get());
		System.out.println("Right: "+rightEncoder.get());
>>>>>>> c2edc1de8c80a3bb08a86713ecc459a196e1b944

		if (Robot.oi.button_L2()) multiplier = 0.85;
		if (Math.abs(y) < deadBand) y = 0;
		if (Math.abs(x) < deadBand) x = 0;
		if (y != 0) {
			theta = Math.atan(Math.abs(x / y));
		} 
		else {
			theta = 0;
		}
		double ratio = Math.cos(theta);
		if (y == 0) ratio = -ratio;

		if (x >= 0) {
			leftSpeed = y;
			rightSpeed = y * ratio;
		}
		else if (x < 0) {
			rightSpeed = y;
			leftSpeed=y*ratio;
		}

		if(y==0){
<<<<<<< HEAD
			leftSpeed=x*0.6;
=======
			leftSpeed=x*0.8;
>>>>>>> c2edc1de8c80a3bb08a86713ecc459a196e1b944
			rightSpeed = leftSpeed * ratio;
		}

		leftTarget = leftEncoder.getRate();
		rightTarget = leftTarget*ratio;
		difference = -rightTarget + rightEncoder.getRate();
		if (Math.abs(y) > 0.2) rightSpeed += difference * kp;
<<<<<<< HEAD
		driveTrain.setLeftGroupSpeed(leftSpeed * multiplier);
		driveTrain.setRightGroupSpeed(rightSpeed * multiplier);
=======
		driveTrain.setLeftGroupSpeed(leftSpeed*0.3);
		driveTrain.setRightGroupSpeed(rightSpeed * 0.3);
>>>>>>> c2edc1de8c80a3bb08a86713ecc459a196e1b944
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

	public void pickup_hatch() {

	}
	public void place_hatch() {

	}
	
}