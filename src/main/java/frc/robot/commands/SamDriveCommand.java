
package frc.robot.commands;

import java.time.Year;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

//

public class SamDriveCommand extends Command {	
	double currentSpeedLeft = 0;
    double currentSpeedRight = 0;
    
    DriveTrain driveTrain;
    
    public SamDriveCommand() {
        driveTrain = DriveTrain.getInstance();

        requires(driveTrain);
    }

    @Override
    protected void initialize() {
        super.initialize();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        super.execute();
        System.out.println("Execute");
        double x = Robot.oi.axisLeft_X();
        double lt = -Robot.oi.axisTrigger_L2();
        double rt = Robot.oi.axisTrigger_R2();

        double leftSpeed = lt + rt;
        double rightSpeed = lt + rt;

        double speed = 0.8;
        double deadzone = 0.1;

        System.out.println("Right: " + rightSpeed);
        System.out.println("Left: " + leftSpeed);

        if (Math.abs(x) > deadzone) { //if youve moved the joystick out of the deadzone
            if (x > 0) rightSpeed = Math.signum(x) * rightSpeed * speed; 
            else leftSpeed = Math.signum(x) * leftSpeed * speed;
        }

        double finalLeftSpeed = bufferSpeed(this.currentSpeedLeft, leftSpeed);
		double finalRightSpeed = bufferSpeed(this.currentSpeedRight, rightSpeed);
		driveTrain.setLeftGroupSpeed(finalLeftSpeed);
		driveTrain.setRightGroupSpeed(finalRightSpeed);
		this.currentSpeedLeft = finalLeftSpeed;
		this.currentSpeedRight = finalRightSpeed;
    }

	public double bufferSpeed(double currentSpeed, double desiredSpeed){
		double speedIncrement=desiredSpeed;
		double threshold = 0.05;
		if(Math.abs(desiredSpeed-currentSpeedLeft)>threshold){
			if(desiredSpeed>currentSpeedLeft){
				speedIncrement=currentSpeedLeft+threshold;
			}
			if(desiredSpeed<currentSpeedLeft){
				speedIncrement=currentSpeedLeft-threshold;
			}
		}
		if(Math.abs(desiredSpeed-currentSpeedLeft)<0.1){
			speedIncrement=desiredSpeed;
		}

		return speedIncrement;
	}
}