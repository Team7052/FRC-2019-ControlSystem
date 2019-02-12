package frc.robot.commands;


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

public class TankDriveCommand extends Command {
    double currentSpeedLeft = 0;
	double currentSpeedRight = 0;
    // declare subsystem variable
	DriveTrain driveTrain;
	Encoder leftEncoder;
    Encoder rightEncoder;
    
    public TankDriveCommand() {
        super("Tank Drive Command");
        // get drive train subsystem instance
		driveTrain = DriveTrain.getInstance();
		
        
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

        // your code goes here:
    double x = Robot.oi.axisLeft_X();
    double v = Robot.oi.axisTrigger_R2();
    double w = Robot.oi.axisTrigger_L2();
    double forwardSpeed = v;
    double backwardSpeed = -w;
	double rightSpeed = forwardSpeed+backwardSpeed;
	boolean leftSpeedPositive = true;
	double leftSpeed= forwardSpeed+backwardSpeed;
	double arc = 0;
	double constant=1;

	if(x!=0){
		arc = Math.atan(Math.abs(rightSpeed)/Math.abs(x));
	} else{
		arc = 0;
	}
	if (x > 0.1) {
		rightSpeed = rightSpeed*arc;
	}
	else if (x < -0.1) {
		constant = -x*0.5*(1-x);

	}

	if(0.2> Math.abs(v+w)){

		if(x<-0.3){
			leftSpeed = -0.5*x;
			rightSpeed = 0.5*x;
		}
		else if(x>0.3){

			rightSpeed = 0.5*x;
			leftSpeed = -0.5*x;
		}
	}
	if(leftSpeed == forwardSpeed+backwardSpeed){
		leftSpeed = (rightEncoder.getRate() +leftEncoder.getRate())*constant/4277;
	}

		driveTrain.setLeftGroupSpeed(leftSpeed);
        driveTrain.setRightGroupSpeed(rightSpeed);
	
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