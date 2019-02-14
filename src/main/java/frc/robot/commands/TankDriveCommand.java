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
	double deadBand = 0.000;
	double kp = 0;
    
    public TankDriveCommand() {
        super("Tank Drive Command");
        // get drive train subsystem instance
		driveTrain = DriveTrain.getInstance();
		
        
        //required for each command to know which subsystems it will be using
		requires(driveTrain);
		leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
        rightEncoder = new Encoder(3, 2, false, EncodingType.k4X);
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
		double x = Robot.oi.axisLeft_X();
		double y = v - w;
		double rightSpeed = 0;
		double leftSpeed = 0;
		double leftTarget = 0;
		double rightTarget = 0;
		double difference = 0;
		double theta = 0;
		//System.out.println("Left: "+leftEncoder.getRate());
		//System.out.println("Right: "+rightEncoder.getRate());

		if (Math.abs(y) < deadBand) y=0;
		if (Math.abs(x) < deadBand) x=0;
		if (x != 0) {
			theta = Math.atan(Math.abs(y / x));
		} 
		else {
			theta = 0;
		}
		double ratio = Math.cos(theta);
		if (y == 0) ratio = -ratio;

		if (x >= 0) {
			leftSpeed=y;
			rightSpeed = leftSpeed*ratio;
		}
		if (x < 0) {
			rightSpeed=y;
			leftSpeed=rightSpeed*ratio;
		}
		if(y==0){
			leftSpeed=x*0.65;
			rightSpeed = leftSpeed * ratio;
		}
		leftTarget = leftEncoder.getRate();
		rightTarget = leftTarget*ratio;
		//System.out.println(leftSpeed);
		difference = -rightTarget + rightEncoder.getRate();
		driveTrain.setLeftGroupSpeed(leftSpeed*0.5);
		driveTrain.setRightGroupSpeed((rightSpeed +difference*kp)*0.5);
		
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