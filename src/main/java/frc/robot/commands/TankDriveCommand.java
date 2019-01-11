package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class TankDriveCommand extends Command {
    // declare subsystem variable
    DriveTrain driveTrain;
    public TankDriveCommand() {
        super("Tank Drive Command");
        // get drive train subsystem instance
        driveTrain = DriveTrain.getInstance();
        
        //required for each command to know which subsystems it will be using
        //requires(driveTrain);
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
    double r2 = Robot.oi.axisTrigger_R2();
    double l2 = Robot.oi.axisTrigger_L2();
    double forwardSpeed = Math.atan(r2);
    double backwardSpeed = -Math.atan(l2);
    double leftSpeed = forwardSpeed+backwardSpeed;
    double rightSpeed = forwardSpeed+backwardSpeed;

    if (x > 0.1) {
        rightSpeed= rightSpeed*(1-x);
    }
    if(x<-0.1){
        leftSpeed = leftSpeed*(1+x);
    }

    if ((Math.abs(l2)-Math.abs(r2))<0.1){
      if(x>0.1){
        rightSpeed = -0.5;
        leftSpeed = 0.5;
      }
      if(x<-0.1){
        rightSpeed = 0.5;
        leftSpeed = -0.5;
      }
    }
    driveTrain.setLeftGroupSpeed(leftSpeed);
    driveTrain.setRightGroupSpeed(rightSpeed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}