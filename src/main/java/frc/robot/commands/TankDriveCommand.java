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
        double value = Robot.oi.getAxis(0);
        if (value < 0.2) {
            value = 0;
        }
        driveTrain.setLeftGroupSpeed(value * 0.7);
        driveTrain.setRightGroupSpeed(value * 0.7);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}