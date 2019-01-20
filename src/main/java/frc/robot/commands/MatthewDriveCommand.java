package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class MatthewDriveCommand extends Command {
    // declare subsystem variable
    DriveTrain driveTrain;
    
    public MatthewDriveCommand() {
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
        
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}