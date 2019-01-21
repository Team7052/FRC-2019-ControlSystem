package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class RotateShoulderJoint extends Command {

    double targetPosition = 0;

    ArmSubsystem arm;
    public RotateShoulderJoint() {
        arm = ArmSubsystem.getInstance();
        requires(arm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    double motorSpeed = 0;

    @Override
    protected void execute() {
        super.execute();
                
        
    }
}