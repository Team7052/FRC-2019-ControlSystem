package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class HoldArm90 extends Command {

    double targetPosition = 0;

    ArmSubsystem arm;
    public HoldArm90() {
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
        System.out.println("Execute");
        
        
    }
}