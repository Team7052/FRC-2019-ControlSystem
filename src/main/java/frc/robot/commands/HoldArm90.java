package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
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
        return true;
    }

    double motorSpeed = 0;

    @Override
    protected void execute() {
        super.execute();
        arm.setDegrees(Motor.jointBaseMotor, 90);
        
    }
}