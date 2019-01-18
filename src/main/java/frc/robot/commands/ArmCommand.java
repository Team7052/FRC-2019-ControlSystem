package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    ArmSubsystem arm;
    public ArmCommand(){
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
        WPI_TalonSRX motor = arm.motor1;
        motor.set(ControlMode.Current, motorSpeed);

    }
}