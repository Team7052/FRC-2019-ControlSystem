package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.subsystems.ArmSubsystem;

public class WristPositionPIDController implements PIDSource, PIDOutput {
    private ArmSubsystem arm;
    private RotateWristJoint wristJointCommand;

    public WristPositionPIDController(ArmSubsystem arm, RotateWristJoint wristJoint) {
        this.arm = arm;
        this.wristJointCommand = wristJoint;
    }

    @Override
    public void pidWrite(double output) {
        if (output > 0.5) output = 0.5;
        if (output < -0.5) output = -0.5;
        wristJointCommand.wristMotorOutput = output;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        pidSource = PIDSourceType.kDisplacement;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return arm.getWristPosition();
    }

}