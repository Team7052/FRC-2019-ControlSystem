package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.motionProfiling.LowPassFilter;
import frc.robot.motionProfiling.Point;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class RotateWristJoint implements PIDOutput, PIDSource {

    PIDController pidController;

    double k_p1 = 0.010;
    double k_i1 = 0;
    double k_d1 = 0;

    LowPassFilter wristFilter;

    ArmSubsystem arm;

    boolean targetFront = true;

    public RotateWristJoint() {
        arm = ArmSubsystem.getInstance();

        pidController = new PIDController(k_p1, k_i1, k_d1, this, this);
    }

    double motorSpeed = 0;

    boolean prevPressed = false;
    int initialPosition = 0;
    double wristMotorOutput = 0;
    double prevPitchError = 0;

    ArrayList<Point> errorFunction = new ArrayList<>();
    double time = 0;
    public void execute() {
        if (!pidController.isEnabled()) {
            pidController.enable();
        }

        AHRS sensor = arm.getIMUSensor();

        if (!sensor.isCalibrating() && sensor.isConnected()) {
            float pitch = sensor.getPitch();

            if (wristFilter == null) {
                wristFilter = new LowPassFilter(0, pitch);
                wristFilter.startFilter();
            }
            double target = wristFilter.updateFilter();
            if ((Math.abs(target) < 0.1 && Math.abs(pitch) > 20) || Math.abs(target - pitch) > 20) {
                wristFilter = new LowPassFilter(0, pitch);
                wristFilter.startFilter();
                target = wristFilter.updateFilter();
            }

            if (Math.abs(pitch) <= 20 && Math.abs(pitch) > 0.5) {
                pidController.setP(k_p1);
                pidController.setI(0.00012);
                pidController.setD(0);
            }
            else {
                pidController.setI(k_i1);
                pidController.setP(0.01);
            }

            this.pidController.setSetpoint(target);
            arm.getWristMotor().set(ControlMode.PercentOutput, this.wristMotorOutput);
        }
        else {
            arm.getWristMotor().set(ControlMode.PercentOutput, 0);
        }

    }

    double startTime = 0;

    @Override
    public void pidWrite(double output) {
        if (time == 0) {
            time = Timer.getFPGATimestamp();
        }            
        double max = 0.4;
        if (output > max) output = max;
        else if (output < -max) output = -max;
        if (targetFront) output *= -1;
        this.wristMotorOutput = output;
        
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
        //System.out.println(arm.getIMUSensor().getPitch());
        return arm.getIMUSensor().getPitch();
    }
}
