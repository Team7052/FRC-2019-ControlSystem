package frc.robot.commands.arm;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.motionProfiling.LowPassFilter;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.Point;
import frc.robot.subsystems.ArmSubsystem;

public class RotateWristJoint implements PIDOutput, PIDSource {

    PIDController angleController;
    PIDController positionController;

    double k_p1 = 0.010;
    double k_i1 = 0;
    double k_d1 = 0;

    double k_p2 = 0;
    double k_i2 = 0;
    double k_d2 = 0;

    LowPassFilter wristFilter;
    MotionProfiler motionProfiler;

    ArmSubsystem arm;

    boolean targetFront = true;

    public RotateWristJoint() {
        arm = ArmSubsystem.getInstance();
        angleController = new PIDController(k_p1, k_i1, k_d1, this, this);
     //   positionController = new PIDController(k_p2, k_i2, k_d2, source, output);
    }

    double motorSpeed = 0;

    boolean prevPressed = false;
    int initialPosition = 0;
    double wristMotorOutput = 0;
    double prevPitchError = 0;

    ArrayList<Point> errorFunction = new ArrayList<>();
    double time = 0;
    public void execute() {

        PigeonIMU sensor = arm.getIMUSensor();

        if (!this.angleController.isEnabled()) this.angleController.enable();

        if (sensor.getState() == PigeonState.Ready) {
            double angle = arm.getWristAngle();
            //System.out.println(arm.getWristAngle());
            
            if (wristFilter == null) {
                wristFilter = new LowPassFilter(90, angle);
                wristFilter.startFilter();
            }
            double target = wristFilter.updateFilter();
            if ((Math.abs(target) < 0.1 && Math.abs(angle) > 20) || Math.abs(target - angle) > 20) {
                wristFilter = new LowPassFilter(90, angle);
                wristFilter.startFilter();
                target = wristFilter.updateFilter();
            }

            if (Math.abs(angle) <= 20 && Math.abs(angle) > 0.5) {
                angleController.setP(k_p1);
                angleController.setI(0.00012);
                angleController.setD(0);
            }
            else {
                angleController.setI(k_i1);
                angleController.setP(0.01);
            }

            this.angleController.setSetpoint(target);
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
        return arm.getWristAngle();
    }
}
