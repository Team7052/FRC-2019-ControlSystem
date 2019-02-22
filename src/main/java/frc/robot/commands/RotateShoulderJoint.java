package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.motionProfiling.LowPassFilter;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.Point;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class RotateShoulderJoint extends Command implements PIDOutput, PIDSource {

    public int targetDegrees = 90;
    MotionProfiler shoulderMotionProfiler;
    MotionProfiler elbowMotionProfiler;

    PIDController pidController;

    double k_p1 = 0.01;
    double k_i1 = 0;
    double k_d1 = 0;

    LowPassFilter wristFilter;

    ArmSubsystem arm;
    public RotateShoulderJoint() {
        arm = ArmSubsystem.getInstance();
        requires(arm);
        
        shoulderMotionProfiler = new MotionProfiler();
        elbowMotionProfiler = new MotionProfiler();

        ArrayList<Point> points = new ArrayList<>();
        
        points.add(new Point(0,arm.getHomeDegrees(Motor.SHOULDER_JOINT) / 180 * Math.PI));
        points.add(new Point(1, 270.0 / 180.0 * Math.PI));
        ArrayList<Point> interpolatedShoulderPoints = shoulderMotionProfiler.getLinearInterpolation(points, 0.01);

        shoulderMotionProfiler.setPositionPoints(interpolatedShoulderPoints);

        ArrayList<Point> elbowPoints = new ArrayList<>();
        elbowPoints.add(new Point(0, arm.getHomeDegrees(Motor.ELBOW_JOINT) / 180 * Math.PI));
        elbowPoints.add(new Point(2.0, 270.0 / 180.0 * Math.PI));

        ArrayList<Point> interpolatedElbowPoints = elbowMotionProfiler.getLinearInterpolation(elbowPoints, 0.01);
        elbowMotionProfiler.setPositionPoints(interpolatedElbowPoints);

        pidController = new PIDController(k_p1, k_i1, k_d1, this, this);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    double motorSpeed = 0;

    boolean prevPressed = false;
    int initialPosition = 0;
    double wristMotorOutput = 0;
    double prevPitchError = 0;

    ArrayList<Point> errorFunction = new ArrayList<>();
    double time = 0;
    @Override
    protected void execute() {
        super.execute();
        if (!pidController.isEnabled()) {
            pidController.enable();
        }
        if (Robot.oi.button_A()) {
            shoulderMotionProfiler.startMotionProfile();
            MotionTriplet shoulderTriplet = shoulderMotionProfiler.updateMotionProfile(1);
            if (shoulderMotionProfiler.getState() == MotionProfileState.RUNNING && shoulderTriplet != null) {
                arm.setDegrees(Motor.SHOULDER_JOINT, shoulderTriplet.position / (2*Math.PI) * 360);
            }
            else if (shoulderMotionProfiler.getState() == MotionProfileState.FINISHED) {
                if (shoulderMotionProfiler.getPositionFunction().size() > 0) {
                    double position = shoulderMotionProfiler.getPositionFunction().get(shoulderMotionProfiler.getPositionFunction().size() - 1).y;
                    arm.setDegrees(Motor.SHOULDER_JOINT, position / (2*Math.PI) * 360);
                }
            }
            
            elbowMotionProfiler.startMotionProfile();
        }
        MotionTriplet elbowTriplet = elbowMotionProfiler.updateMotionProfile(2.0);
            if (elbowMotionProfiler.getState() == MotionProfileState.RUNNING && elbowTriplet != null) {
                arm.setDegrees(Motor.ELBOW_JOINT, elbowTriplet.position / (2*Math.PI) * 360);
              //  System.out.println("setpoint: " + elbowTriplet.position / (2*Math.PI) * 360 + ", current: " + arm.getDegrees(Motor.ELBOW_JOINT));
            }
            else if (elbowMotionProfiler.getState() == MotionProfileState.FINISHED) {
                if (elbowMotionProfiler.getPositionFunction().size() > 0) {
                    double position = elbowMotionProfiler.getPositionFunction().get(elbowMotionProfiler.getPositionFunction().size() - 1).y;
                    System.out.println(arm.getSpeed(Motor.ELBOW_JOINT) + "A");
                    arm.setDegrees(Motor.ELBOW_JOINT, position / (2 * Math.PI) * 360);
                }
            }
        // find wrist error
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
                pidController.setP(0.014);
                pidController.setI(0.00012);
                pidController.setD(0);
            }
            else {
                pidController.setI(k_i1);
                pidController.setP(this.k_p1);
            }

            this.pidController.setSetpoint(target);
        
            System.out.println("%: " + Math.round(this.wristMotorOutput * 100) + ", target: " + Math.round(target * 100) / 100  + ", pitch: " + pitch);
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
        //System.out.println(arm.getIMUSensor().getPitch());
        return arm.getIMUSensor().getPitch();
    }
}
