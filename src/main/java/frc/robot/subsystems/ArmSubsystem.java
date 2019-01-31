package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.motionProfiling.FunctionGenerator;
import frc.robot.motionProfiling.FunctionSet;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmSubsystem extends Subsystem {
    // static variable that represents the drive train
  public enum Motor {
    SHOULDER_JOINT, ELBOW_JOINT
  }
  private static ArmSubsystem instance;
  private WPI_TalonSRX shoulderJointMotor;
  private final double shoulderJointMotor_kP = 1.6;
  private final double shoulderJointMotor_kI = 0;//0.0015;
  private final double shoulderJointMotor_kD = 0;
  private final double shoulderJointMotor_kF = 0.0;

  private WPI_TalonSRX elbowJointMotor;
  private final double elbowJointMotor_kP = 1.6;
  private final double elbowJointMotor_kI = 0;//0.0015;
  private final double elbowJointMotor_kD = 0;
  private final double elbowJointMotor_kF = 0.0;

  MotionProfiler motionProfiler;

  // always get the current instance of the drive train
  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  // home position is motor as of prototype one of the arm
  public final int shoulderHomePosition = 3764;
  public final int elbowHomePosition = 1238;
  // going positive degrees is negative for quadrature

  // private initializer so you can't initialize more than 1 drive train
  private ArmSubsystem() {
    // set up the new arm motor
    shoulderJointMotor = new WPI_TalonSRX(RobotMap.ARM_SHOULDER_JOINT_MOTOR);

    shoulderJointMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    shoulderJointMotor.setSensorPhase(true);
    shoulderJointMotor.configNominalOutputForward(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configNominalOutputReverse(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configPeakOutputForward(0.8, RobotMap.kPIDTimeoutMillis);
    shoulderJointMotor.configPeakOutputReverse(-0.8, RobotMap.kPIDTimeoutMillis);
    
    shoulderJointMotor.config_kP(RobotMap.kPIDIdx, this.shoulderJointMotor_kP);
    shoulderJointMotor.config_kI(RobotMap.kPIDIdx, this.shoulderJointMotor_kI);
    shoulderJointMotor.config_kD(RobotMap.kPIDIdx, this.shoulderJointMotor_kD);
    shoulderJointMotor.config_kF(RobotMap.kPIDIdx, this.shoulderJointMotor_kF);

    int shoulderAbsolutePosition = shoulderJointMotor.getSensorCollection().getPulseWidthPosition();
    shoulderJointMotor.setSelectedSensorPosition(shoulderAbsolutePosition, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    shoulderJointMotor.configAllowableClosedloopError(10, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);


    elbowJointMotor = new WPI_TalonSRX(RobotMap.ARM_ELBOW_JOINT_MOTOR);

    elbowJointMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    //elbowJointMotor.setSensorPhase(true);
    elbowJointMotor.configNominalOutputForward(0, RobotMap.kPIDTimeoutMillis);
		elbowJointMotor.configNominalOutputReverse(0, RobotMap.kPIDTimeoutMillis);
		elbowJointMotor.configPeakOutputForward(0.8, RobotMap.kPIDTimeoutMillis);
    elbowJointMotor.configPeakOutputReverse(-0.8, RobotMap.kPIDTimeoutMillis);
    
    elbowJointMotor.config_kP(RobotMap.kPIDIdx, this.elbowJointMotor_kP);
    elbowJointMotor.config_kI(RobotMap.kPIDIdx, this.elbowJointMotor_kI);
    elbowJointMotor.config_kD(RobotMap.kPIDIdx, this.elbowJointMotor_kD);
    elbowJointMotor.config_kF(RobotMap.kPIDIdx, this.elbowJointMotor_kF);

    int elbowAbsolutePosition = elbowJointMotor.getSensorCollection().getPulseWidthPosition();
    elbowJointMotor.setSelectedSensorPosition(elbowAbsolutePosition, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    elbowJointMotor.configAllowableClosedloopError(10, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  public void setCurrent(Motor motor, double current) {
    WPI_TalonSRX selectedMotor = getMotor(motor);
    selectedMotor.set(ControlMode.Current, current);
  }

  public void stop(Motor motor) {
    WPI_TalonSRX selectedMotor = getMotor(motor);
    selectedMotor.set(ControlMode.Current, 0);
  }

  public void setSpeed(Motor motor, double speed) {
    WPI_TalonSRX selectedMotor = getMotor(motor);
    selectedMotor.set(ControlMode.PercentOutput, speed);
  }

  public int getPosition(Motor motor) {
    WPI_TalonSRX selected = this.getMotor(motor);
    return selected.getSelectedSensorPosition();
  }
  public void setPosition(Motor motor, int position) {
    WPI_TalonSRX selected = this.getMotor(motor);
    selected.set(ControlMode.Position, position);
  }

  private WPI_TalonSRX getMotor(Motor motor) {
    WPI_TalonSRX selected = this.shoulderJointMotor;
    switch(motor) {
      case SHOULDER_JOINT:
        selected = this.shoulderJointMotor;
        break;
      case ELBOW_JOINT:
        selected = this.elbowJointMotor;
        break;
    }
    return selected;
  }
}