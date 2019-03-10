package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.helpers.RotationMotor;
import frc.robot.helpers.WristMotor;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmSubsystem extends Subsystem {
    // static variable that represents the drive train

  int initWristAngle = 180;
  public enum Motor {
    SHOULDER_JOINT, ELBOW_JOINT
  }
  private static ArmSubsystem instance;
  private RotationMotor shoulderJointMotor;
  private final double shoulderJointMotor_kP = 1.8;
  private final double shoulderJointMotor_kI = 0;//0.0015;
  private final double shoulderJointMotor_kD = 0;

  private RotationMotor elbowJointMotor;
  private final double elbowJointMotor_kP = 3.0;
  private final double elbowJointMotor_kI = 0.002;//0.0015;
  private final double elbowJointMotor_kD = 0.01;

  private WristMotor wristMotor;
  private final double wristJoint_kP = 0.005;
  private final double wristJoint_kI = 0.0002;//0.0015;
  private final double wristJoint_kD = 0;

  // always get the current instance of the drive train
  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  // going positive degrees is negative for quadrature

  // private initializer so you can't initialize more than 1 drive train
  private ArmSubsystem() {
    // set up the new arm motor
    shoulderJointMotor = new RotationMotor(RobotMap.ARM_SHOULDER_JOINT_MOTOR, 48.0 / 15.0);
    shoulderJointMotor.setSensorPhase(true);
    shoulderJointMotor.setInverted(true);
    shoulderJointMotor.positionInverted = false;
    shoulderJointMotor.configNominalOutputForward(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configNominalOutputReverse(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configPeakOutputForward(1.0, RobotMap.kPIDTimeoutMillis);
    shoulderJointMotor.configPeakOutputReverse(-1.0, RobotMap.kPIDTimeoutMillis);
    
    shoulderJointMotor.setk_P(this.shoulderJointMotor_kP);
    shoulderJointMotor.setk_I(this.shoulderJointMotor_kI);
    shoulderJointMotor.setk_D(this.shoulderJointMotor_kD);

    shoulderJointMotor.configAllowableClosedloopError(10, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    shoulderJointMotor.homeDegrees = 20;
    shoulderJointMotor.minDegrees = 20;
    shoulderJointMotor.maxDegrees = 335;
    shoulderJointMotor.initializeHome(20);

    elbowJointMotor = new RotationMotor(RobotMap.ARM_ELBOW_JOINT_MOTOR);
    elbowJointMotor.setInverted(false);
    elbowJointMotor.setSensorPhase(true);
    elbowJointMotor.positionInverted = true;
    elbowJointMotor.configNominalOutputForward(0, RobotMap.kPIDTimeoutMillis);
		elbowJointMotor.configNominalOutputReverse(0, RobotMap.kPIDTimeoutMillis);
		elbowJointMotor.configPeakOutputForward(0.6, RobotMap.kPIDTimeoutMillis);
    elbowJointMotor.configPeakOutputReverse(-0.6, RobotMap.kPIDTimeoutMillis);
    
    elbowJointMotor.setk_P(this.elbowJointMotor_kP);
    elbowJointMotor.setk_I(this.elbowJointMotor_kI);
    elbowJointMotor.setk_D(this.elbowJointMotor_kD);

    elbowJointMotor.configAllowableClosedloopError(10, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    
    elbowJointMotor.homeDegrees = 183;
    elbowJointMotor.initializeHome(183);

    wristMotor = new WristMotor(RobotMap.ARM_WRIST_JOINT_MOTOR, RobotMap.ARM_WRIST_JOINT_ENCODER_A, RobotMap.ARM_WRIST_JOINT_ENCODER_B);
    wristMotor.configPeakOutputForward(0.5);
    wristMotor.configPeakOutputReverse(-0.5);
    wristMotor.configNominalOutputForward(0);
    wristMotor.configNominalOutputReverse(0);
    wristMotor.setk_P(this.wristJoint_kP);
    wristMotor.setk_I(this.wristJoint_kI);
    wristMotor.setk_D(this.wristJoint_kD);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  public RotationMotor getRotationMotor(Motor motorType) {
    return this.rotationMotor(motorType);
  }

  public WristMotor getWristMotor() {
    return this.wristMotor;
  }

  public double getCurrent(Motor motor) {
    RotationMotor selectedMotor = rotationMotor(motor);
    return selectedMotor.getCurrent();

  }

  public void setSpeed(Motor motor, double speed) {
    RotationMotor selectedMotor = rotationMotor(motor);
    selectedMotor.setSpeed(speed);
  }

  public double getSpeed(Motor motor) {
    RotationMotor selectedMotor = rotationMotor(motor);
    return selectedMotor.getSpeed();
  }

  public int getPosition(Motor motor) {
    RotationMotor selected = this.rotationMotor(motor);
    return selected.getPosition();
  }
  public double getHomeDegrees(Motor motor) {
    RotationMotor selected = this.rotationMotor(motor);
    return selected.homeDegrees;
  }
  public int getTargetPosition(Motor motor) {
    RotationMotor selected = this.rotationMotor(motor);
    return selected.getTarget();
  }
  public void setDegrees(Motor motor, double degrees) {
    RotationMotor selected = this.rotationMotor(motor);
    selected.setDegrees(degrees);
  }
  public double getDegrees(Motor motor) {
    RotationMotor selected = this.rotationMotor(motor);
    return selected.getCurrentDegrees();
  }

  public void setPosition(Motor motor, int position) {
    WPI_TalonSRX selected = this.getMotor(motor);
    selected.set(ControlMode.Position, position);
  }

  public double getMotorOutputPercent(Motor motor) {
    WPI_TalonSRX selected = this.getMotor(motor);
    return selected.getMotorOutputPercent();
  }

  public double getAbsoluteElbowDegrees() {
    return this.getDegrees(Motor.ELBOW_JOINT) + this.getDegrees(Motor.SHOULDER_JOINT) - this.getHomeDegrees(Motor.SHOULDER_JOINT);
  }

  public double getAbsoluteWristDegrees() {
    return this.getWristMotor().getCurrentDegrees() + this.getAbsoluteElbowDegrees() - this.getHomeDegrees(Motor.ELBOW_JOINT);
  }
  
  public int getWristPosition() {
    return this.wristMotor.getPosition();
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

  private RotationMotor rotationMotor(Motor motor) {
    RotationMotor selected = this.shoulderJointMotor;
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