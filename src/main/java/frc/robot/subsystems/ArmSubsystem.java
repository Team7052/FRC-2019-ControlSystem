package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.states.ArmSuperState;
import frc.robot.states.ArmSuperState.ArmState;
import frc.robot.helpers.RotationMotor;
import frc.robot.helpers.Triplet;
import frc.robot.helpers.WristMotor;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.sequencing.Sequence;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmSubsystem extends Subsystem {
    // static variable that represents the drive train

  int initWristAngle = 180;
  public enum Motor {
    SHOULDER_JOINT, ELBOW_JOINT, WRIST_JOINT
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

  private ArmSuperState armSuperState;

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
    armSuperState = new ArmSuperState();
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
    
    elbowJointMotor.maxDegrees = 190;
    elbowJointMotor.minDegrees = -140;
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
  }

  public ArmState getState() {
    return this.armSuperState.getState();
  }
  public Triplet<Sequence<MotionTriplet>> setState(ArmState newState) {
    return this.armSuperState.setState(newState);
  }

  public double getSpeed(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.getSpeed();
    RotationMotor selectedMotor = getMotor(motor);
    return selectedMotor.getSpeed();
  }
  public void setSpeed(Motor motor, double speed) {
    if (isWrist(motor)) {
      wristMotor.setSpeed(speed);
      return;
    }
    RotationMotor selectedMotor = getMotor(motor);
    selectedMotor.setSpeed(speed);
  }

  public int getTargetPosition(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.getTarget();
    RotationMotor selected = this.getMotor(motor);
    return selected.getTarget();
  }
  public int getPosition(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.getPosition();
    RotationMotor selected = this.getMotor(motor);
    return selected.getPosition();
  }
  public void setPosition(Motor motor, int position) {
    if (this.isWrist(motor)) return;
    RotationMotor selected = this.getMotor(motor);
    selected.set(ControlMode.Position, position);
  }

  public int getRawVelocity(Motor motor) {
    if (this.isWrist(motor)) return (int) wristMotor.getRawVelocity();
    RotationMotor selected = this.getMotor(motor);
    return selected.getRawVelocity();
  }


  public double getHomeDegrees(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.homeDegrees;
    RotationMotor selected = this.getMotor(motor);
    return selected.homeDegrees;
  }

  public void setDegrees(Motor motor, double degrees) {
    if (this.isWrist(motor)) {
      wristMotor.setDegrees(degrees);
      return;
    }
    RotationMotor selected = this.getMotor(motor);
    selected.setDegrees(degrees);
  }
  public double getDegrees(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.getCurrentDegrees();
    RotationMotor selected = this.getMotor(motor);
    return selected.getCurrentDegrees();
  }
  public double getVelocityDegrees(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.getVelocityDegrees();
    RotationMotor selected = this.getMotor(motor);
    return selected.getVelocityDegrees();
  }

  public double getCurrent(Motor motor) {
    if (isWrist(motor)) return wristMotor.getCurrent();
    RotationMotor selectedMotor = getMotor(motor);
    return selectedMotor.getCurrent();
  }
  public double getMotorOutputPercent(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.getMotorOutputPercent();
    WPI_TalonSRX selected = this.getMotor(motor);
    return selected.getMotorOutputPercent();
  }
  public double getMotorOutputVoltage(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.getMotorOutputVoltage();
    RotationMotor selected = this.getMotor(motor);
    return selected.getMotorOutputVoltage();
  }

  public boolean getPhase(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.getPhase();
    return this.getMotor(motor).getPhase();
  }

  public boolean isInverted(Motor motor) {
    if (this.isWrist(motor)) return wristMotor.getInverted();
    return this.getMotor(motor).getInverted();
  }

  public double getAbsoluteDegrees(Motor motor) {
    if (isWrist(motor)) {
      return getAbsoluteWristDegrees();
    }
    else if (motor == Motor.ELBOW_JOINT) {
      return getAbsoluteElbowDegrees();
    }
    return this.getDegrees(motor);
  }

  public void setIntegralAccumulator(Motor motor, double value) {
    if (isWrist(motor)) {
      wristMotor.setIntegralValue(value);
      return;
    }
    this.getMotor(motor).setIntegralAccumulator(value);
  }

  private double getAbsoluteElbowDegrees() {
    return this.getDegrees(Motor.ELBOW_JOINT) + this.getDegrees(Motor.SHOULDER_JOINT) - this.getHomeDegrees(Motor.SHOULDER_JOINT);
  }

  private double getAbsoluteWristDegrees() {
    return this.wristMotor.getCurrentDegrees() + this.getAbsoluteElbowDegrees() - this.getHomeDegrees(Motor.ELBOW_JOINT);
  }

  private boolean isWrist(Motor motor) {
    return motor == Motor.WRIST_JOINT;
  }


  private RotationMotor getMotor(Motor motor) {
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