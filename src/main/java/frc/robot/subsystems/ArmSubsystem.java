package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.helpers.RotationMotor;
import frc.robot.motionProfiling.MotionProfiler;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

public class ArmSubsystem extends Subsystem {
    // static variable that represents the drive train
  public enum Motor {
    SHOULDER_JOINT, ELBOW_JOINT
  }
  private static ArmSubsystem instance;
  private RotationMotor shoulderJointMotor;
  private final double shoulderJointMotor_kP = 1.4;
  private final double shoulderJointMotor_kI = 0;//0.0015;
  private final double shoulderJointMotor_kD = 0;
  private final double shoulderJointMotor_kF = 0.0;

  private RotationMotor elbowJointMotor;
  private final double elbowJointMotor_kP = 1.4;
  private final double elbowJointMotor_kI = 0;//0.0015;
  private final double elbowJointMotor_kD = 0;
  private final double elbowJointMotor_kF = 0.0;

  private VictorSPX wristMotor;

  private AHRS imuSensor;

  MotionProfiler motionProfiler;

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
    shoulderJointMotor.positionInverted = true;
    shoulderJointMotor.configNominalOutputForward(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configNominalOutputReverse(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configPeakOutputForward(0.8, RobotMap.kPIDTimeoutMillis);
    shoulderJointMotor.configPeakOutputReverse(-0.8, RobotMap.kPIDTimeoutMillis);
    
    shoulderJointMotor.setk_P(this.shoulderJointMotor_kP);
    shoulderJointMotor.setk_I(this.shoulderJointMotor_kI);
    shoulderJointMotor.setk_D(this.shoulderJointMotor_kD);

    shoulderJointMotor.configAllowableClosedloopError(10, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    System.out.println("Shoulder joint motor: " + this.shoulderJointMotor.getInvertedPosition());
    shoulderJointMotor.homeDegrees = 35;
    shoulderJointMotor.minDegrees = 35;
    shoulderJointMotor.maxDegrees = 340;
    shoulderJointMotor.initializeHome(35);

    elbowJointMotor = new RotationMotor(RobotMap.ARM_ELBOW_JOINT_MOTOR);
    elbowJointMotor.setInverted(false);
    elbowJointMotor.setSensorPhase(true);
    elbowJointMotor.positionInverted = false;
    elbowJointMotor.configNominalOutputForward(0, RobotMap.kPIDTimeoutMillis);
		elbowJointMotor.configNominalOutputReverse(0, RobotMap.kPIDTimeoutMillis);
		elbowJointMotor.configPeakOutputForward(0.4, RobotMap.kPIDTimeoutMillis);
    elbowJointMotor.configPeakOutputReverse(-0.4, RobotMap.kPIDTimeoutMillis);
    
    elbowJointMotor.config_kP(RobotMap.kPIDIdx, this.elbowJointMotor_kP);
    elbowJointMotor.config_kI(RobotMap.kPIDIdx, this.elbowJointMotor_kI);
    elbowJointMotor.config_kD(RobotMap.kPIDIdx, this.elbowJointMotor_kD);
    elbowJointMotor.config_kF(RobotMap.kPIDIdx, this.elbowJointMotor_kF);

    elbowJointMotor.configAllowableClosedloopError(10, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    
    elbowJointMotor.homeDegrees = 190;
    elbowJointMotor.initializeHome(190);

    wristMotor = new VictorSPX(7);
    wristMotor.configPeakOutputForward(0.3);
    wristMotor.configPeakOutputReverse(-0.3);
    wristMotor.configNominalOutputForward(0);
    wristMotor.configNominalOutputReverse(0);
    imuSensor = new AHRS(I2C.Port.kOnboard);
    wristMotor.setInverted(true);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  public AHRS getIMUSensor() {
    return this.imuSensor;
  }

  public RotationMotor getRotationMotor(Motor motorType) {
    return this.rotationMotor(motorType);
  }

  public VictorSPX getWristMotor() {
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