package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.helpers.IRotationMotor;
import frc.robot.helpers.RotationTalonSRX;
import frc.robot.helpers.RotationVictorSPX;
import frc.robot.states.ArmSuperState;

public class ArmSubsystem extends Subsystem {
  // static variable that represents the drive train
  public enum Motor {
    SHOULDER_JOINT, ELBOW_JOINT, WRIST_JOINT
  }

  private ArmSuperState superState;

  private static ArmSubsystem instance;
  public static ArmSubsystem getInstance() {
    if (instance == null) instance = new ArmSubsystem();
    return instance;
  }

  private RotationTalonSRX shoulderJointMotor;
  private final double shoulderJointMotor_kP = 1.8;
  private final double shoulderJointMotor_kI = 0; //0.0015;
  private final double shoulderJointMotor_kD = 0;

  private RotationTalonSRX elbowJointMotor;
  private final double elbowJointMotor_kP = 3.0;
  private final double elbowJointMotor_kI = 0.002;//0.0015;
  private final double elbowJointMotor_kD = 0.01;

  private RotationVictorSPX wristMotor;
  private final double wristJoint_kP = 0.005;
  private final double wristJoint_kI = 0.0002;//0.0015;
  private final double wristJoint_kD = 0;

  // private initializer so you can't initialize more than 1 drive train
  private ArmSubsystem() {
    // set up the new arm motor
    shoulderJointMotor = new RotationTalonSRX(RobotMap.kArmShoulderMotor, RobotMap.kArmShoulderMotorGearRatio);
    elbowJointMotor = new RotationTalonSRX(RobotMap.kArmElbowMotor);
    wristMotor = new RotationVictorSPX(RobotMap.kArmWristMotor, RobotMap.kArmWristMotorEncoderA, RobotMap.kArmWristMotorEncoderB);

    shoulderJointMotor.setSensorPhase(true);
    shoulderJointMotor.setInverted(true);
    shoulderJointMotor.setInvertedPosition(false);
    shoulderJointMotor.configNominalOutputForward(0, shoulderJointMotor.slotIdx);
		shoulderJointMotor.configNominalOutputReverse(0, shoulderJointMotor.slotIdx);
		shoulderJointMotor.configPeakOutputForward(1.0, shoulderJointMotor.slotIdx);
    shoulderJointMotor.configPeakOutputReverse(-1.0, shoulderJointMotor.slotIdx);
    
    shoulderJointMotor.set_kp(this.shoulderJointMotor_kP);
    shoulderJointMotor.set_ki(this.shoulderJointMotor_kI);
    shoulderJointMotor.set_kd(this.shoulderJointMotor_kD);

    shoulderJointMotor.configAllowableClosedloopError(10, shoulderJointMotor.slotIdx, shoulderJointMotor.timeoutMs);
    shoulderJointMotor.setHomeDegrees(20);
    shoulderJointMotor.setDegreesLimits(20, 335);

    elbowJointMotor.setInverted(false);
    elbowJointMotor.setSensorPhase(true);
    elbowJointMotor.setInvertedPosition(true);
    elbowJointMotor.configNominalOutputForward(0, elbowJointMotor.timeoutMs);
		elbowJointMotor.configNominalOutputReverse(0, elbowJointMotor.timeoutMs);
		elbowJointMotor.configPeakOutputForward(0.6, elbowJointMotor.timeoutMs);
    elbowJointMotor.configPeakOutputReverse(-0.6, elbowJointMotor.timeoutMs);

    
    elbowJointMotor.set_kp(this.elbowJointMotor_kP);
    elbowJointMotor.set_ki(this.elbowJointMotor_kI);
    elbowJointMotor.set_kd(this.elbowJointMotor_kD);

    elbowJointMotor.configAllowableClosedloopError(10, elbowJointMotor.slotIdx, elbowJointMotor.timeoutMs);
    
    elbowJointMotor.setHomeDegrees(187);
    elbowJointMotor.setDegreesLimits(-140, 190);

    wristMotor.configPeakOutputForward(0.5);
    wristMotor.configPeakOutputReverse(-0.5);
    wristMotor.configNominalOutputForward(0);
    wristMotor.configNominalOutputReverse(0);
    wristMotor.set_kp(this.wristJoint_kP);
    wristMotor.set_ki(this.wristJoint_kI);
    wristMotor.set_kd(this.wristJoint_kD);

    wristMotor.setInverted(false);
    wristMotor.setSensorPhase(false);
    wristMotor.setInvertedPosition(true);

    wristMotor.setHomeDegrees(160);
    wristMotor.setDegreesLimits(0, 360);

    superState = new ArmSuperState();
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

  public ArmSuperState getSuperState() {
    return this.superState;
  }
  /* Getters */
  public double getSpeed(Motor motor) {
    return this.getMotor(motor).getPercentOutput();
  }
  public int getTargetPosition(Motor motor) {
    return this.getMotor(motor).getTargetPosition();
  }
  public int getPosition(Motor motor) {
    return this.getMotor(motor).getPosition();
  }
  public int getRawVelocity(Motor motor) {
    return this.getMotor(motor).getRawVelocity();
  }
  public double getHomeDegrees(Motor motor) {
    return this.getMotor(motor).getHomeDegrees();
  }
  public double getDegrees(Motor motor) {
    return this.getMotor(motor).getDegrees();
  }
  public double getAbsoluteDegrees(Motor motor) {
    if (motor == Motor.WRIST_JOINT) {
      return getAbsoluteWristDegrees();
    }
    else if (motor == Motor.ELBOW_JOINT) {
      return getAbsoluteElbowDegrees();
    }
    return this.getDegrees(motor);
  }
  public double getVelocityDegrees(Motor motor) {
    return this.getMotor(motor).getVelocityDegrees();
  }
  public double getCurrent(Motor motor) {
    return this.getMotor(motor).getCurrent();
  }
  public double getMotorOutputPercent(Motor motor) {
    return this.getMotor(motor).getPercentOutput();
  }
  public double getMotorOutputVoltage(Motor motor) {
    return this.getMotor(motor).getVoltage();
  }
  public boolean getInverted(Motor motor) {
    return this.getMotor(motor).getInverted();
  }
  public boolean getPhase(Motor motor) {
    return this.getMotor(motor).getSensorPhase();
  }

  /* Setters */
  public void setSpeed(Motor motor, double speed) {
    this.getMotor(motor).setPercentOutput(speed);
  }
  public void setPosition(Motor motor, int position) {
    this.getMotor(motor).setPosition(position);
  }
  public void setDegrees(Motor motor, double degrees) {
    this.getMotor(motor).setDegrees(degrees);
  }
  public void setIntegralAccumulator(Motor motor, double value) {
    this.getMotor(motor).setIntegralValue(value);
  }

  /* private getters */
  private double getAbsoluteElbowDegrees() {
    return this.getDegrees(Motor.ELBOW_JOINT) + this.getDegrees(Motor.SHOULDER_JOINT) - this.getHomeDegrees(Motor.SHOULDER_JOINT);
  }
  private double getAbsoluteWristDegrees() {
    return this.getDegrees(Motor.WRIST_JOINT) + this.getAbsoluteElbowDegrees() - this.getHomeDegrees(Motor.ELBOW_JOINT);
  }

  private IRotationMotor getMotor(Motor motor) {
    switch(motor) {
      case SHOULDER_JOINT:
        return this.shoulderJointMotor;
      case ELBOW_JOINT:
        return this.elbowJointMotor;
      case WRIST_JOINT:
        return this.wristMotor;
      default:
        return null;
    }
  }
}