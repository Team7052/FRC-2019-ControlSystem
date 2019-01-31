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
    SHOULDER_JOINT
  }
  private static ArmSubsystem instance;
  private WPI_TalonSRX shoulderJointMotor;
  private final double shoulderJointMotor_kP = 0.4;
  private final double shoulderJointMotor_kI = 0;//0.0015;
  private final double shoulderJointMotor_kD = 0.2;
  private final double shoulderJointMotor_kF = 0.0;

  MotionProfiler motionProfiler;

  // always get the current instance of the drive train
  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  // home position is motor as of prototype one of the arm
  public final int homePosition = 3883;
  // going positive degrees is negative for quadrature
  int spinDirection = -1;

  int targetPosition;

  // private initializer so you can't initialize more than 1 drive train
  private ArmSubsystem() {
    // set up the new arm motor
    this.targetPosition = homePosition;

    shoulderJointMotor = new WPI_TalonSRX(RobotMap.ARM_SHOULDER_JOINT_MOTOR);

    shoulderJointMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    
    shoulderJointMotor.configNominalOutputForward(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configNominalOutputReverse(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configPeakOutputForward(0.3, RobotMap.kPIDTimeoutMillis);
    shoulderJointMotor.configPeakOutputReverse(-0.3, RobotMap.kPIDTimeoutMillis);
    
    shoulderJointMotor.config_kP(RobotMap.kPIDIdx, this.shoulderJointMotor_kP);
    shoulderJointMotor.config_kI(RobotMap.kPIDIdx, this.shoulderJointMotor_kI);
    shoulderJointMotor.config_kD(RobotMap.kPIDIdx, this.shoulderJointMotor_kD);
    shoulderJointMotor.config_kF(RobotMap.kPIDIdx, this.shoulderJointMotor_kF);

    int absolutePosition = shoulderJointMotor.getSensorCollection().getPulseWidthPosition();
    shoulderJointMotor.setSelectedSensorPosition(absolutePosition, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);

    shoulderJointMotor.configAllowableClosedloopError(10, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  int loop = 0;
  public void setDegrees(Motor motor, double degrees) {    
    // convert to quadrature postition
    int quadratureDegrees = (int) Math.round(degrees / 360.0 * 4096.0) * spinDirection; // convert to a quadrature
    if (this.targetPosition != homePosition + quadratureDegrees) {
      this.targetPosition = homePosition + quadratureDegrees;
    }
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

  public void setPosition(Motor motor, int targetPosition) {
    WPI_TalonSRX selectedMotor = getMotor(motor);
    selectedMotor.set(ControlMode.Position, targetPosition);
  }

  private void setTarget(int newPosition) {
    if (this.targetPosition != newPosition) {
      this.targetPosition = newPosition;
      
    }
  }

  public int getPosition(Motor motor) {
    WPI_TalonSRX selected = this.shoulderJointMotor;
    return selected.getSelectedSensorPosition();
  }

  private WPI_TalonSRX getMotor(Motor motor) {
    WPI_TalonSRX selected = this.shoulderJointMotor;
    switch(motor) {
      case SHOULDER_JOINT:
        selected = this.shoulderJointMotor;
        break;
    }
    return selected;
  }
}