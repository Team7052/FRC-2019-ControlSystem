package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmSubsystem extends Subsystem {
    // static variable that represents the drive train
  public enum Motor {
    SHOULDER_JOINT
  }
  private static ArmSubsystem instance;
  private WPI_TalonSRX shoulderJointMotor;
  private TalonSRXPIDSetConfiguration shoulderJointPIDConfig;
  private final double shoulderJointMotor_kP = 1.0;
  private final double shoulderJointMotor_kI = 0.0;
  private final double shoulderJointMotor_kD = 0.0;
  private final double shoulderJointMotor_kF = 0.0;

  // always get the current instance of the drive train
  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  // private initializer so you can't initialize more than 1 drive train
  private ArmSubsystem() {
    // set up the new arm motor
    shoulderJointMotor = new WPI_TalonSRX(RobotMap.ARM_SHOULDER_JOINT_MOTOR);

    shoulderJointMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    
    shoulderJointMotor.config_kP(RobotMap.kPIDIdx, this.shoulderJointMotor_kP);
    shoulderJointMotor.config_kI(RobotMap.kPIDIdx, this.shoulderJointMotor_kI);
    shoulderJointMotor.config_kD(RobotMap.kPIDIdx, this.shoulderJointMotor_kD);
    shoulderJointMotor.config_kF(RobotMap.kPIDIdx, this.shoulderJointMotor_kF);

    shoulderJointMotor.configAllowableClosedloopError(0, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  public void setDegrees(Motor motor, double degrees) {
    WPI_TalonSRX selectedMotor = getMotor(motor);
    
    // convert to quadrature postition
    selectedMotor.setSelectedSensorPosition((int) degrees / 360 * 4096);
  }

  public void setSpeed(Motor motor, double speed) {
    WPI_TalonSRX selectedMotor = getMotor(motor);
    selectedMotor.set(ControlMode.PercentOutput, speed);
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