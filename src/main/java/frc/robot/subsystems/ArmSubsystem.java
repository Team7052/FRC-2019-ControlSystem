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
    jointBaseMotor
  }
  private static ArmSubsystem instance;
  private WPI_TalonSRX jointBaseMotor;
  private TalonSRXPIDSetConfiguration jointBasePIDConfig;
  private final double jointBaseMotor_kP = 1.0;
  private final double jointBaseMotor_kI = 0.0;
  private final double jointBaseMotor_kD = 0.0;
  private final double jointBaseMotor_kF = 0.0;

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
    jointBaseMotor = new WPI_TalonSRX(RobotMap.ARM_JOINT_BASE_MOTOR);

    jointBasePIDConfig = new TalonSRXPIDSetConfiguration();
    jointBasePIDConfig.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    jointBasePIDConfig.selectedFeedbackCoefficient = RobotMap.ARM_JOINT_BASE_ENCODER;

    jointBaseMotor.configurePID(jointBasePIDConfig);
    jointBaseMotor.config_kP(RobotMap.ARM_JOINT_BASE_SLOT_IDX, this.jointBaseMotor_kP);
    jointBaseMotor.config_kI(RobotMap.ARM_JOINT_BASE_SLOT_IDX, this.jointBaseMotor_kI);
    jointBaseMotor.config_kD(RobotMap.ARM_JOINT_BASE_SLOT_IDX, this.jointBaseMotor_kD);
    jointBaseMotor.config_kF(RobotMap.ARM_JOINT_BASE_SLOT_IDX, this.jointBaseMotor_kF);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  public void setDegrees(Motor motor, double degrees) {
    WPI_TalonSRX selectedMotor;
    switch(motor) {
      case jointBaseMotor:
        selectedMotor = this.jointBaseMotor;
        break;
      default:
        return;
    }
    
    selectedMotor.setSelectedSensorPosition((int) degrees / 360 * 4096);
  }

  public void setSpeed(Motor motor, double speed) {
    WPI_TalonSRX selectedMotor;
    switch(motor) {
      case jointBaseMotor:
        selectedMotor = this.jointBaseMotor;
        break;
      default:
        return;
    }
    selectedMotor.set(ControlMode.PercentOutput, speed);
  }
}