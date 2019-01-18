package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

class ArmSubsystem extends Subsystem {
    // static variable that represents the drive train
  private static ArmSubsystem instance;
  WPI_TalonSRX motor1;

  // always get the current instance of the drive train
  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  // private initializer so you can't initialize more than 1 drive train
  private ArmSubsystem() {
    motor1 = new WPI_TalonSRX(5);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  @Override 
  public void execute() {
      System.out.println("Execute");
  }
}