/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TankDriveCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
  // static variable that represents the drive train
  private static DriveTrain instance;

  // always get the current instance of the drive train
  public static DriveTrain getInstance() {
    if (DriveTrain.instance == null) {
      return new DriveTrain();
    }
    return DriveTrain.instance;
  }

  private Spark frontLeftMotor;
  private Spark backLeftMotor;
  private Spark frontRightMotor;
  private Spark backRightMotor;

  private SpeedControllerGroup leftSpeedGroup;
  private SpeedControllerGroup rightSpeedGroup;

  // private initializer so you can't initialize more than 1 drive train
  private DriveTrain() {
    frontLeftMotor = new Spark(RobotMap.frontLeftMotor);
    backLeftMotor = new Spark(RobotMap.backLeftMotor);
    frontRightMotor = new Spark(RobotMap.frontRightMotor);
    backRightMotor = new Spark(RobotMap.backRightMotor);

    leftSpeedGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    rightSpeedGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    TankDriveCommand driveCommand = new TankDriveCommand();
    // define the Trigger drive here
    setDefaultCommand(driveCommand);
  }

  /* public methods callable by commands */
  public void setLeftGroupSpeed(double speed) {
    leftSpeedGroup.set(speed);
  }
  public void setRightGroupSpeed(double speed) {
    rightSpeedGroup.set(speed);
  }
}
