/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
  // static variable that represents the drive train
  private static DriveTrain instance;

  // always get the current instance of the drive train
  public static DriveTrain getInstance() {
    System.out.println(instance);
    if (instance == null) {
      instance = new DriveTrain();
    }
    return instance;
  }

  private WPI_VictorSPX frontLeftMotor;
  private WPI_VictorSPX backLeftMotor;
  private WPI_VictorSPX frontRightMotor;
  private WPI_VictorSPX backRightMotor;

  private SpeedControllerGroup leftSpeedGroup;
  private SpeedControllerGroup rightSpeedGroup;

  // private initializer so you can't initialize more than 1 drive train
  private DriveTrain() {
    frontLeftMotor = new WPI_VictorSPX(RobotMap.frontLeftMotor);
    backLeftMotor = new WPI_VictorSPX(RobotMap.backLeftMotor);
    frontRightMotor = new WPI_VictorSPX(RobotMap.frontRightMotor);
    backRightMotor = new WPI_VictorSPX(RobotMap.backRightMotor);

    leftSpeedGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    rightSpeedGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
    rightSpeedGroup.setInverted(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  /* public methods callable by commands */
  public void setLeftGroupSpeed(double speed) {
    leftSpeedGroup.set(speed);
  }
  public void setRightGroupSpeed(double speed) {
    rightSpeedGroup.set(speed);
  }
}
