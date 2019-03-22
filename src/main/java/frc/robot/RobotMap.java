/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // PWM ports for front right motor
  public static final int kFrontRightMotor = 1;
  public static final int kBackRightMotor = 0;
  public static final int kBackLeftMotor = 2;
  public static final int kFrontLeftMotor = 3;

  public static final int kArmShoulderMotor = 5;
  public static final double kArmShoulderMotorGearRatio = 48.0 / 15.0;

  public static final int kArmElbowMotor = 6;
  
  public static final int kArmWristMotor = 7;
  public static int kArmWristMotorEncoderA = 8;
  public static int kArmWristMotorEncoderB = 9;

  // names for 
  public static final String kArmShoulderMotorName = "armShoulderMotor";
  public static final String kArmElbowMotorName = "armElbowMotor";
  public static final String kArmWristMotorName = "armWristMotor";
}
