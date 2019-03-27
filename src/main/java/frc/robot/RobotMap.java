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
  public static final int kArmWristMotorEncoderA = 8;
  public static final int kArmWristMotorEncoderB = 9;

  public static final int kClimberClawMotor = 13;
  public static final int kClimberClawMotorEncoderA = 4;
  public static final int kClimberClawMotorEncoderB = 5;
  public static final int kClimberClawMotorEncoderTicks = 1024;
  public static final double kClimberClawMotorGearRatio = 2.0;

  public static final int kClimberRackMotor = 12;
  public static final double kClimberRackPitchDiameter = 1.1;

  public static final int kPigeonSensor = 20;

  // names for 
  public static final String kArmShoulderMotorName = "armShoulderMotor";
  public static final String kArmElbowMotorName = "armElbowMotor";
  public static final String kArmWristMotorName = "armWristMotor";
}
