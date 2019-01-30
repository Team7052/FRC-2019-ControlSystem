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
  public static int frontRightMotor = 0;
  public static int backRightMotor = 1;
  public static int backLeftMotor = 2;
  public static int frontLeftMotor = 3;

  public static int ARM_SHOULDER_JOINT_MOTOR = 5;
  public static int ARM_SHOULDER_JOINT_ENCODER = 0;
  public static int ARM_SHOULDER_JOINT_SLOT_IDX = 0;

  public static int kPIDIdx = 0;
  public static int kPIDTimeoutMillis = 30;
}
