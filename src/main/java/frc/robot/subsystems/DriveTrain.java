/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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
    if (instance == null) {
      instance = new DriveTrain();
    }
    return instance;
  }

  private Spark frontLeftMotor;
  private Spark backLeftMotor;
  private Spark frontRightMotor;
  private Spark backRightMotor;

  private SpeedControllerGroup leftSpeedGroup;
  private SpeedControllerGroup rightSpeedGroup;

  private Encoder leftEncoder;
  private Encoder rightEncoder;

  private PigeonIMU imuSensor;

  // private initializer so you can't initialize more than 1 drive train
  private DriveTrain() {
    frontLeftMotor = new Spark(RobotMap.kFrontLeftMotor);
    backLeftMotor = new Spark(RobotMap.kBackLeftMotor);
    frontRightMotor = new Spark(RobotMap.kFrontRightMotor);
    backRightMotor = new Spark(RobotMap.kBackRightMotor);

    rightSpeedGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    leftSpeedGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
    rightSpeedGroup.setInverted(true);

    leftEncoder = new Encoder(2, 3, false, EncodingType.k1X);
    rightEncoder = new Encoder(0, 1, true, EncodingType.k1X);

    //imuSensor = new PigeonIMU(RobotMap.kPigeonSensor);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  public void setLeftGroupSpeed(double speed) {
    leftSpeedGroup.set(speed);
  }
  public void setRightGroupSpeed(double speed) {
    rightSpeedGroup.set(speed);
  }
  public double getLeftSpeed() {
    return leftSpeedGroup.get();
  }
  public double getRightSpeed() {
    return rightSpeedGroup.get();
  }

  public int getLeftDisplacement() {
    return this.leftEncoder.get();
  } 
  public int getRightDisplacement() {
    return this.rightEncoder.get();
  }

  public double getLeftVelocity() {
    return this.leftEncoder.getRate();
  }
  public double getRightVelocity() {
    return this.rightEncoder.getRate();
  }

  public void resetAngle() {
    this.imuSensor.setYaw(0);
  }

  public double getAngle() {
    double[] ypr = new double[3];
    this.imuSensor.getYawPitchRoll(ypr);
    return ypr[0] / 180.0 * Math.PI;
  }

  public boolean imuSensorIsConnectedAndCalibrated() {
    return this.imuSensor.getState() == PigeonState.Ready;
  }
}
