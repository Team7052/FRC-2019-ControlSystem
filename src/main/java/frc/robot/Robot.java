/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //WPI_TalonSRX motor1;
  public static OI oi;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  Spark frontLeftMotor;
  Spark backRightMotor;
  Spark frontRightMotor;
  Spark backLeftMotor;
  Joystick joystick;
  SpeedControllerGroup leftMotorGroup;
  SpeedControllerGroup rightMotorGroup;



  @Override
  public void robotInit() {
   // m_oi = new OI(0);
    frontRightMotor = new Spark(0);
    backRightMotor = new Spark(1);
    frontLeftMotor = new Spark(3);
    backLeftMotor = new Spark(2);

    frontLeftMotor.setInverted(true);

    rightMotorGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
    leftMotorGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    joystick = new Joystick(0);
    
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    //runTankDrive();
    runTriggerDrive();
    /* 
    Scheduler.getInstance().run();
		if (joystick.getRawAxis(0) > 0.5) {
			sparkmotor.set(0.8);
		}
		else if (joystick.getRawAxis(0) < -0.5) {
			sparkmotor.set(-0.8);
		}
		else {
			sparkmotor.set(0.0);
		}
		if (joystick.getRawAxis(1)) {
			
		}
    */
  }

  public void runTriggerDrive(){
    double x = joystick.getRawAxis(0);
    double r2 = joystick.getRawAxis(7);
    double l2 = joystick.getRawAxis(6);
    double forwardSpeed = Math.atan(r2);
    double backwardSpeed = -Math.atan(l2);
    double leftSpeed = forwardSpeed+backwardSpeed;
    double rightSpeed = forwardSpeed+backwardSpeed;

    if (x > 0.1) {
        rightSpeed= rightSpeed*(1-x);
    }
    if(x<-0.1){
        leftSpeed = leftSpeed*(1+x);
    }

    if ((Math.abs(l2)-Math.abs(r2))<0.1){
      if(x>0.1){
        rightSpeed = -0.5;
        leftSpeed = 0.5;
      }
      if(x<-0.1){
        rightSpeed = 0.5;
        leftSpeed = -0.5;
      }
    }
    leftMotorGroup.set(leftSpeed);
    rightMotorGroup.set(rightSpeed);
  }
  public void runTankDrive(){
    Scheduler.getInstance().run();
    double y = joystick.getRawAxis(1);
    double x = joystick.getRawAxis(0);
    
    double leftSpeed = y * 0.7;
    double rightSpeed = y * 0.7;
    if (Math.abs(x) > 0.1) {
      if (x > 0) {
        rightSpeed = 0;
        if (Math.abs(leftSpeed) < 0.1) {
          rightSpeed = 0.5;
          leftSpeed = -0.5;
        }
      }
      else if (x < 0) {
        leftSpeed = 0;
        if (Math.abs(leftSpeed) < 0.1) {
          rightSpeed = -0.5;
          leftSpeed = 0.5;
        }
      }
    }

    leftMotorGroup.set(leftSpeed);
    rightMotorGroup.set(rightSpeed);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
