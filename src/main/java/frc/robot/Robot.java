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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //WPI_TalonSRX motor1;
  public static OI m_oi;

<<<<<<< HEAD
  /*int motor = 0;
	int kevingay = 1;
	int calebcewl = 2;
	int kevingaymore = 3;
	Spark sparkmotor;
	Spark sparkkevingay;
	Spark sparkcalebcewl;
	Spark sparkkevingaymore;
	SpeedControllerGroup left;
	SpeedControllerGroup right;*/

=======
>>>>>>> 5cf1c77cbe3291c63a7643ced3e6192b270d2398
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  Spark leftMotor;
  Spark rightMotor;
  Joystick joystick;

  @Override
  public void robotInit() {
   // m_oi = new OI(0);
    rightMotor = new Spark(0);
    leftMotor = new Spark(3);
    joystick = new Joystick(0);
<<<<<<< HEAD

    
      /*sparkmotor = new Spark(motor);
		  sparkkevingay = new Spark(kevingay);
		  sparkcalebcewl = new Spark(calebcewl);
		sparkkevingaymore = new Spark(kevingaymore);
		
		right = new SpeedControllerGroup(sparkkevingay, sparkmotor);
		left = new SpeedControllerGroup(sparkcalebcewl,sparkkevingaymore);
    */
=======
    drive = new DifferentialDrive(leftMotor, rightMotor);

    
    
>>>>>>> 5cf1c77cbe3291c63a7643ced3e6192b270d2398
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
    double y = joystick.getRawAxis(1);
    double x = joystick.getRawAxis(2);
<<<<<<< HEAD
    
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

    leftMotor.set(-leftSpeed);
    rightMotor.set(rightSpeed);
    
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
=======
    drive.tankDrive(x, y);
>>>>>>> 5cf1c77cbe3291c63a7643ced3e6192b270d2398
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
