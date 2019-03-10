/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.joysticks.*;
import frc.robot.commands.arm.ArmControllerCommand;
import frc.robot.motionProfiling.Point;
import frc.robot.networking.Network;
import frc.robot.commands.FollowSplineCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.tests.TestManager;
import frc.robot.tests.TestManagerState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static OI oi;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  ArmControllerCommand armCommand;
  TankDriveCommand driveCommand;
  FollowSplineCommand autoCommand;

  CommandGroup newGroup;
  @Override
  public void robotInit() {
      //change Logitech to newly extended class
    oi = new Logitech(0);
    newGroup = new CommandGroup();
    armCommand = new ArmControllerCommand();
    driveCommand = new TankDriveCommand();
    newGroup.addParallel(armCommand);
    newGroup.addParallel(driveCommand);
    Network network = Network.getInstance();
    armCommand.elbowCommand.delegate = network;
    armCommand.shoulderCommand.delegate = network;
    armCommand.wristCommand.delegate = network;

    Point[] path = {
      new Point (0, 0), new Point (2, 1), new Point (4, 4), new Point (6, 2)
    };
    autoCommand = new FollowSplineCommand(new ArrayList<>(Arrays.asList(path)), 5.0);

  }

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
    Scheduler.getInstance().removeAll();
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
    Scheduler.getInstance().removeAll();
    Scheduler.getInstance().add(autoCommand);
    calibrated = false;
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
    
    Scheduler.getInstance().removeAll();
    Scheduler.getInstance().add(newGroup);
    calibrated = false;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testInit() {
  }
  
  boolean calibrated = false;
  @Override
  public void testPeriodic() {
  }
}
