/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.joysticks.*;
import frc.robot.commands.arm.ArmControllerCommand;
import frc.robot.networking.Network;
import frc.robot.subsystems.ArmSubsystem;
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
  //RotateShoulderJoint armCommand = new RotateShoulderJoint();
  //TankDriveCommand driveCommand = new TankDriveCommand();
 //AutoCommand autoDrive = new AutoCommand();
  //DriveTenM driveTenCommand = new DriveTenM();
  //TankDriveCommand tankDriveCommand = new TankDriveCommand();
  CommandGroup newGroup;
  @Override
  public void robotInit() {
      //change Logitech to newly extended class
    oi = new Logitech(0);
<<<<<<< HEAD
=======
    newGroup = new CommandGroup();
    armCommand = new ArmControllerCommand();
    newGroup.addParallel(armCommand);
    newGroup.addParallel(driveCommand);
    Network network = Network.getInstance();
    armCommand.elbowCommand.delegate = network;
    armCommand.shoulderCommand.delegate = network;
    armCommand.wristCommand.delegate = network;

    testManager = TestManager.getInstance();
>>>>>>> a40439eb926ff0b1d5034ed553479b506d9fafea
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
<<<<<<< HEAD
=======
    Scheduler.getInstance().removeAll();
    testManager.setState(TestManagerState.IDLE);
    calibrated = false;
>>>>>>> a40439eb926ff0b1d5034ed553479b506d9fafea
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
<<<<<<< HEAD

=======
    testManager.setState(TestManagerState.IDLE);
    Scheduler.getInstance().removeAll();
    calibrated = false;
>>>>>>> a40439eb926ff0b1d5034ed553479b506d9fafea
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  ArmSubsystem arm;
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
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
<<<<<<< HEAD
  public void testPeriodic() {
=======
  public void testInit() {
    testManager.setState(TestManagerState.IDLE);
  }
  
  boolean calibrated = false;
  @Override
  public void testPeriodic() {
    testManager.update();
>>>>>>> a40439eb926ff0b1d5034ed553479b506d9fafea
  }
}
