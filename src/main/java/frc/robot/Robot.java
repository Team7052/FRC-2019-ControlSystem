/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.joysticks.*;
import frc.robot.commands.ArmControllerCommand;
import frc.robot.commands.RotateShoulderJoint;
import frc.robot.commands.TankDriveCommand;
import frc.robot.networking.Network;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;
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
  TankDriveCommand driveCommand = new TankDriveCommand();
  ArmControllerCommand armCommand;
  //DriveTenM driveTenCommand = new DriveTenM();
  //TankDriveCommand tankDriveCommand = new TankDriveCommand();
  CommandGroup newGroup;

  TestManager testManager;

  @Override
  public void robotInit() {
      //change Logitech to newly extended class
    oi = new Logitech(0);
    newGroup = new CommandGroup();
    armCommand = new ArmControllerCommand();
    newGroup.addParallel(armCommand);
    newGroup.addParallel(driveCommand);

    testManager = TestManager.getInstance();
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
    Network.getInstance().sendDataToServer();
  }

  @Override
  public void disabledInit() {
    Scheduler.getInstance().removeAll();
    testManager.setState(TestManagerState.IDLE);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    testManager.setState(TestManagerState.IDLE);
    Scheduler.getInstance().removeAll();
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    testManager.setState(TestManagerState.IDLE);
    Scheduler.getInstance().removeAll();
    Scheduler.getInstance().add(newGroup);
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
    testManager.setState(TestManagerState.IDLE);
  }
  
  @Override
  public void testPeriodic() {
    testManager.update();
    
  }
}
