/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.joysticks.*;
import frc.robot.commands.RotateShoulderJoint;
import frc.robot.commands.TankDriveCommand;
import frc.robot.networking.Network;
import frc.robot.networking.TableType;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;

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
  RotateShoulderJoint armCommand = new RotateShoulderJoint();
  TankDriveCommand driveCommand = new TankDriveCommand();
  //DriveTenM driveTenCommand = new DriveTenM();
  //TankDriveCommand tankDriveCommand = new TankDriveCommand();
  CommandGroup newGroup;
  @Override
  public void robotInit() {
      //change Logitech to newly extended class
    oi = new Logitech(0);
    newGroup = new CommandGroup();
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
    ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    double wristOutput = armSubsystem.wristMotor.getMotorOutputPercent();
    double elbowOutput = armSubsystem.getMotorOutputPercent(Motor.ELBOW_JOINT);
    double shoulderOutput = armSubsystem.getMotorOutputPercent(Motor.SHOULDER_JOINT);

    Network network = Network.getInstance();
    
    NetworkTableEntry wristEntry = network.getTableEntry(TableType.MOTOR_DATA, "armWristMotor");
    NetworkTableEntry elbowEntry = network.getTableEntry(TableType.MOTOR_DATA, "armElbowMotor");
    NetworkTableEntry shoulderEntry = network.getTableEntry(TableType.MOTOR_DATA, "armShoulderMotor");
    
    wristEntry.setDouble(wristOutput);
    elbowEntry.setDouble(elbowOutput);
    shoulderEntry.setDouble(shoulderOutput);
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

    newGroup.addParallel(armCommand);
    newGroup.addParallel(driveCommand);
    
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
  public void testPeriodic() {
  }
}
