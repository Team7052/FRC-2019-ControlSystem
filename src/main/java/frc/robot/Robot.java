/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.joysticks.*;
import frc.robot.util.loops.Looper;


public class Robot extends TimedRobot {

  public static OI oi;

  Looper mlooper;
  LoopsManager loopsManager;
  @Override
  public void robotInit() {
    // change Logitech to newly extended class
    mlooper = new Looper();
    oi = new Logitech(0);
    loopsManager = new LoopsManager();
    
    mlooper.register(loopsManager.hardwareLoop);
    mlooper.register(loopsManager.networkLoop);
    mlooper.register(loopsManager.physicsWorldLoop);
    mlooper.register(loopsManager.stateManagerLoop);

    mlooper.start();

    /*Point[] path = {
      new Point (0, 0), new Point (0.5,4), new Point (3, 5), new Point (6, 6)
    };*/
    //autoCommand = new FollowSplineCommand(new ArrayList<>(Arrays.asList(path)), 5.0);
  }
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
    this.mlooper.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    this.mlooper.start();
  }

  @Override
  public void testInit() {
  }
  
  @Override
  public void testPeriodic() {
  }
}
