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
import frc.joysticks.*;
import frc.robot.commands.FollowSplineCommand;
import frc.robot.motionProfiling.Point;
import frc.robot.util.loops.Loop;
import frc.robot.util.loops.Looper;


public class Robot extends TimedRobot {

  public static OI oi;

  Looper mlooper;
  Looper autoLooper;
  LoopsManager loopsManager;
  FollowSplineCommand autoCommand;
  @Override
  public void robotInit() {
    // change Logitech to newly extended class
    mlooper = new Looper();
    oi = new Logitech(0);
    loopsManager = new LoopsManager();
    autoLooper = new Looper();
    
    mlooper.register(loopsManager.hardwareLoop);
    mlooper.register(loopsManager.networkLoop);
    mlooper.register(loopsManager.physicsWorldLoop);
    mlooper.register(loopsManager.stateManagerLoop);

    Point[] path = {
      new Point (0, 0), new Point (27,56.5), new Point (45, 81.5), new Point (60, 89.5)
    };
    autoCommand = new FollowSplineCommand(new ArrayList<>(Arrays.asList(path)), 5.0);

    autoLooper.register(new Loop(){
      @Override
      public void onStart() {
        
      }
      @Override
      public void onUpdate() {
        System.out.println("Update");
        autoCommand.execute();
      }
    });
    mlooper.start();
  }
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
    this.mlooper.stop();
    autoLooper.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    autoLooper.start();
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
