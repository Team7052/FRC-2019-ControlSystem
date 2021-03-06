/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.joysticks.*;
import frc.robot.commands.FollowSplineCommand;

import frc.robot.util.loops.Looper;


public class Robot extends TimedRobot {

    public static OI oi;

    Looper globalLooper;
    Looper teleopLooper;
    Looper autoLooper;
    LoopsManager loopsManager;
    FollowSplineCommand autoCommand;
    @Override
    public void robotInit() {
        // change Logitech to newly extended class
        teleopLooper = new Looper();
        autoLooper = new Looper();
        globalLooper = new Looper();
        oi = new Logitech(0);

        loopsManager = new LoopsManager();

        teleopLooper.register(loopsManager.hardwareLoop);
        teleopLooper.register(loopsManager.stateManagerLoop);

        globalLooper.register(loopsManager.networkLoop);
        globalLooper.register(loopsManager.physicsWorldLoop);

        autoLooper.register(loopsManager.autoLoop);

        globalLooper.start();
    }
    @Override
    public void robotPeriodic() {
    }

    @Override
    public void disabledInit() {
        this.teleopLooper.stop();
        this.autoLooper.stop();
    }

    @Override
    public void disabledPeriodic() {
        this.teleopLooper.stop();
        this.autoLooper.stop();
    }

    @Override
    public void autonomousInit() {
        autoLooper.start();
    }

    @Override
    public void autonomousPeriodic() {
        autoLooper.start();
    }

    @Override
    public void teleopInit() {
        teleopLooper.start();
    }

    @Override
    public void teleopPeriodic() {
        teleopLooper.start();
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }
}
