package frc.robot.commands.arm;

import frc.robot.motionProfiling.MotionProfiler;

public interface CommandDelegate {
    void beganMotionProfile(String commandName, MotionProfiler profiler);
}