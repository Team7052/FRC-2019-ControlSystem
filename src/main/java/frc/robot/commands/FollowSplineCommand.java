package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.auto.Spline;
import frc.auto.SplineFollower;
import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.Point;

public class FollowSplineCommand extends Command {
    double time = 0;
    // declare subsystem variable
    DriveTrain driveTrain;
    Spline spline;
    SplineFollower splineFollower;

    public ArrayList<Point> splinePath;
    
    public FollowSplineCommand(ArrayList<Point> splinePath, double totalTime) {
        super("Follow Spline Command");
        this.splinePath = splinePath;
        // get drive train subsystem instance
        driveTrain = DriveTrain.getInstance();
        
        //required for each command to know which subsystems it will be using
        requires(driveTrain);
        //Add (x, y) displacement points
        spline = new Spline(splinePath);
        splineFollower = new SplineFollower(spline, totalTime);
    }

    @Override
    protected void initialize() {
        super.initialize();
    }

    @Override
    protected void execute() {
        super.execute();

        Encoder leftEncoder = driveTrain.getLeftEncoder();
        Encoder rightEncoder = driveTrain.getRightEncoder();

        double kp = 0; //needs to be tuned
        double leftDifference = 0;
        double rightDifference = 0;
        splineFollower.startFollowingSpline();
        Pair<Double> velocities = splineFollower.updateSplinePosition();
        if (velocities == null) velocities = new Pair<Double>(0.0, 0.0);

        double leftSpeed = (velocities.a) + (leftDifference * kp);
        double rightSpeed = (velocities.b) + (rightDifference * kp);

        System.out.println( Math.round((leftSpeed) * 100.0) / 100.0 + " " + Math.round((rightSpeed) * 100.0) / 100.0);
  	    driveTrain.setLeftGroupSpeed(leftSpeed);
        driveTrain.setRightGroupSpeed(rightSpeed);
    }

    @Override
    protected boolean isFinished() {
        return splineFollower.isFinished();
    }
}