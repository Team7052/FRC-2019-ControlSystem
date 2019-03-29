package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;
import frc.auto.Spline;
import frc.auto.SplineFollower;
import frc.auto.TurnToAngleStep;
import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.Point;
import frc.robot.sequencing.Sequence;

public class FollowSplineCommand {
    double time = 0;
    // declare subsystem variable
    DriveTrain driveTrain;

    Sequence<Pair<Double>> driveTrainSequence;

    public ArrayList<Point> splinePath;
    
    public FollowSplineCommand(ArrayList<Point> splinePath, double totalTime) {
        this.splinePath = splinePath;
        // get drive train subsystem instance
        driveTrain = DriveTrain.getInstance();
        
        //required for each command to know which subsystems it will be using
        //Add (x, y) displacement points
        Spline spline = new Spline(splinePath);

        // turn to angle step
        /*if (driveTrain.imuSensorIsConnectedAndCalibrated()) {
            driveTrain.resetAngle();
            double initAngle = driveTrain.getAngle();
            TrapezoidShape turnVelocityShape = TrapezoidalFunctions.generateTrapezoidShape(initAngle, spline.getInitialTurnAngle(), Math.PI / 2, Math.PI / 2);
            FilterStep<MotionTriplet> turningFilter = FilterStep.trapezoidalProfileFilter(turnVelocityShape, initAngle);
            
        }
        else {*/
            driveTrainSequence = new Sequence<>();
            TurnToAngleStep angleTurner = new TurnToAngleStep(spline.getInitialTurnAngle(), false);
            driveTrainSequence.addStep(angleTurner);
        //}


        SplineFollower splineFollower = new SplineFollower(spline, totalTime);

        driveTrainSequence.addStep(splineFollower);
    }

    public void execute() {
        double kp = 0; //needs to be tuned
        double leftDifference = 0;
        double rightDifference = 0;
        driveTrainSequence.start(Timer.getFPGATimestamp());
        Pair<Double> velocities = driveTrainSequence.update(Timer.getFPGATimestamp());
        if (velocities == null) velocities = new Pair<Double>(0.0, 0.0);

        double leftSpeed = (velocities.a) + (leftDifference * kp);
        double rightSpeed = (velocities.b) + (rightDifference * kp);

  	    driveTrain.setLeftGroupSpeed(leftSpeed);
        driveTrain.setRightGroupSpeed(rightSpeed);
    }
}