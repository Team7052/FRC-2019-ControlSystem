package frc.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.Point;

public class SplineFollower {
    private MotionProfileState state;
    private SplineFollowerState followingState;
    private double desiredTime;
    private double startTime;
    private double splineStartTime;
    private Spline spline;
    private double targetEncoderPosition;

    private final double radiusBase = 13.5;
    private final double radiusWheel = 3;
    double baseSpeed = 0.3;
    double turnConst1 = 2;
    double turnCost2 = 15;

    public SplineFollower(Spline spline, double desiredTime) {
        this.desiredTime = desiredTime;
        this.spline = spline;
        this.state = MotionProfileState.IDLE;
        this.followingState = SplineFollowerState.FOLLOWING_SPLINE;
        
       /* Point p0 = spline.getCubicSpline().get(0);
        Point p1 = spline.getCubicSpline().get(1);
        double targetTheta = Math.atan((p0.x - p1.x) / (p0.y - p1.y));
        targetEncoderPosition = targetTheta * this.radiusBase / (2 * Math.PI * radiusWheel);
        this.followingState = SplineFollowerState.FOLLOWING_SPLINE;*/
    }

    public void startFollowingSpline() {
        if (state == MotionProfileState.IDLE) {
            state = MotionProfileState.RUNNING;
            splineStartTime = Timer.getFPGATimestamp();
        }
    }

    public Pair<Double> updateSplinePosition() {
        if (state == MotionProfileState.RUNNING) {
            if (followingState == SplineFollowerState.FOLLOWING_SPLINE) {
                double currentTime = Timer.getFPGATimestamp();
                double percentage = (currentTime - splineStartTime) / desiredTime;
                if (percentage >= 1.00) {
                    this.stopFollowingSpline();
                    return null;
                }
                int desiredPoint = (int) (percentage * spline.getCubicSpline().size());
                //closest entered point to desirecpoint
                int i = this.closest(spline.get_xs(), spline.getCubicSpline().get(desiredPoint).x);
                //System.out.println(i);
                boolean concaveUp = this.concaveUp(spline.get_xs(), i, spline.get_ys(), spline.getTangents());
                // System.out.println("Concave Up at " + newPoints.get(desiredPoint).x + ": " + concaveUp);
                double leftTheo = this.getLeftSum(desiredPoint, spline.getCubicSpline());
                double rightTheo = this.getRightSum(desiredPoint, spline.getCubicSpline());
                Pair<Double> velocities = this.getSlope(desiredPoint, spline.getCubicSpline(), concaveUp);
                return velocities;
            }
            else if (this.followingState == SplineFollowerState.TURNING) {
                // turning code
                
                // if (robot finishd turn) this.updateFollowingState(SplineFollowerState.FOLLOW_SPLINE);
                // else turn;
            }
        }
        return null;
    }

    public void stopFollowingSpline() {
        this.state = MotionProfileState.FINISHED;
    }

    public boolean isFinished() {
        return this.state == MotionProfileState.FINISHED;
    }

    private void updateSplineState(SplineFollowerState newState) {
        if (newState == SplineFollowerState.FOLLOWING_SPLINE && this.followingState != SplineFollowerState.FOLLOWING_SPLINE) {
            this.followingState = newState;
            this.splineStartTime = Timer.getFPGATimestamp();
        }
        else if (newState == SplineFollowerState.TURNING) {
            this.followingState = newState;
        }
    }

    private boolean concaveUp(double[] xs, int i, double[] ys, ArrayList<Point> tangents) {
        double yLow;
        double yHigh;
        double mLow;
        double mHigh;
        double h = SplineMethods.calch(xs, i);
        if (i == 0) {
            yLow = ys[i];
            mLow = tangents.get(i).y;
        } else {
            yLow = ys[i];
            mLow = tangents.get(i).y;
        }

        if (i == ys.length - 1) {
            yHigh = ys[i];
            mHigh = tangents.get(i).y;
        } else if (i == ys.length - 2) {
            yHigh = ys[i + 1];
            mHigh = tangents.get(i + 1).y;
        } else {
            yHigh = ys[i + 1];
            mHigh = tangents.get(i + 1).y;
        }
        double const1 = yLow;
        double const2 = h * mLow;
        double const3 = yHigh;
        double const4 = h * mHigh;
        double v = xs[i];
        double x = SplineMethods.calct(xs, i, v);

        double part1 = 12 * x - 6;
        double part2 = 6 * x - 4;
        double part3 = -12 * x + 6;
        double part4 = 6 * x - 2;

        double num = const1 * part1 + const2 * part2 + const3 * part3 + const4 * part4;

        if (num >= 0) {
            return true;
        } else {
            return false;
        }
    }

    private int closest(double[] numbers, double myNumber) {
        double distance = Math.abs(numbers[0] - myNumber);
        int idx = 0;
        for (int c = 1; c < numbers.length; c++) {
            double cdistance = Math.abs(numbers[c] - myNumber);
            if (cdistance < distance) {
                idx = c;
                distance = cdistance;
            }
        }
        return idx;
    }

    private double getLeftSum(double desiredPoint, ArrayList<Point> points) {
        double leftSum=0;
        for(int i=0; i<desiredPoint; i++){
            if((points.get(i+1).x - points.get(i).x)>0){
                double theta = Math.atan(10*(points.get(i+1).y - points.get(i).y));
                leftSum+=Math.sin(theta)*(points.get(i+1).y - points.get(i).y);   
            }
            else{
                leftSum+=points.get(i+1).y - points.get(i).y;
            }
        }
        return leftSum*4096;
    }
    private double getRightSum(double desiredPoint, ArrayList<Point> points) {
        double rightSum = 0;
        for(int i=0; i<desiredPoint; i++){
            if((points.get(i+1).x - points.get(i).x)<0){
                double theta = Math.atan(10*(points.get(i+1).y - points.get(i).y));
                rightSum+=Math.sin(theta)*points.get(i).y;
            }
            else{
                rightSum+=points.get(i+1).y-points.get(i).y;
            }
            
        }
        return rightSum*4096;

    }

    private double getSlope(int desiredPoint, ArrayList<Point> points, boolean concaveUp) {
       double leftSlope = 0.3, rightSlope = 0.3;
        double theta;
        if (desiredPoint != points.size() - 1 && desiredPoint != points.size() - 2) {
            double xOne = points.get(desiredPoint + 1).x - points.get(desiredPoint).x;
            double yOne = points.get(desiredPoint + 1).y - points.get(desiredPoint).y;
            double length1 = Math.sqrt(Math.pow(xOne, 2) + Math.pow(yOne, 2));

            // System.out.println("Length one: " + length1);
            double xTwo = points.get(desiredPoint + 2).x - points.get(desiredPoint + 1).x;
            double yTwo = points.get(desiredPoint + 2).y - points.get(desiredPoint + 1).y;
            double length2 = Math.sqrt(Math.pow(xTwo, 2) + Math.pow(yTwo, 2));

            // System.out.println("Length two: " + length2);
            double xThree = points.get(desiredPoint + 2).x - points.get(desiredPoint).x;
            double yThree = points.get(desiredPoint + 2).y - points.get(desiredPoint).y;
            double length3 = Math.sqrt(Math.pow(xThree, 2) + Math.pow(yThree, 2));

            //  System.out.println("Length three: " + length3);
            double part3 = (Math.pow(length1, 2) + Math.pow(length2, 2) - Math.pow(length3, 2));
            double alpha = Math.acos(part3 / (2 * length1 * length2));

            //  System.out.println("Alpha: " + alpha);
            double turn = Math.pow(turnConst1, turnCost2 * (Math.PI - alpha));

            //System.out.println("ka: " + ka);
            if (concaveUp) {
                rightSlope = baseSpeed * turn;
                //    System.out.println("leftSLope: " + leftSlope);
            } else {
                leftSlope = baseSpeed * turn;
            }

        }

        return new Pair<>(leftSlope, rightSlope);
    }
}