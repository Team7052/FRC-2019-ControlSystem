package frc.auto;

import java.util.ArrayList;

import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.Point;
import frc.robot.sequencing.Step;

public class SplineFollower extends Step<Pair<Double>> {
    private Spline spline;

    double baseSpeed = 0.36;
    double turnConst1 = 2;
    double turnConst2 = 110;
    double wheelSpinConst = 3;

    public SplineFollower(Spline spline, double desiredTime) {
        super(() -> desiredTime);
        this.spline = spline;
    }

    double prevTime = 0;

    @Override
    public Pair<Double> getUpdateForDeltaTime(double dt) {
        return this.computeForPercentage(dt / this.totalRunningTime.get());
    }

    private Pair<Double> computeForPercentage(double percentage) {
        if (percentage >= 1.00) return null;
        int desiredPoint = (int) (percentage * spline.getCubicSpline().size());
        //closest entered point to desired point

        int i = this.closest(spline.get_xs(), spline.getCubicSpline().get(desiredPoint).getX());
        boolean concaveUp = this.concaveUp(spline.get_xs(), i, spline.get_ys(), spline.getTangents());
        double leftTheo = this.getLeftSum(desiredPoint, spline.getCubicSpline());
        double rightTheo = this.getRightSum(desiredPoint, spline.getCubicSpline());
        Pair<Double> velocities = this.getSlope(desiredPoint, spline.getCubicSpline(), concaveUp);
        return velocities;
    }

    private boolean concaveUp(double[] xs, int i, double[] ys, ArrayList<Point> tangents) {
        double yLow;
        double yHigh;
        double mLow;
        double mHigh;
        double h = SplineMethods.calch(xs, i);
        if (i == 0) {
            yLow = ys[i];
            mLow = tangents.get(i).getY();
        } else {
            yLow = ys[i];
            mLow = tangents.get(i).getY();
        }

        if (i == ys.length - 1) {
            yHigh = ys[i];
            mHigh = tangents.get(i).getY();
        } else if (i == ys.length - 2) {
            yHigh = ys[i + 1];
            mHigh = tangents.get(i + 1).getY();
        } else {
            yHigh = ys[i + 1];
            mHigh = tangents.get(i + 1).getY();
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
            if((points.get(i+1).getX() - points.get(i).getX())>0){
                double theta = Math.atan(10*(points.get(i+1).getY() - points.get(i).getY()));
                leftSum+=Math.sin(theta)*(points.get(i+1).getY() - points.get(i).getY());   
            }
            else{
                leftSum+=points.get(i+1).getY() - points.get(i).getY();
            }
        }
        return leftSum*4096;
    }
    private double getRightSum(double desiredPoint, ArrayList<Point> points) {
        double rightSum = 0;
        for(int i=0; i<desiredPoint; i++){
            if((points.get(i+1).getX() - points.get(i).getX())<0){
                double theta = Math.atan(10*(points.get(i+1).getY() - points.get(i).getY()));
                rightSum+=Math.sin(theta)*points.get(i).getY();
            }
            else{
                rightSum+=points.get(i+1).getY()-points.get(i).getY();
            }
            
        }
        return rightSum*4096;

    }

    private Pair<Double> getSlope(int desiredPoint, ArrayList<Point> points, boolean concaveUp) {
       double leftSlope = baseSpeed, rightSlope = baseSpeed;
        double theta;
        if (desiredPoint != points.size() - 1 && desiredPoint != points.size() - 2) {
            double xOne = points.get(desiredPoint + 1).getX() - points.get(desiredPoint).getX();
            double yOne = points.get(desiredPoint + 1).getY() - points.get(desiredPoint).getY();
            double length1 = Math.sqrt(Math.pow(xOne, 2) + Math.pow(yOne, 2));

            // System.out.println("Length one: " + length1);
            double xTwo = points.get(desiredPoint + 2).getX() - points.get(desiredPoint + 1).getX();
            double yTwo = points.get(desiredPoint + 2).getY() - points.get(desiredPoint + 1).getY();
            double length2 = Math.sqrt(Math.pow(xTwo, 2) + Math.pow(yTwo, 2));

            // System.out.println("Length two: " + length2);
            double xThree = points.get(desiredPoint + 2).getX() - points.get(desiredPoint).getX();
            double yThree = points.get(desiredPoint + 2).getY() - points.get(desiredPoint).getY();
            double length3 = Math.sqrt(Math.pow(xThree, 2) + Math.pow(yThree, 2));

            //  System.out.println("Length three: " + length3);
            double part3 = (Math.pow(length1, 2) + Math.pow(length2, 2) - Math.pow(length3, 2));
            double alpha = Math.acos(part3 / (2 * length1 * length2));

            //  System.out.println("Alpha: " + alpha);
            double turn = Math.pow(turnConst1, turnConst2 * (Math.PI - alpha));
            
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