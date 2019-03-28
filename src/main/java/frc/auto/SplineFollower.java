package frc.auto;

import java.util.ArrayList;

import frc.robot.helpers.Pair;
import frc.robot.helpers.Quadruple;
import frc.robot.motionProfiling.Point;
import frc.robot.sequencing.Step;
import frc.robot.subsystems.DriveTrain;

public class SplineFollower extends Step<Pair<Double>> {
    DriveTrain driveTrain;
    private Spline spline;

    double baseSpeed = 0.36;
    double turnConst1 = 2;
    double turnConst2 = 110;
    double wheelSpinConst = 3;
    double rw = 3;
    double base = 23;
    double leftTheo = 0;
    double rightTheo = 0;
    double kv = 0.7;
    double kp = 0.3;
    int previous;

    public SplineFollower(Spline spline, double desiredTime) {
        super(() -> desiredTime);
        this.spline = spline;
        driveTrain = DriveTrain.getInstance();
    }

    double prevTime = 0;

    @Override
    public Pair<Double> update(double timestamp) {
        double percentage = (timestamp - this.startTime) / this.totalRunningTime.get();
        if (percentage >= 1.00) {
            this.endStep();
            return null;
        }
        return this.computeForPercentage(percentage);
    }

    @Override
    public Pair<Double> getLastUpdate() {
        return this.computeForPercentage(0.99999);
    }

    private Pair<Double> computeForPercentage(double percentage) {
        if (percentage >= 1.00) return null;
        int desiredPoint = (int) (percentage * spline.getCubicSpline().size());
        int i = this.closest(spline.getCubicSpline(), spline.getCubicSpline().get(desiredPoint).x, true);
        //closest entered point to desired point

        if(desiredPoint !=  previous){
            Pair<Double> theos = this.sums(spline.getCubicSpline(), spline.getTangents(), spline.getCubicSpline().get(desiredPoint), i);
            leftTheo += theos.a;
            rightTheo += theos.b;
        }
        System.out.println("Left: "+ leftTheo + " Right: " + rightTheo);

        boolean concaveUp = this.concaveUp(spline.getCubicSpline(), spline.getTangents(), i);
        Pair<Double> velocities = this.getSlope(desiredPoint, spline.getCubicSpline(), concaveUp);
        double leftDifference = leftTheo - driveTrain.getLeftDisplacement();
        double rightDifference = rightTheo - driveTrain.getLeftDisplacement();
        Pair<Double> finalVelocities = new Pair<>((kv*velocities.a) + (kp*leftDifference), (kv*velocities.b) + (kp*rightDifference));
        previous = desiredPoint;
        return finalVelocities;
    }

    private boolean concaveUp(ArrayList<Point> splinePoints, ArrayList<Point> tangents,  int index) {
        Quadruple<Double> constants = spline.getConstants(index);
        double v = splinePoints.get(index).x;
        double x = spline.calct(splinePoints, index, v);

        double part1 = 12 * x - 6;
        double part2 = 6 * x - 4;
        double part3 = -12 * x + 6;
        double part4 = 6 * x - 2;

        double num = constants.a * part1 + constants.b * part2 + constants.c * part3 + constants.d * part4;

        if (num >= 0) {
            return true;
        } else {
            return false;
        }
    }

    private int closest(ArrayList<Point> splinePoints, double myNumber, boolean x) {
        double distance = Math.abs(x ? splinePoints.get(0).x : splinePoints.get(0).y - myNumber);
        int idx = 0;
        for (int c = 1; c < splinePoints.size(); c++) {
            double cdistance = Math.abs(x ? splinePoints.get(c).x : splinePoints.get(c).y - myNumber);
            if (cdistance < distance) {
                idx = c;
                distance = cdistance;
            }
        }
        return idx;
    }

    private Pair<Double> sums(ArrayList<Point> splinePoints, ArrayList<Point> tangents, Point desiredPoint, int i) {
        Quadruple<Double> constants = spline.getConstants(i);
        double v = desiredPoint.y;

        double x = spline.calct(splinePoints, i, v);

        //Calc first derivative
        double par1 = 6*Math.pow(x,2) - 6*x;
        double par2 = 3*Math.pow(x,2) - 4*x +1;
        double par3 = -6*Math.pow(x,2) +6*x;
        double par4 = 3*Math.pow(x,2) - 2*x;

        double firDyDx = constants.a * par1 + constants.b * par2 + constants.c * par3 + constants.d * par4;        
        
        //Calc second derivative
        double part1 = 12 * x - 6;
        double part2 = 6 * x - 4;
        double part3 = -12 * x + 6;
        double part4 = 6 * x - 2;

        double secDyDx = constants.a * part1 + constants.b * part2 + constants.c * part3 + constants.d * part4;

        double radiusCircle = Math.pow(1+Math.pow(firDyDx,2), 3/2)/Math.abs(secDyDx);
        int s = this.closest(splinePoints, desiredPoint.x, true);
        double rightDisplacement;
        double leftDisplacement;
        if(concaveUp(splinePoints, tangents, s)){
            double theta = 1-0.005/Math.pow(radiusCircle,2);

             rightDisplacement = (theta*(radiusCircle+base))/rw;
             leftDisplacement = (theta*radiusCircle)/rw;

        } else {
            double theta = 1-0.005/Math.pow(radiusCircle,2);

             leftDisplacement = (theta*(radiusCircle+base))/rw;
             rightDisplacement = (theta*radiusCircle)/rw;
            }

        return new Pair<>(leftDisplacement,rightDisplacement);
    }

    private Pair<Double> getSlope(int desiredPoint, ArrayList<Point> points, boolean concaveUp) {
       double leftSlope = baseSpeed, rightSlope = baseSpeed;
        if (desiredPoint != points.size() - 1 && desiredPoint != points.size() - 2) {
            double xOne = points.get(desiredPoint + 1).x - points.get(desiredPoint).x;
            double yOne = points.get(desiredPoint + 1).y - points.get(desiredPoint).y;
            double length1 = Math.sqrt(Math.pow(xOne, 2) + Math.pow(yOne, 2));

            double xTwo = points.get(desiredPoint + 2).x - points.get(desiredPoint + 1).x;
            double yTwo = points.get(desiredPoint + 2).y - points.get(desiredPoint + 1).y;
            double length2 = Math.sqrt(Math.pow(xTwo, 2) + Math.pow(yTwo, 2));

            double xThree = points.get(desiredPoint + 2).x - points.get(desiredPoint).x;
            double yThree = points.get(desiredPoint + 2).y - points.get(desiredPoint).y;
            double length3 = Math.sqrt(Math.pow(xThree, 2) + Math.pow(yThree, 2));

            double part3 = (Math.pow(length1, 2) + Math.pow(length2, 2) - Math.pow(length3, 2));
            double alpha = Math.acos(part3 / (2 * length1 * length2));

            double turn = Math.pow(turnConst1, turnConst2 * (Math.PI - alpha));
            
            if (concaveUp) {
                rightSlope = baseSpeed * turn;

            } else {
                leftSlope = baseSpeed * turn;
            }

        }

        return new Pair<>(leftSlope, rightSlope);
    }
}