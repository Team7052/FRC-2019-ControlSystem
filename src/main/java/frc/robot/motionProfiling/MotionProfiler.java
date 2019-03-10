package frc.robot.motionProfiling;

import java.util.ArrayList;

public class MotionProfiler {
    private ArrayList<Point> velocityFunction = new ArrayList<>();
    private ArrayList<Point> accelerationFunction = new ArrayList<>();
    private ArrayList<Point> positionFunction = new ArrayList<>();

    private MotionProfileState state = MotionProfileState.IDLE;

    private double totalRunningTime;

    public MotionProfiler() {
        
    }
    public double startTime = 0;
    public double index = 0;
    public MotionTriplet updateMotionProfile() {
        if (state == MotionProfileState.RUNNING) {
            double currentTime = (double) System.currentTimeMillis() / 1000;
            
            double percentage = (currentTime - startTime) / this.totalRunningTime;
            if (percentage > 1.0) {
                this.stopMotionProfile();
                return null;
            }
            
            int index = (int) Math.round(percentage * ((double) velocityFunction.size() - 1));
            
            // get velocity
            double velocityMeasurement = velocityFunction.get(index).y;
            double accelerationMeasurement = accelerationFunction.get(index).y;
            double positionMeasurement = positionFunction.get(index).y;

            return new MotionTriplet(velocityMeasurement, positionMeasurement, accelerationMeasurement);
        }
        else {
            return null;
        }
    }

    // setters and getters for motion profile

    public MotionProfileState getState() {
        return this.state;
    }

    private void setTotalRunningTime() {
        if (this.positionFunction.size() > 0) {
            this.totalRunningTime = this.positionFunction.get(this.positionFunction.size() - 1).x;
        }
    }

    public void stopMotionProfile() {
        this.state = MotionProfileState.FINISHED;
    }
    public void reset() {
        this.state = MotionProfileState.IDLE;
    }

    public void startMotionProfile() {
        if (this.state == MotionProfileState.IDLE) {
            this.state = MotionProfileState.RUNNING;
            this.startTime = (double) System.currentTimeMillis() / 1000;
        }
    }

    public static ArrayList<Point> getLinearInterpolation(ArrayList<Point> points, double delta) {
        ArrayList<Point> interpolatedPoints = new ArrayList<>();
        if (points.size() < 2) {
            return points;
        }
        for (int i = 0, n = points.size() - 1; i < n; i++) {
            double deltaX = points.get(i+1).x - points.get(i).x;
            interpolatedPoints.add(points.get(i));
            double slope = (points.get(i+1).y - points.get(i).y) / (points.get(i+1).x - points.get(i).x);
            double b = points.get(i).y - slope * points.get(i).x;

            for (double j = delta; j < deltaX; j += delta) {
                double x = j + points.get(i).x;
                interpolatedPoints.add(new Point(x, slope * x + b));
            }
        }
        interpolatedPoints.add(points.get(points.size() - 1));
        return interpolatedPoints;
    }

    public double getLeftSum(double desiredPoint, ArrayList<Point> points) {
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
    public double getRightSum(double desiredPoint, ArrayList<Point> points) {
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
    public double getLeftSlope(int desiredPoint, ArrayList<Point> points, boolean concaveUp) {
       double leftSlope = 0.5, rightSlope = 0.5;
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
            double ka = Math.pow(2, 25 * (Math.PI - alpha));
            double kp = 0.5;

            //System.out.println("ka: " + ka);
            if (concaveUp) {
                rightSlope = kp * ka;
                //    System.out.println("leftSLope: " + leftSlope);
            } else {
                leftSlope = kp * ka;
            }

        }

        return leftSlope;


    }
    public static ArrayList<Point> calcSecants(ArrayList<Point> points) {
        ArrayList<Point> secants = new ArrayList<>();
        double secant = 0;
        for (int i = 0; i < points.size() - 1; i++) {
            secant = (points.get(i + 1).y - points.get(i).y) / (points.get(i + 1).x - points.get(i).x);

            secants.add(new Point(i, secant));

        }

        return secants;
    }

    public static double calch(double xs[], int i) {
        double h;
        if (i == 0) {
            h = xs[i + 1] - xs[i];
        } else if (i == xs.length - 1) {
            h = xs[i] - xs[i];
        } else {
            h = xs[i + 1] - xs[i];
        }

        return h;
    }

    public static double calct(double[] xs, int i, double x) {
        double h = calch(xs, i);
        double t;
        if (xs[i] == 0) {
            t = (x - xs[i]) / h;
        } else {
            t = (x - xs[i]) / h;
        }
        return t;
    }

    public static ArrayList<Point> calcFinalPoints(double[] xs, int i, double[] ys, ArrayList<Point> tangents) {
        ArrayList<Point> finalPoints = new ArrayList<>();
        double yLow;
        double yHigh;
        double mLow;
        double mHigh;
        double h = calch(xs, i);
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
        if (xs[i] != xs[xs.length - 1]) {
            if (xs[i] <= xs[i + 1]) {
                for (double x = xs[i]; x <= xs[i + 1]; x = x + 0.02) {
                    double t = calct(xs, i, x);
                    double part1 = 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
                    part1 = part1 * const1;

                    double part2 = Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
                    part2 = part2 * const2;

                    double part3 = -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
                    part3 = part3 * const3;

                    double part4 = Math.pow(t, 3) - Math.pow(t, 2);
                    part4 = part4 * const4;

                    double num = part1 + part2 + part3 + part4;

                    finalPoints.add(new Point(x, num));
                }
            } else {
                for (double x = xs[i]; x >= xs[i + 1]; x = x - 0.02) {
                    double t = calct(xs, i, x);
                    double part1 = 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
                    part1 = part1 * const1;

                    double part2 = Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
                    part2 = part2 * const2;

                    double part3 = -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
                    part3 = part3 * const3;

                    double part4 = Math.pow(t, 3) - Math.pow(t, 2);
                    part4 = part4 * const4;

                    double num = part1 + part2 + part3 + part4;

                    finalPoints.add(new Point(x, num));
                }

            }

        }
        return finalPoints;
    }

    public boolean concaveUp(double[] xs, int i, double[] ys, ArrayList<Point> tangents) {
        double yLow;
        double yHigh;
        double mLow;
        double mHigh;
        double h = calch(xs, i);
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
        double x = calct(xs, i, v);

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

    public static ArrayList<Point> calcTangents(ArrayList<Point> points) {
        ArrayList<Point> tangents = new ArrayList<>();
        ArrayList<Point> secants = calcSecants(points);
        double tangent = 0;
        double alpha = 0;
        double beta = 0;
        boolean run = true;
        boolean doubleInc = false;
        int i = 0;
        while (i < points.size()) {
            if (i == 0) {
                tangents.add(new Point(i, secants.get(i).y));
                run = false;
            } else if (i == points.size() - 1) {
                tangents.add(new Point(i, secants.get(i - 1).y));
                run = false;
            } else if (secants.get(i).y == 0) {
                tangents.add(new Point(i, 0));
                tangents.add(new Point(i + 1, 0));
                doubleInc = true;
                run = false;
            } else if (secants.get(i).y < 0 && secants.get(i).y > 0) {
                tangent = 0;
            } else if (secants.get(i).y > 0 && secants.get(i).y < 0) {
                tangent = 0;
            } else {

                tangent = (secants.get(i - 1).y + secants.get(i).y) / 2;
            }
            if (run) {
                if (secants.get(i).y != 0) {
                    alpha = tangent / secants.get(i).y;
                }
                if (secants.get(i - 1).y != 0) {
                    beta = tangent / secants.get(i - 1).y;
                }

                if (alpha < 0 || beta < 0) {
                    tangent = 0;
                }
                if (alpha > 3) {
                    tangent = 3 * beta;
                }
                if (beta > 3) {
                    tangent = 3 * alpha;
                }

                tangents.add(new Point(i, tangent));
            }
            if (doubleInc) {
                i++;
            }
            doubleInc = false;
            run = true;
            i++;
        }

        return tangents;
    }

    public int closest(double[] numbers, double myNumber) {
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

    public double getRightSlope(int desiredPoint, ArrayList<Point> points, boolean concaveUp) {
        double leftSlope = 0.5, rightSlope = 0.5;
        double theta;
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

            double ka = Math.pow(2, 25 * (Math.PI - alpha));
            double kp = 0.5;

            if (concaveUp) {
                rightSlope = kp * ka;
            } else {
                leftSlope = kp * ka;
            }
        }

        return rightSlope;
    }

    public void setVelocityPoints(ArrayList<Point> points) {
        this.setVelocityPoints(points, 0);
    }
    
    // setters and getters for functions
    public void setVelocityPoints(ArrayList<Point> points, double initialDisplacement) {
        velocityFunction = points;
        accelerationFunction = FunctionGenerator.getDerivative(points);
        positionFunction = this.pointsWithInitialDisplacement(FunctionGenerator.getIntegral(points), initialDisplacement);

        this.setTotalRunningTime();
    }
    public void setPositionPoints(ArrayList<Point> points) {
        this.setPositionPoints(points, 0);

    }
    public void setPositionPoints(ArrayList<Point> points, double initialDisplacement) {
        positionFunction = this.pointsWithInitialDisplacement(points, initialDisplacement);
        velocityFunction = FunctionGenerator.getDerivative(points);
        accelerationFunction = FunctionGenerator.getDerivative(velocityFunction);

        this.setTotalRunningTime();
    }

    public void setAccelerationFunctionPoints(ArrayList<Point> points) {
        this.setAccelerationFunctionPoints(points, 0);
    }
    public void setAccelerationFunctionPoints(ArrayList<Point> points, double initialDisplacement) {
        accelerationFunction = points;
        velocityFunction = FunctionGenerator.getIntegral(points);
        positionFunction = this.pointsWithInitialDisplacement(FunctionGenerator.getIntegral(velocityFunction), initialDisplacement);

        this.setTotalRunningTime();
    }

    public double getFinalPosition() {
        return this.positionFunction.get(this.positionFunction.size() - 1).y;
    }

    public double getTotalTime() {
        return this.positionFunction.get(this.positionFunction.size() - 1).x;
    }

    private ArrayList<Point> pointsWithInitialDisplacement(ArrayList<Point> points, double initialDisplacement) {
        ArrayList<Point> converted = new ArrayList<>();
        for (Point point: points) {
            converted.add(new Point(point.x, point.y + initialDisplacement));
        }
        return converted;
    }

    public ArrayList<Point> getVelocityFunction() {
        return velocityFunction;
    }
    public ArrayList<Point> getAccelerationFunction() {
        return accelerationFunction;
    }
    public ArrayList<Point> getPositionFunction() {
        return positionFunction;
    }

    public static ArrayList<Point> generateTrapezoidalProfile(double startPosition, double endPosition, double maxVelocity, double maxAcceleration) {
        // max acceleration = (pi) rad / s^2
        // max velocity = (pi / 2) rad / s
        double displacement = endPosition - startPosition;
        if (displacement < 0) {
            maxVelocity = -maxVelocity;
            maxAcceleration = -maxAcceleration;
        }
        ArrayList<Point> trapezoidalPoints = new ArrayList<>();
        trapezoidalPoints.add(new Point(0, 0));
        double accelTime = maxVelocity / maxAcceleration;
        double totalTime = displacement / maxVelocity + accelTime;

        if (accelTime >= totalTime / 2) {
            // constrained max velocity
            double halfTime = Math.sqrt(displacement / maxAcceleration);
            double newVelocity = halfTime * maxAcceleration;
            trapezoidalPoints.add(new Point(halfTime, newVelocity));
            trapezoidalPoints.add(new Point(halfTime * 2, 0));
        }
        else {
            trapezoidalPoints.add(new Point(accelTime, maxVelocity));
            trapezoidalPoints.add(new Point(totalTime - accelTime, maxVelocity));
            trapezoidalPoints.add(new Point(totalTime, 0));
        }
        return trapezoidalPoints;
    }

    public static ArrayList<Point> transformTrapezoidByTime(ArrayList<Point> trapezoidShape, double newTotalTime) { 
        ArrayList<Point> newTrapezoidShape = new ArrayList<>();

        double ratio = newTotalTime / MotionProfiler.totalTimeOfProfile(trapezoidShape);
        for (Point p: trapezoidShape) {
            newTrapezoidShape.add(new Point(p.x * ratio, p.y / ratio));
        }
        
        return newTrapezoidShape;
    }

    public static double totalTimeOfProfile(ArrayList<Point> points) {
        return points.get(points.size() - 1).x;
    }
    public void printPositions() {
        for (Point point: this.getPositionFunction()) {
            System.out.print(point.y / Math.PI * 180.0 + " ");
        }
        System.out.println();
    }
}