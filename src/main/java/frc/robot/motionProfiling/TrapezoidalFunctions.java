package frc.robot.motionProfiling;

import java.util.ArrayList;

import frc.robot.helpers.Pair;

public class TrapezoidalFunctions {
    // generates a trapezoidal shape based on max velocity and max acceleration
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

    public static TrapezoidShape generateTrapezoidShape(double startPosition, double endPosition, double maxVelocity, double maxAcceleration) {
        // max acceleration = (pi) rad / s^2
        // max velocity = (pi / 2) rad / s
        double displacement = endPosition - startPosition;
        if (displacement < 0) {
            maxVelocity = -maxVelocity;
            maxAcceleration = -maxAcceleration;
        }
        Point p1 = new Point(0, 0);
        Point p2, p3, p4;
        double accelTime = maxVelocity / maxAcceleration;
        double totalTime = displacement / maxVelocity + accelTime;

        if (accelTime >= totalTime / 2) {
            // constrained max velocity
            double halfTime = Math.sqrt(displacement / maxAcceleration);
            double newVelocity = halfTime * maxAcceleration;
            p2 = new Point(halfTime, newVelocity);
            p3 = new Point(halfTime, newVelocity);
            p4 = new Point(halfTime * 2, 0);
        }
        else {
            p2 = new Point(accelTime, maxVelocity);
            p3 = new Point(totalTime - accelTime, maxVelocity);
            p4 = new Point(totalTime, 0);
        }
        return new TrapezoidShape(p1, p2, p3, p4);
    }

    public static Pair<TrapezoidShape> syncTrapezoidShapes(TrapezoidShape shape1, TrapezoidShape shape2) {
        TrapezoidShape newShape1 = shape1;
        TrapezoidShape newShape2 = shape2;
        if (shape1.totalTime() > shape2.totalTime()) {
            newShape2 = transformShapeToTime(shape2, shape1.totalTime());
        }
        else if (shape2.totalTime() > shape1.totalTime()) {
            newShape1 = transformShapeToTime(shape1, shape2.totalTime());
        }
        return new Pair<TrapezoidShape>(newShape1, newShape2);
    }

    public static Pair<ArrayList<Point>> matchedTotalTimeForShapes(ArrayList<Point> shape1, ArrayList<Point> shape2) {
        double time1 = totalTimeOfShape(shape1);
        double time2 = totalTimeOfShape(shape2);
        ArrayList<Point> newShape1 = shape1;
        ArrayList<Point> newShape2 = shape2;
        if (time1 > time2) {
            newShape2 = transformTrapezoidByTime(shape2, time1);
        }
        else if (time2 > time1) {
            newShape1 = transformTrapezoidByTime(shape1, time2);
        }
        return new Pair<ArrayList<Point>>(newShape1, newShape2);
    }

    // transform a velocity trapezoidal shape to keep the same total displacement with a new total time
    public static ArrayList<Point> transformTrapezoidByTime(ArrayList<Point> trapezoidShape, double newTotalTime) { 
        ArrayList<Point> newTrapezoidShape = new ArrayList<>();

        double ratio = newTotalTime / totalTimeOfShape(trapezoidShape);
        for (Point p: trapezoidShape) {
            newTrapezoidShape.add(new Point(p.getX() * ratio, p.getY() / ratio));
        }
        
        return newTrapezoidShape;
    }

    public static TrapezoidShape transformShapeToTime(TrapezoidShape trapezoidShape, double newTotalTime) { 
        double ratio = newTotalTime / trapezoidShape.totalTime();
        Point[] points = trapezoidShape.getPoints();
        Point[] newPoints = new Point[points.length];
        for (int i =0 ; i < points.length; i++) {
            newPoints[i] = new Point(points[i].getX() * ratio, points[i].getY() / ratio);
        }
        if (newPoints.length != 4) return null;
        return new TrapezoidShape(newPoints[0], newPoints[1], newPoints[2], newPoints[3]);
    }

    public static double totalTimeOfShape(ArrayList<Point> shape) {
        if (shape.size() == 0) return 0;
        return shape.get(shape.size() - 1).getX();
    }
}