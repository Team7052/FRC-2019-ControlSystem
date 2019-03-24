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
            newTrapezoidShape.add(new Point(p.x * ratio, p.y / ratio));
        }
        
        return newTrapezoidShape;
    }

    public static double totalTimeOfShape(ArrayList<Point> shape) {
        if (shape.size() == 0) return 0;
        return shape.get(shape.size() - 1).x;
    }
}