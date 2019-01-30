package frc.robot.motionProfiling;

import java.util.ArrayList;

public class FunctionGenerator {
    public static ArrayList<Point> generate(FunctionSet... sets) {
        ArrayList<Point> finalFunction = new ArrayList<>();
        for (int i = 0, n = sets.length; i < n; i++) {
            if (i != 0) {
                if (sets[i - 1].getUpper() > sets[i].getLower()) {
                    return new ArrayList<>();
                }
            }
            ArrayList<Point> currentSetFunction = FunctionGenerator.generate((sets[i]));
            for (Point p: currentSetFunction) finalFunction.add(p);
        }
        if (sets.length > 1) {
            if (sets[sets.length - 1].getLower() < sets[sets.length - 2].getUpper()) return new ArrayList<>();
        }
        return finalFunction;
    }

    public static ArrayList<Point> generate(FunctionSet set) {
        ArrayList<Point> newPoints = new ArrayList<>();
        if (set.getLower() > set.getUpper()) return newPoints;
        for (double i = set.getLower(); i <= set.getUpper(); i += set.getDelta()) {
            double x = i;
            double y = set.getFunction().resultFor(x);
            newPoints.add(new Point(x, y));
        }
        return newPoints;
    }

    public static ArrayList<Point> getDerivative(ArrayList<Point> points) {
        ArrayList<Point> derivate = new ArrayList<>();
        for (int i = 0, n = points.size(); i < n; i++) {
            double leftSlope = 0;
            if (i > 0) {
                leftSlope = (points.get(i).y - points.get(i-1).y) / (points.get(i).x - points.get(i-1).x);
            }
            else {
                leftSlope = (points.get(i + 1).y - points.get(i).y) / (points.get(i + 1).x - points.get(i).x);
            }
            derivate.add(new Point(points.get(i).x, leftSlope));
        }
        return derivate;
    }
    public static ArrayList<Point> getIntegral(ArrayList<Point> points) {
        ArrayList<Point> integral = new ArrayList<>();
        double sum = 0;
        for (int i = 0, n = points.size(); i < n; i++) {
            if (i != 0) {
                sum += (points.get(i).x - points.get(i-1).x) * (points.get(i-1).y + points.get(i).y) / 2; // trapezoidal rule
            }

            integral.add(new Point(points.get(i).x, sum));
        }

        return integral;
    }
}
