package frc.robot.motionProfiling;

import java.util.ArrayList;

public class FunctionGenerator {
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
