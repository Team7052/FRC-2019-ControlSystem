package frc.auto;
import frc.robot.motionProfiling.Point;
import java.util.ArrayList;

public class SplineMethods {
    public static final double delta = 0.02;
    public static final double k_value = 5.0 * delta;
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

    public static double xDiff(double[] xs, double[] ys, ArrayList<Point> tangents, int i, double delta, Point goalPoint) {
        ArrayList<Point> points = new ArrayList<>();
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
        double[] distances = new double[(99999)];
        int g = 0;

        //System.out.println("Desired Point: (" + xs[i] + ", " + ys[i] + ")");
        for (double j = goalPoint.x + 0.001; j <= xs[i + 1]/2; j = j + 0.001) {

            double t = SplineMethods.calct(xs, i, j);
            double part1 = 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
            part1 = part1 * const1;

            double part2 = Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
            part2 = part2 * const2;

            double part3 = -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
            part3 = part3 * const3;

            double part4 = Math.pow(t, 3) - Math.pow(t, 2);
            part4 = part4 * const4;

            double num = part1 + part2 + part3 + part4;

            distances[g] = Math.sqrt(Math.pow(j - goalPoint.x, 2) + (Math.pow(num - goalPoint.y, 2)));
            points.add(new Point(j, num));
            //System.out.println("Point " + g + ": (" + j + ", " + num + ")");
            g++;

        }
        for (int q = 0; q < distances.length; q++) {
            if (distances[q] != 0) {
                //System.out.println(distances[q]);
            }
        }
        //Points empty at every new xs[i]

        double xDif = delta;
        if (!points.isEmpty()) {
            int index = closest(distances, delta);
            xDif = Math.abs(goalPoint.x - points.get(index).x);
            //System.out.println("Closest: " + distances[index] + " at point: " + index);
            //System.out.println("x delta: " + xDif);

        }

        //System.out.println(xDif);
        return xDif;
    }

    public static int closest(double[] numbers, double myNumber) {
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
}