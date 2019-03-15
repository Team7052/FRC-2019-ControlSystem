package frc.auto;

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