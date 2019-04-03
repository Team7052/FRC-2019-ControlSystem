package frc.auto;

import java.util.ArrayList;

import frc.robot.motionProfiling.Point;

public class Spline {
    private ArrayList<Point> path, cubicSpline, tangents;
    private double[] xs, ys;

    public Spline(ArrayList<Point> path) {
        this.path = path;
        this.cubicSpline = this.generateCubicSpline(path);
    }

    public ArrayList<Point> getInitialPath() {
        return this.path;
    }

    public double getInitialTurnAngle() {
        if (this.path.size() < 2) return 0;
        Point initPoint = this.path.get(0);
        Point secondPoint = this.path.get(1);
        double theta = Math.atan(Math.abs((secondPoint.getY() - initPoint.getY()) / (secondPoint.getX() - initPoint.getX())));
        if (secondPoint.getX() - initPoint.getX() < 0) theta += Math.PI;
        return theta;
    }
    
    public ArrayList<Point> getCubicSpline() {
        return this.cubicSpline;
    }
    
    public double[] get_xs() {
        return xs;
    }
    public double[] get_ys() {
        return ys;
    }

    public ArrayList<Point> getTangents() {
        return this.tangents;
    }

    private ArrayList<Point> generateCubicSpline(ArrayList<Point> path) {
        //Begin cubic interpolation
        xs = new double[path.size()];
        ys = new double[path.size()];

        for (int i = 0; i < path.size(); i++) {
            xs[i] = path.get(i).getX();
            ys[i] = path.get(i).getY();
        }
        this.tangents = this.calcTangents(path);
        ArrayList<Point> cubic = new ArrayList<Point>();

        ArrayList<Point> otherP = this.calcFinalPoints(xs, ys, tangents);
        cubic.addAll(otherP);
        

        return cubic;
    }

    private ArrayList<Point> calcFinalPoints(double[] xs, double[] ys, ArrayList<Point> tangents) {
        ArrayList<Point> finalPoints = new ArrayList<>();
        double delta = 0.1;
        for (int i = 0; i < tangents.size(); i++) {
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

            if (xs[i] != xs[xs.length - 1]) {
                if (xs[i] == xs[i + 1]) {
                    for (double y = ys[i]; y <= ys[i + 1]; y = y + delta) {
                        finalPoints.add(new Point(xs[i], y));
                    }
                } else {
                    double x = xs[i];
                    while (x <= xs[i + 1]) {
                        double t = SplineMethods.calct(xs, i, x);
                        double part1 = 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
                        part1 = part1 * const1;

                        double part2 = Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
                        part2 = part2 * const2;

                        double part3 = -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
                        part3 = part3 * const3;

                        double part4 = Math.pow(t, 3) - Math.pow(t, 2);
                        part4 = part4 * const4;

                        double num = part1 + part2 + part3 + part4;
                        //x = Math.round(x * 100.0) / 100.0;
                        //num = Math.round(num * 100.0) / 100.0;

                        finalPoints.add(new Point(x, num));

                        if (Math.sqrt(Math.pow(xs[i + 1] - x, 2) + (Math.pow(num - ys[i + 1], 2))) <= 0.1 && Math.sqrt(Math.pow(xs[i + 1] - x, 2) + (Math.pow(num - ys[i + 1], 2))) != 0) {
                            x = xs[i + 1];
                        } else {
                            x = x + SplineMethods.xDiff(xs, ys, tangents, i, delta, new Point(x, num));
                        }
                        //System.out.println("X delta: " + xDiff(xs, ys, tangents, i, delta));

                    }
                }
            }
        }
        return finalPoints;
    }

    private ArrayList<Point> calcTangents(ArrayList<Point> points) {
        ArrayList<Point> tangents = new ArrayList<>();
        ArrayList<Point> secants = this.calcSecants(points);
        double tangent = 0;
        double alpha = 0;
        double beta = 0;
        boolean run = true;
        boolean doubleInc = false;
        int i = 0;
        while (i < points.size()) {
            if (i == 0) {
                tangents.add(new Point(i, secants.get(i).getY()));
                run = false;
            } else if (i == points.size() - 1) {
                tangents.add(new Point(i, secants.get(i - 1).getY()));
                run = false;
            } else if (secants.get(i).getY() == 0) {
                tangents.add(new Point(i, 0));
                tangents.add(new Point(i + 1, 0));
                doubleInc = true;
                run = false;
            } else if (secants.get(i).getY() < 0 && secants.get(i).getY() > 0) {
                tangent = 0;
            } else if (secants.get(i).getY() > 0 && secants.get(i).getY() < 0) {
                tangent = 0;
            } else {
                tangent = (secants.get(i - 1).getY() + secants.get(i).getY()) / 2;
            }
            if (run) {
                if (secants.get(i).getY() != 0) {
                    alpha = tangent / secants.get(i).getY();
                }
                if (secants.get(i - 1).getY() != 0) {
                    beta = tangent / secants.get(i - 1).getY();
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

    private ArrayList<Point> calcSecants(ArrayList<Point> points) {
        ArrayList<Point> secants = new ArrayList<>();
        double secant = 0;
        for (int i = 0; i < points.size() - 1; i++) {
            secant = (points.get(i + 1).getY() - points.get(i).getY()) / (points.get(i + 1).getX() - points.get(i).getX());

            secants.add(new Point(i, secant));

        }

        return secants;
    }
}