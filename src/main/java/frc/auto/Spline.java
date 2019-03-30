package frc.auto;

import java.util.ArrayList;

import frc.robot.helpers.Quadruple;
import frc.robot.motionProfiling.Point;

public class Spline {
    private ArrayList<Point> path, cubicSpline, tangents;
    private Quadruple<Double> constants;

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
        double theta = Math.atan(Math.abs((secondPoint.y - initPoint.y) / (secondPoint.x - initPoint.x)));
        if (secondPoint.x - initPoint.x < 0) theta += Math.PI;
        return theta;
    }
    
    public ArrayList<Point> getCubicSpline() {
        return this.cubicSpline;
    }


    public ArrayList<Point> getTangents() {
        return this.tangents;
    }

    public Quadruple<Double> getConstants(int index) {
        return this.calcConstants(this.path, this.tangents, index);
    }

    private ArrayList<Point> generateCubicSpline(ArrayList<Point> path) {
        //Begin cubic interpolation
        this.tangents = this.calcTangents(path);

        ArrayList<Point> cubic = this.calcFinalPoints(path, tangents);

        return cubic;
    }

    private ArrayList<Point> calcFinalPoints(ArrayList<Point> splinePoints, ArrayList<Point> tangents) {
        ArrayList<Point> finalPoints = new ArrayList<>();
        double delta = 0.3;
        for (int i = 0; i < tangents.size(); i++) {
            Quadruple<Double> constants = calcConstants(splinePoints, tangents, i);

            if (splinePoints.get(i).x != splinePoints.get(splinePoints.size() - 1).x) {
                if (splinePoints.get(i).x == splinePoints.get(i + 1).x) {
                    for (double y = splinePoints.get(i).y; y <= splinePoints.get(i + 1).y; y = y + delta) {
                        finalPoints.add(new Point(splinePoints.get(i).x, y));
                    }
                } else {
                    double x = splinePoints.get(i).x;
                    while (x <= splinePoints.get(i + 1).x) {
                        double t = calct(splinePoints, i, x);
                        double part1 = 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
                        part1 = part1 * constants.a;

                        double part2 = Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
                        part2 = part2 * constants.b;

                        double part3 = -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
                        part3 = part3 * constants.c;

                        double part4 = Math.pow(t, 3) - Math.pow(t, 2);
                        part4 = part4 * constants.d;

                        double num = part1 + part2 + part3 + part4;
                        //x = Math.round(x * 100.0) / 100.0;
                        //num = Math.round(num * 100.0) / 100.0;

                        finalPoints.add(new Point(x, num));

                        if (Math.sqrt(Math.pow(splinePoints.get(i + 1).x - x, 2) + (Math.pow(num - splinePoints.get(i + 1).y, 2))) <= 0.1 && Math.sqrt(Math.pow(splinePoints.get(i + 1).y - x, 2) + (Math.pow(num - splinePoints.get(i + 1).y, 2))) != 0) {
                            x = splinePoints.get(i + 1).x;
                        } else {
                            x = x + xDiff(splinePoints, tangents, i, delta, new Point(x, num));
                        }
                        //System.out.println("X delta: " + xDiff(xs, ys, tangents, i, delta));

                    }
                }
            }
        }
        return finalPoints;
    }

    private ArrayList<Point> calcSecants(ArrayList<Point> points) {
        ArrayList<Point> secants = new ArrayList<>();
        double secant = 0;
        for (int i = 0; i < points.size() - 1; i++) {
            secant = (points.get(i + 1).y - points.get(i).y) / (points.get(i + 1).x - points.get(i).x);

            secants.add(new Point(i, secant));

        }

        return secants;
    }

    public final double delta = 0.02;
    public final double k_value = 5.0 * delta;

    public double xDiff(ArrayList<Point> splinePoints, ArrayList<Point> tangents, int i, double delta, Point goalPoint) {
        ArrayList<Point> points = new ArrayList<>();
        Quadruple<Double> constants = calcConstants(splinePoints, tangents, i);
        double[] distances = new double[99999];
        int g = 0;

        //System.out.println("Desired Point: (" + xs[i] + ", " + ys[i] + ")");
        for (double j = goalPoint.x + 0.1; j <= splinePoints.get(i + 1).x / 2; j = j + 0.1) {
            double t = calct(splinePoints, i, j);
            double part1 = 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
            part1 = part1 * constants.a;

            double part2 = Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
            part2 = part2 * constants.b;

            double part3 = -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
            part3 = part3 * constants.c;

            double part4 = Math.pow(t, 3) - Math.pow(t, 2);
            part4 = part4 * constants.d;

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

    public double calct(ArrayList<Point> splinePoints, int i, double x) {
        double h = calch(splinePoints, i);
        return (x - splinePoints.get(i).x / h);
    }

    private double calch(ArrayList<Point> splinePoints, int index) {
        return index == splinePoints.size() - 1 ? 0 : splinePoints.get(index + 1).x - splinePoints.get(index).x;
    }


    private Quadruple<Double> calcConstants(ArrayList<Point> splinePoints, ArrayList<Point> tangents, int index) {
        if (splinePoints.size() != tangents.size() || index >= splinePoints.size()) return null;
        int size = splinePoints.size();
        double yLow = splinePoints.get(index).y;
        double mLow = tangents.get(index).y;
        double yHigh = index == size - 1 ? splinePoints.get(index).y : splinePoints.get(index + 1).y;
        double mHigh  = index == size - 1 ? tangents.get(index).y : tangents.get(index + 1).y;
        double h = calch(splinePoints, index);
        
        return new Quadruple<>(yLow, h * mLow, yHigh, h * mHigh);
    }

    private Quadruple<Double> calculateParts(double t) {
        double part1 = 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
        double part2 = Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
        double part3 = -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
        double part4 = Math.pow(t, 3) - Math.pow(t, 2);
        return new Quadruple<>(part1, part2, part3, part4);
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
}