package frc.robot.motionProfiling;

import frc.robot.helpers.Pair;

public class Point extends Pair<Double> {
    public Point(double x, double y) {
        super(x, y);
    }

    public double getX() {
        return this.getFirst();
    }
    public double getY() {
        return this.getSecond();
    }

    @Override
    public String toString() {
        return "(" + this.getFirst() + ", " + this.getSecond() +")";
    }
}