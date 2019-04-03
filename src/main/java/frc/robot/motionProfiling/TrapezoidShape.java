package frc.robot.motionProfiling;

public class TrapezoidShape {
    private boolean goodShape;
    private Point p1, p2, p3, p4;

    /* If it is a triangle, then p2 = p3*/

    public TrapezoidShape(Point p1, Point p2, Point p3, Point p4) {
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.p4 = p4;
    }

    public Point[] getPoints() {
        Point[] points = { p1, p2, p3, p4 };
        return points;
    }

    public boolean isValidShape() {
        if (p1.getX() < p2.getX() && p2.getX() <= p3.getX() && p3.getX() < p4.getX()) {
            if (p1.getX() == 0 && p1.getY() == 0 && p4.getY() == 0 && p2.getY() == p3.getY()) return true;
        }
        return false;
    }
    public boolean isTriangle() {
        return this.p2.getX() == p3.getX() && p2.getY() == p3.getY();
    }

    public double totalTime() {
        return this.p4.getX();
    }

    public double getVelocityForTime(double x) {
        if (x < p1.getX()) return 0;
        if (!isValidShape()) return 0;
        if (x <= p2.getX()) {
            return p1.getY() + (p2.getY() - p1.getY()) * (x - p1.getX()) / (p2.getX() - p1.getX());
        }
        else if (x <= p3.getX()) {
            return p2.getY() + (p3.getY() - p2.getY()) * (x - p2.getX()) / (p3.getX() - p2.getX());
        }
        else if (x <= p4.getX()) {
            return p3.getY() + (p4.getY() - p3.getY()) * (x - p3.getX()) / (p4.getX() - p3.getX());
        }
        return 0.0;
    }

    public double getIntegralForTime(double x) {
        if (x < p1.getX()) return 0;
        if (!isValidShape()) return 0;
        double velocity = getVelocityForTime(x);
        if (x <= p3.getX()) {
            double base = x;
            double top = (x <= p2.getX()) ? 0 : x - p2.getX();
            return (base + top) * velocity / 2;
        }
        if (x > p4.getX()) x = p4.getX();
        double p3Area = (2 * p3.getX() - p2.getX()) * p3.getY() / 2;
        return p3Area + (p3.getY() + velocity) * (x - p3.getX()) / 2;
    }

    public double getDerivativeForTime(double x) {
        if (x < p1.getX()) return 0.0;
        if (x <= p2.getX()) return (p2.getY() - p1.getY()) / (p2.getX() - p1.getX());
        if (x <= p3.getX()) return 0.0;
        if (x <= p4.getX()) return (p4.getY() - p3.getY()) / (p4.getX() - p3.getX());
        return 0.0;
    }

    @Override
    public String toString() {
        return "<" + p1 + ", " + p2 + ", " + (!isTriangle()? p3 : "") + ", " + p4 + ">";
    }
}