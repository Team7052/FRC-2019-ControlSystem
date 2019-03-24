package frc.robot.motionProfiling;

public class TrapezoidShape {
    private boolean goodShape;
    public Point p1, p2, p3, p4;

    public TrapezoidShape(Point p1, Point p2, Point p3, Point p4) {
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.p4 = p4;
        this.goodShape = false;
        if (p1.x < p2.x && p2.x <= p3.x && p3.x < p4.x) {
            if (p1.x == 0 && p1.y == 0 && p4.y == 0 && p2.y == p3.y) goodShape = true;
        }
    }

    public boolean isValidShape() {
        return this.goodShape;
    }
    public boolean isTriangle() {
        return this.p2.x == p3.x && p2.y == p3.y;
    }

    public double totalTime() {
        return this.p4.x;
    }

    public double getVelocityForTime(double x) {
        if (x < p1.x) return 0;
        if (x <= p2.x) {
            return (p2.y - p1.y) * x / (p2.x - p1.x);
        }
        else if (x <= p3.x) {
            return (p3.y - p2.y) * x / (p3.x - p2.x);
        }
        else if (x <= p4.x) {
            return (p4.y - p3.y) * x / (p4.x - p3.x);
        }
        return 0.0;
    }

    /*public double getIntegralForTime(double x) {
            
    }*/
}