package frc.robot;

import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.Point;

import java.util.ArrayList;

public class PhysicsWorld {
    /* all measurements in inches, degrees in radians */
    double baseX = 100;

    double theta1 = 20 / 180 * Math.PI;
    double theta2 = 70 / 180 * Math.PI;

    Point shoulderJoint = new Point(0,0);
    Point elbowJoint = new Point(0,0);
    Point wristJoint = new Point(0,0);
    Point fingerTip = new Point(0,0);

    public PhysicsWorld() {
        // initialize moving parts on the arm
        this.updateWorld(100, this.theta1, this.theta2, false);
    }

    public void updateWorld() {
        this.updateWorld(this.baseX, this.theta1 / Math.PI * 180, this.theta1 / Math.PI * 180, true);
    }

    public void updateWorld(double baseX, double theta1, double theta2, boolean degrees) {
        if (degrees) {
            this.theta1 = theta1 / 180 * Math.PI;
            this.theta2 = theta2 / 180 * Math.PI;
        }
        else {
            this.theta1 = theta1;
            this.theta2 = theta2;
        }
        this.baseX = baseX;

        //forward kinematics for the arm joints
        this.shoulderJoint = new Point(this.baseX + PhysicsConstants.backToArm + PhysicsConstants.thickness / 2, PhysicsConstants.baseHeight + PhysicsConstants.armHeight);
        this.elbowJoint = new Point(this.shoulderJoint.x + PhysicsConstants.upperArm * Math.sin(this.theta1), this.shoulderJoint.y - PhysicsConstants.upperArm * Math.cos(this.theta1));
        this.wristJoint = new Point(this.elbowJoint.x + PhysicsConstants.lowerArm * Math.sin(this.theta2), this.elbowJoint.y - PhysicsConstants.lowerArm * Math.cos(this.theta2));
        this.fingerTip = new Point(this.wristJoint.x + PhysicsConstants.hand, this.wristJoint.y);
    }

    private ArrayList<Point> _interpolate(Point currentPos, Point newPos) {
        double delta_h = newPos.y - currentPos.y;
        double delta_l = newPos.x - currentPos.x;
        ArrayList<Point> points = new ArrayList<>();
        int numberOfLines = 20;
        for (int i = 0; i <= numberOfLines; i++) {
            double h = currentPos.y + delta_h * (i / numberOfLines);
            double l = currentPos.x + delta_l * (i / numberOfLines);
            points.add(new Point(l, h));
        }
        return points;
    }

    public Pair<Double> inverseKinematics(double delta_l, double delta_h) {
        // get the delta h and delta l
        double alpha = Math.atan(delta_h / delta_l);
        if (delta_l < 0) alpha += Math.PI;
        double p = Math.sqrt(delta_l * delta_l + delta_h * delta_h);
        //console.log(delta_l + " " + delta_h);
        double d1 = PhysicsConstants.upperArm, d2 = PhysicsConstants.lowerArm;

        double theta1 = Math.asin((-Math.pow(d2, 2) + Math.pow(d1, 2) + delta_l * delta_l + delta_h * delta_h) / (2 * d1 * p)) - alpha;
        double theta2 = Math.PI - Math.asin((-Math.pow(d1, 2) + Math.pow(d2, 2) + delta_l * delta_l + delta_h * delta_h) / (2 * d2 * p)) - alpha;
        //console.log(theta1 + " " + theta2);
        // console.log((this.wristJoint.x - this.shoulderJoint.x) + " " + (this.lowerArm + this.upperArm));
        return new Pair(theta1, theta2);
    }

    private boolean isInfinity(double value) {
        return value == Double.POSITIVE_INFINITY || value == Double.NEGATIVE_INFINITY;
    }

    public Point getLengthAndHeight() {
        double current_l = this.wristJoint.x - this.shoulderJoint.x;
        double current_h = this.shoulderJoint.y - this.wristJoint.y;
        return new Point(current_l, current_h);
    }

    public Pair<Double> displaceLengthAndHeight(double delta_l, double delta_h) {
        Point currentDistance = this.getLengthAndHeight();
        double new_l = currentDistance.x + delta_l;
        double new_h = currentDistance.y - delta_h;

        return inverseKinematics(new_l, new_h);
    }

    public ArrayList<Pair<Double>> displaceSequentially(ArrayList<Pair<Double>> deltas) {
        ArrayList<Pair<Double>> angleSequence = new ArrayList<>();
        double counting_l = this.get_l();
        double counting_h = this.get_h();
        for (int i = 0; i < deltas.size(); i++) {
            counting_l += deltas.get(i).a;
            counting_h -= deltas.get(i).b;
            angleSequence.add(inverseKinematics(counting_l, counting_h));
        }
        return angleSequence;
    }

    public double get_l() {
        return this.wristJoint.x - this.shoulderJoint.x;
    }

    public double get_h() {
        return this.shoulderJoint.y - this.wristJoint.y;
    }


    public ArrayList<Pair<Double>> generateTrajectory(double x, double y) {
        // linear interpolate between 
        double new_l = x - this.shoulderJoint.x - PhysicsConstants.hand;
        double new_h = this.shoulderJoint.y - y;

        double current_l = this.wristJoint.x - this.shoulderJoint.x;
        double current_h = this.shoulderJoint.y - this.wristJoint.y;

        ArrayList<Point> length_height_trajectory = this._interpolate(new Point(current_l, current_h), new Point(new_l, new_h));
        ArrayList<Pair<Double>> displacementProfiles = this._generateDisplacementProfiles(length_height_trajectory);
        return displacementProfiles;
    }

    private ArrayList<Pair<Double>> _generateDisplacementProfiles(ArrayList<Point> trajectory) {
        ArrayList<Pair<Double>> profiles = new ArrayList<>();

        for (Point point: trajectory) {
            Pair<Double> anglePair = this.inverseKinematics(point.x, point.y);
            if (!isInfinity(anglePair.a) && !isInfinity(anglePair.b)) {
                profiles.add(new Pair(anglePair.a, anglePair.b));
            }
        }
        return profiles;
    }

}
