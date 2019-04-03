package frc.robot.util.physics;

import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.Point;

import java.util.ArrayList;

public class ArmKinematics {
    public Point shoulderJoint = new Point(0,0);
    public Point elbowJoint = new Point(0,0);
    public Point wristJoint = new Point(0,0);
    public Point fingerTip = new Point(0,0);

    public void update(double theta1, double theta2, double theta3) {
        //forward kinematics for the arm joints
        this.shoulderJoint = new Point( PhysicsConstants.baseLength - PhysicsConstants.backToArm + PhysicsConstants.thickness / 2, PhysicsConstants.baseHeight + PhysicsConstants.armHeight);
        this.elbowJoint = new Point(this.shoulderJoint.getX() + PhysicsConstants.upperArm * Math.sin(theta1), this.shoulderJoint.getY() - PhysicsConstants.upperArm * Math.cos(theta1));
        this.wristJoint = new Point(this.elbowJoint.getX() + PhysicsConstants.lowerArm * Math.sin(theta2), this.elbowJoint.getY() - PhysicsConstants.lowerArm * Math.cos(theta2));
        this.fingerTip = new Point(this.wristJoint.getX() + PhysicsConstants.hand, this.wristJoint.getY());
    }

    private ArrayList<Point> _interpolate(Point currentPos, Point newPos) {
        double delta_h = newPos.getY() - currentPos.getY();
        double delta_l = newPos.getX() - currentPos.getX();
        ArrayList<Point> points = new ArrayList<>();
        int numberOfLines = 20;
        for (int i = 0; i <= numberOfLines; i++) {
            double h = currentPos.getY() + delta_h * (i / numberOfLines);
            double l = currentPos.getX() + delta_l * (i / numberOfLines);
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
        // console.log((this.wristJoint.getX() - this.shoulderJoint.getX()) + " " + (this.lowerArm + this.upperArm));
        return new Pair<>(theta1, theta2);
    }

    private boolean isInfinity(double value) {
        return value == Double.POSITIVE_INFINITY || value == Double.NEGATIVE_INFINITY;
    }

    public Pair<Double> getLengthAndHeight() {
        double current_l = this.wristJoint.getX() - this.shoulderJoint.getX();
        double current_h = this.shoulderJoint.getY() - this.wristJoint.getY();
        return new Pair<>(current_l, current_h);
    }

    public Pair<Double> displaceLengthAndHeight(double delta_l, double delta_h) {
        Pair<Double> currentDistance = this.getLengthAndHeight();
        double new_l = currentDistance.getFirst() + delta_l;
        double new_h = currentDistance.getSecond() - delta_h;

        return inverseKinematics(new_l, new_h);
    }

    public ArrayList<Pair<Double>> displaceSequentially(ArrayList<Pair<Double>> deltas) {
        ArrayList<Pair<Double>> angleSequence = new ArrayList<>();
        double counting_l = this.get_l();
        double counting_h = this.get_h();
        for (int i = 0; i < deltas.size(); i++) {
            counting_l += deltas.get(i).getFirst();
            counting_h -= deltas.get(i).getSecond();
            angleSequence.add(inverseKinematics(counting_l, counting_h));
        }
        return angleSequence;
    }

    public double get_l() {
        return this.wristJoint.getX() - this.shoulderJoint.getX();
    }

    public double get_h() {
        return this.shoulderJoint.getY() - this.wristJoint.getY();
    }

    public ArrayList<Pair<Double>> generateTrajectory(double x, double y) {
        // linear interpolate between
        double new_l = x - this.shoulderJoint.getX() - PhysicsConstants.hand;
        double new_h = this.shoulderJoint.getY() - y;

        double current_l = this.wristJoint.getX() - this.shoulderJoint.getX();
        double current_h = this.shoulderJoint.getY() - this.wristJoint.getY();

        ArrayList<Point> length_height_trajectory = this._interpolate(new Point(current_l, current_h), new Point(new_l, new_h));
        ArrayList<Pair<Double>> displacementProfiles = this._generateDisplacementProfiles(length_height_trajectory);
        return displacementProfiles;
    }

    private ArrayList<Pair<Double>> _generateDisplacementProfiles(ArrayList<Point> trajectory) {
        ArrayList<Pair<Double>> profiles = new ArrayList<>();

        for (Point point: trajectory) {
            Pair<Double> anglePair = this.inverseKinematics(point.getX(), point.getY());
            if (!isInfinity(anglePair.getFirst()) && !isInfinity(anglePair.getSecond())) {
                profiles.add(new Pair<Double>(anglePair.getFirst(), anglePair.getSecond()));
            }
        }
        return profiles;
    }


}
