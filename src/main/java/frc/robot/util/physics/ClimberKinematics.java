package frc.robot.util.physics;

public class ClimberKinematics {
    public double coupledHeightFromAngle(double angle, double initHabHeight) {
        double alpha = angle - Math.PI / 2;

        double l = PhysicsConstants.climberClawLength;
        double w = PhysicsConstants.climberClawWidth;

        double h2 = l * Math.sin(alpha) - w / Math.cos(alpha);
        return initHabHeight - h2 - PhysicsConstants.climberClawBaseYOffset - PhysicsConstants.baseHeight;
    }

    public double coupledAngleFromHeight(double height, double initHabHeight) {
        double l = PhysicsConstants.climberClawLength;
        double w = PhysicsConstants.climberClawWidth;

        double h2 = initHabHeight - height - PhysicsConstants.baseHeight - PhysicsConstants.climberClawBaseYOffset;
        System.out.println("h2: " + h2 + " height: " + height + " " + Math.sqrt(l*l + w*w));
        return Math.asin(h2 / Math.sqrt(l*l + w*w)) + Math.atan(w / l) + Math.PI / 2;
    }
}