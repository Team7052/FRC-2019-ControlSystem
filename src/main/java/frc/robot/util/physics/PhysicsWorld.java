package frc.robot.util.physics;

import frc.robot.helpers.Pair;
import frc.robot.helpers.Triplet;

public class PhysicsWorld {
    /* all measurements in inches, degrees in radians */
    private ArmKinematics armKinematics;
    private ClimberKinematics climberKinematics;
    private static PhysicsWorld instance;
    public static PhysicsWorld getInstance() {
        if (instance == null) instance = new PhysicsWorld();
        return instance;
    }

    private double armTheta1 = 20 / 180 * Math.PI;
    private double armTheta2 = 70 / 180 * Math.PI;
    private double armTheta3 = 180 / 180 * Math.PI;

    private double climberClawTheta = Math.PI;
    private double climberRackHeight = 0;
    private double baseX = 0;

    private PhysicsWorld() {
        // initialize moving parts on the arm
        armKinematics = new ArmKinematics();
        climberKinematics = new ClimberKinematics();
    }

    public Triplet<Double> getArmAngles() {
        return new Triplet<>(armTheta1, armTheta2, armTheta3);
    }

    public double getClimberClawAngle() {
        return climberClawTheta;
    }

    public double getClimberRackHeight() {
        return climberRackHeight;
    }

    public void updateClimberKinematics(double clawTheta, double legHeight) {
        this.climberClawTheta = clawTheta;
        this.climberRackHeight = legHeight;
    }

    public void updateArmKinematics(double theta1, double theta2, double theta3, boolean degrees) {
        if (degrees) {
            this.armTheta1 = theta1 / 180 * Math.PI;
            this.armTheta2 = theta2 / 180 * Math.PI;
            this.armTheta3 = theta3 / 180 * Math.PI;
        }
        else {
            this.armTheta1 = theta1;
            this.armTheta2 = theta2;
            this.armTheta3 = theta3;
        }
        this.armKinematics.update(this.armTheta1, this.armTheta2, this.armTheta3);
    }

    public Pair<Double> solveArmKinematics() {
        return this.armKinematics.getLengthAndHeight();
    }

    public double solveClimberClawAngleForHeight(double height, double habHeight) {
        return this.climberKinematics.coupledAngleFromHeight(height, habHeight);
    }

    public double solveClimberRackHeightForAngle(double angle, double habHeight) {
        return this.climberKinematics.coupledHeightFromAngle(angle, habHeight);
    }

    public Pair<Double> armInverseKinematics(double delta_l, double delta_h) {
        return this.armKinematics.inverseKinematics(delta_l, delta_h);
    }

}
