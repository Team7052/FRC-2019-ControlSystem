package frc.auto;

import frc.robot.helpers.Pair;
import frc.robot.sequencing.Step;

public class TurnToAngleStep extends Step<Pair<Double>> {

    private final double radiusBase = 11.5;
    private final double radiusWheel = 3;
    private double beginningTurnConst = 2.00;
    private double theta = 0;

    public TurnToAngleStep(double angle, boolean degrees) {
        if (degrees) theta = angle / 180.0 * Math.PI;
        else theta = angle;
        double numRotations = (theta*radiusBase)/(2*Math.PI*radiusWheel);
        this.totalRunningTime = () ->  this.beginningTurnConst * numRotations;
    }
    
    @Override
    public Pair<Double> getUpdateForDeltaTime(double dt) {
        if (dt < this.totalRunningTime.get()) {
            return new Pair<>(-0.3, 0.3);
        }
        return new Pair<>(0.0, 0.0);
    }
}