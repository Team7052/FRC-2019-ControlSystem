package frc.robot.motionProfiling;

import java.util.ArrayList;

import frc.robot.sequencing.Step;
import frc.robot.sequencing.StepState;

public class TrapezoidalProfileFilter extends Step<MotionTriplet> {
    Filter<MotionTriplet> filter;
    public TrapezoidalProfileFilter(TrapezoidShape velocityShape, Filter<MotionTriplet> filter){
        super();
        this.totalRunningTime = () -> velocityShape.isValidShape() ? velocityShape.totalTime() : 0.0;
        this.filter = filter;
    }

    double runningIntegral = 0.0;
    double prevVelocity = 0.0;
    double prevTimeStamp = 0;

    public double index = 0;
    public MotionTriplet update(double timeStamp) {
        if (state == StepState.RUNNING) {
            if (this.totalRunningTime.get() == 0) return null;
            if (prevTimeStamp == 0) prevTimeStamp = timeStamp;
            double percentage = (timeStamp - this.startTime) / this.totalRunningTime.get();
            if (percentage >= 1.0) {
                this.endStep();
                return null;
            }

            // calculate on the trapezoidal shape the current velocity
            
            return null;
        }
        else {
            return null;
        }
    }

    @Override
    public MotionTriplet getLastUpdate() {
        return null;
    }

    private ArrayList<Point> pointsWithInitialDisplacement(ArrayList<Point> points, double initialDisplacement) {
        ArrayList<Point> converted = new ArrayList<>();
        for (Point point: points) {
            converted.add(new Point(point.x, point.y + initialDisplacement));
        }
        return converted;
    }
}