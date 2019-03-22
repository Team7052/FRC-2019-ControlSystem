package frc.robot.helpers;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;

public class CustomEncoder extends Encoder {
    private int trueGet = 0;
    private int adjustedPositionDelta = 0;

    public CustomEncoder(int encoderA, int encoderB, boolean reverse, EncodingType type) {
        super(encoderA, encoderB, reverse, type);
    }

    public void setSelectedSensorPosition(int newPosition) {
        this.adjustedPositionDelta = newPosition - this.trueGet;
    }

    @Override
    public int get() {
        this.trueGet = super.get();
        return this.trueGet + adjustedPositionDelta;
    }
}