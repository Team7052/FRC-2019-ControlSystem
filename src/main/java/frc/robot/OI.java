package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.joysticks.DPadType;

// a class to represent a joystick mapping of for example Logitech
public abstract class OI {
    Joystick joystick;
    public OI(int port) {
        joystick = new Joystick(port);
    }
    public abstract double axisLeft_X();
    public abstract double axisLeft_Y();
    public abstract double axisRight_X();
    public abstract double axisRight_Y();
    public abstract double axisTrigger_L2();
    public abstract double axisTrigger_R2();
    // buttons
    public abstract boolean button_X();
    public abstract boolean button_Y();
    public abstract boolean button_A();
    public abstract boolean button_B();
    public abstract boolean button_L1();
    public abstract boolean button_L2();
    public abstract boolean button_L3();
    public abstract boolean button_R1();
    public abstract boolean button_R2();
    public abstract boolean button_R3();
    public abstract boolean button_back();
    public abstract boolean button_start();
    // dpad
    public abstract boolean dPad_UP();
    public abstract boolean dPad_RIGHT();
    public abstract boolean dPad_DOWN();
    public abstract boolean dPad_LEFT();
    public abstract boolean dPad_UP_RIGHT();
    public abstract boolean dPad_UP_LEFT();
    public abstract boolean dPad_DOWN_RIGHT();
    public abstract boolean dPad_DOWN_LEFT();

    protected double getAxis(int port) {
        return joystick.getRawAxis(port);
    }
    protected boolean getButton(int port) {
        return joystick.getRawButton(port);
    }

    protected boolean getDPad(DPadType type) {
        if (joystick.getPOVCount() == 0) return false;
        switch (type) {
            case NORTH:
                return joystick.getPOV(0) == 0;
            case NORTHEAST:
                return joystick.getPOV(0) == 45;
            case EAST:
                return joystick.getPOV(0) == 90;
            case SOUTHEAST:
                return joystick.getPOV(0) == 135;
            case SOUTH:
                return joystick.getPOV(0) == 180;
            case SOUTHWEST:
                return joystick.getPOV(0) == 225;
            case WEST:
                return joystick.getPOV(0) == 270;
            case NORTHWEST:
                return joystick.getPOV(0) == 315;
            default:
                return false;
        }
    }
}