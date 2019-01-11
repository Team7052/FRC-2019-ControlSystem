package frc.joysticks;

import frc.robot.OI;

// TODO: Map out joystick

public class PS4 extends OI {
    public PS4(int port) { super(port); }

    @Override public double axisLeft_X() { return this.getAxis(0); }
    @Override public double axisLeft_Y() { return -this.getAxis(1); }
    @Override public double axisRight_X() { return this.getAxis(2); }
    @Override public double axisRight_Y() { return -this.getAxis(5); }
    @Override public double axisTrigger_L2() { return this.getAxis(3); }
    @Override public double axisTrigger_R2() { return this.getAxis(4); }
    @Override public boolean button_X() { return this.getButton(0); }
    @Override public boolean button_Y() { return this.getButton(3); }
    @Override public boolean button_A() { return this.getButton(1); }
    @Override public boolean button_B() { return this.getButton(2); }
    @Override public boolean button_L1() { return this.getButton(4); }
    @Override public boolean button_L2() { return this.getButton(5); }
    @Override public boolean button_L3() { return this.getButton(10); }
    @Override public boolean button_R1() { return this.getButton(5); }
    @Override public boolean button_R2() { return this.getButton(7); }
    @Override public boolean button_R3() { return this.getButton(11); }
    @Override public boolean button_back() { return this.getButton(13); }
    @Override public boolean button_start() { return this.getButton(9); }

    @Override public boolean dPad_UP() { return false; }
    @Override public boolean dPad_RIGHT() { return false; }
    @Override public boolean dPad_DOWN() { return false; }
    @Override public boolean dPad_LEFT() { return false; }
    @Override public boolean dPad_UP_RIGHT() { return false; }
    @Override public boolean dPad_UP_LEFT() { return false; }
    @Override public boolean dPad_DOWN_RIGHT() { return false; }
    @Override public boolean dPad_DOWN_LEFT() { return false; }
}