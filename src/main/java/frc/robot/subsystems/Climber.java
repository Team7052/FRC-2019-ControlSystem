package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.helpers.ILinearMotor;
import frc.robot.helpers.IRotationMotor;
import frc.robot.helpers.LinearTalonSRX;
import frc.robot.helpers.RotationVictorSPX;
import frc.robot.util.physics.PhysicsConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Climber extends Subsystem {
    private static Climber instance;
    RotationVictorSPX clawMotor;
    LinearTalonSRX legMotor;
    VictorSPX driveMotor;
    public static Climber getInstance() {
        if (instance == null)  instance = new Climber();
        return instance;
    }

    private Climber() {
        this.clawMotor = new RotationVictorSPX(13, 4, 5, 2.0, 1024);
        clawMotor.setSensorPhase(false);
        clawMotor.setInverted(true);
        clawMotor.configNominalOutputForward(0, clawMotor.slotIdx);
        clawMotor.configNominalOutputReverse(0, clawMotor.slotIdx);
        clawMotor.configPeakOutputForward(0.9, clawMotor.slotIdx);
        clawMotor.configPeakOutputReverse(-0.9, clawMotor.slotIdx);
        clawMotor.setInvertedPosition(true);
        clawMotor.setHomeDegrees(180);
        clawMotor.setDegreesLimits(75, 190);

        clawMotor.set_kp(0.015);
        clawMotor.set_ki(0.0);
        clawMotor.set_kd(0.0);
        this.legMotor = new LinearTalonSRX(12, 0.55 * 2 * Math.PI);
        legMotor.configNominalOutputForward(0, legMotor.slotIdx);
        legMotor.configNominalOutputReverse(0, legMotor.slotIdx);
        legMotor.configPeakOutputForward(0.7, legMotor.slotIdx);
        legMotor.configPeakOutputReverse(-0.7, legMotor.slotIdx);
        legMotor.setSensorPhase(true);
        this.legMotor.setHomeLinearPosition(-PhysicsConstants.climberLegMaxWheelsGroundOffset);
        this.legMotor.setDisplacementLimits(-PhysicsConstants.climberLegMaxWheelsGroundOffset, 19.5);
        legMotor.set_kp(2.0);
        legMotor.set_ki(0.0);
        legMotor.set_kd(0.0);

        driveMotor = new VictorSPX(14);
        driveMotor.setInverted(true);

        System.out.println("absolute position: " + this.legMotor.getSensorCollection().getPulseWidthPosition()); 
    }

    @Override
    protected void initDefaultCommand() {
        
    }

    public void setSpeed(double speed) {
        this.clawMotor.set(ControlMode.PercentOutput, speed);
    }

    public IRotationMotor getClaw() {
        return this.clawMotor;
    }
    public ILinearMotor getLeg() {
        return this.legMotor;
    }


    public void driveWheelsForward() {
        this.driveMotor.set(ControlMode.PercentOutput, 0.2);
    }

    public void driveWheelsBackward() {
        //this.driveMotor.set(ControlMode.PercentOutput, -0.1);
    }
    public void driveWheelsStop() {
        this.driveMotor.set(ControlMode.PercentOutput, 0);
    }
    
}