package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.helpers.ILinearMotor;
import frc.robot.helpers.IRotationMotor;
import frc.robot.helpers.LinearTalonSRX;
import frc.robot.helpers.RotationVictorSPX;
import frc.robot.states.ClimberSuperState;
import frc.robot.util.physics.PhysicsConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climber extends Subsystem {
    private static Climber instance;
    RotationVictorSPX clawMotor;
    LinearTalonSRX rackMotor;
    WPI_VictorSPX driveMotor;

    private ClimberSuperState superState;

    public static Climber getInstance() {
        if (instance == null)  instance = new Climber();
        return instance;
    }

    private Climber() {
        this.clawMotor = new RotationVictorSPX(RobotMap.kClimberClawMotor, RobotMap.kClimberClawMotorEncoderA, RobotMap.kClimberClawMotorEncoderB, RobotMap.kClimberClawMotorGearRatio, RobotMap.kClimberClawMotorEncoderTicks);
        clawMotor.setSensorPhase(false);
        clawMotor.setInverted(true);
        clawMotor.configNominalOutputForward(0, clawMotor.slotIdx);
        clawMotor.configNominalOutputReverse(0, clawMotor.slotIdx);
        clawMotor.configPeakOutputForward(0.9, clawMotor.slotIdx);
        clawMotor.configPeakOutputReverse(-0.9, clawMotor.slotIdx);
        clawMotor.setInvertedPosition(true);
        clawMotor.setHomeDegrees(205);
        clawMotor.setDegreesLimits(60, 210);

        clawMotor.set_kp(0.015);
        clawMotor.set_ki(0.0);
        clawMotor.set_kd(0.0);

        this.rackMotor = new LinearTalonSRX(RobotMap.kClimberRackMotor, RobotMap.kClimberRackPitchDiameter * Math.PI);
        rackMotor.configNominalOutputForward(0, rackMotor.slotIdx);
        rackMotor.configNominalOutputReverse(0, rackMotor.slotIdx);
        rackMotor.configPeakOutputForward(1.0, rackMotor.slotIdx);
        rackMotor.configPeakOutputReverse(-1.0, rackMotor.slotIdx);
        rackMotor.setSensorPhase(true);
        this.rackMotor.setHomeLinearPosition(-PhysicsConstants.climberLegMaxWheelsGroundOffset);
        //this.rackMotor.setDisplacementLimits(-PhysicsConstants.climberLegMaxWheelsGroundOffset, 19.5);
        this.rackMotor.setDisplacementLimits(-20, 19.5);

        rackMotor.set_kp(2.0);
        rackMotor.set_ki(0.0);
        rackMotor.set_kd(0.0);

        driveMotor = new WPI_VictorSPX(14);
        driveMotor.setInverted(true);

        this.superState = new ClimberSuperState();

        System.out.println(rackMotor.getSensorCollection().getPulseWidthPosition());
    }

    public ClimberSuperState getSuperState() {
        return this.superState;
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
    public ILinearMotor getRack() {
        return this.rackMotor;
    }


    public void driveWheelsForward() {
        this.driveMotor.set(ControlMode.PercentOutput, 0.5);
    }

    public void driveWheelsBackward() {
        //this.driveMotor.set(ControlMode.PercentOutput, -0.1);
    }
    public void driveWheelsStop() {
        this.driveMotor.set(ControlMode.PercentOutput, 0);
    }
    
}