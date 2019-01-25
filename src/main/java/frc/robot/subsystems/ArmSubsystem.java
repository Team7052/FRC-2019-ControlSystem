package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.motionProfiling.FunctionGenerator;
import frc.robot.motionProfiling.FunctionSet;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmSubsystem extends Subsystem {
    // static variable that represents the drive train
  public enum Motor {
    SHOULDER_JOINT
  }
  private static ArmSubsystem instance;
  private WPI_TalonSRX shoulderJointMotor;
  private final double shoulderJointMotor_kP = 0.2;
  private final double shoulderJointMotor_kI = 0;//0.0015;
  private final double shoulderJointMotor_kD = 0;
  private final double shoulderJointMotor_kF = 0.0;

  MotionProfiler motionProfiler;

  // always get the current instance of the drive train
  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  // home position is motor as of prototype one of the arm
  public final int homePosition = 2850;
  // going positive degrees is negative for quadrature
  int spinDirection = -1;

  int targetPosition;

  // private initializer so you can't initialize more than 1 drive train
  private ArmSubsystem() {
    // set up the new arm motor
    this.targetPosition = homePosition;

    shoulderJointMotor = new WPI_TalonSRX(RobotMap.ARM_SHOULDER_JOINT_MOTOR);

    shoulderJointMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);
    
    shoulderJointMotor.configNominalOutputForward(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configNominalOutputReverse(0, RobotMap.kPIDTimeoutMillis);
		shoulderJointMotor.configPeakOutputForward(0.3, RobotMap.kPIDTimeoutMillis);
    shoulderJointMotor.configPeakOutputReverse(-0.3, RobotMap.kPIDTimeoutMillis);
    
    shoulderJointMotor.config_kP(RobotMap.kPIDIdx, this.shoulderJointMotor_kP);
    shoulderJointMotor.config_kI(RobotMap.kPIDIdx, this.shoulderJointMotor_kI);
    shoulderJointMotor.config_kD(RobotMap.kPIDIdx, this.shoulderJointMotor_kD);
    shoulderJointMotor.config_kF(RobotMap.kPIDIdx, this.shoulderJointMotor_kF);

    int absolutePosition = shoulderJointMotor.getSensorCollection().getPulseWidthPosition();
    shoulderJointMotor.setSelectedSensorPosition(absolutePosition, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);

    shoulderJointMotor.configAllowableClosedloopError(10, RobotMap.kPIDIdx, RobotMap.kPIDTimeoutMillis);

    motionProfiler = new MotionProfiler();
    FunctionSet rise = new FunctionSet((x) -> x, 0, Math.PI / 6, 0.01);
    FunctionSet straight = new FunctionSet((x) -> x * 0 + Math.PI / 6, Math.PI / 6 + 0.01, 10, 0.01);
    FunctionSet fall = new FunctionSet((x) -> -x + 10 + Math.PI / 6, 10 + 0.01, 13, 0.01);

    motionProfiler.setVelocityPoints(FunctionGenerator.generate(rise, straight, fall));
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // define the Trigger drive here
  }

  double armLength = 0.5207; // in inches
    double centerOfMass = 1360; // pounds at l / 2;
    double gravity = 9.806;

    public double getInertia() {
        return 1/3 * centerOfMass * armLength * armLength;
    }

    public double getTorque(double alpha, double theta) {
        return getInertia() * alpha + 0.5 * centerOfMass * gravity * armLength * Math.cos(theta);
    }

    public double getCurrent(double torque) {
        return 140 / 86 * (torque * 86/140);
  }

  int loop = 0;
  public void setDegrees(Motor motor, double degrees) {
    WPI_TalonSRX selectedMotor = getMotor(motor);
    
    // convert to quadrature postition
    int quadratureDegrees = (int) Math.round(degrees / 360.0 * 4096.0) * spinDirection; // convert to a quadrature
    if (this.targetPosition != homePosition + quadratureDegrees) {
      this.targetPosition = homePosition + quadratureDegrees;
      
    }

    MotionTriplet triplet = motionProfiler.updateMotionProfile(2000);
    if (triplet == null) return;
    double target = triplet.velocity;
    double torque = this.getTorque(triplet.acceleration, triplet.position);
    double current = this.getCurrent(torque);

    selectedMotor.set(ControlMode.Current, current);
    if (loop++ % 10 == 0) {
      System.out.println(target);
    }
  }

  public void stop(Motor motor) {
    WPI_TalonSRX selectedMotor = getMotor(motor);
    selectedMotor.set(ControlMode.Current, 0);
  }

  public void setSpeed(Motor motor, double speed) {
    WPI_TalonSRX selectedMotor = getMotor(motor);
    selectedMotor.set(ControlMode.PercentOutput, speed);
  }

  private void setTarget(int newPosition) {
    if (this.targetPosition != newPosition) {
      this.targetPosition = newPosition;
      
    }
  }

  private WPI_TalonSRX getMotor(Motor motor) {
    WPI_TalonSRX selected = this.shoulderJointMotor;
    switch(motor) {
      case SHOULDER_JOINT:
        selected = this.shoulderJointMotor;
        break;
    }
    return selected;
  }
}