package frc.robot.networking;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.Point;
import frc.robot.helpers.RotationMotor;
import frc.robot.helpers.WristMotor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ArmSubsystem.Motor;
import frc.robot.commands.arm.CommandDelegate;

public class Network implements Runnable, CommandDelegate {    
    // new thread for output data
    Thread t;
    ScheduledExecutorService timer;

    private static Network instance;
    private NetworkTableInstance networkInstance;

    public static Network getInstance() {
        if (instance == null) instance = new Network();
        return instance;
    }

    private Network() {
        networkInstance = NetworkTableInstance.getDefault();
        t = new Thread(this, "network-thread");
        timer = Executors.newSingleThreadScheduledExecutor();
        timer.scheduleAtFixedRate(this, 0, 20, TimeUnit.MILLISECONDS);
        t.start();
    }

    public NetworkTable getTable(TableType type) {
        return networkInstance.getTable(type.toString());
    }

    public NetworkTableEntry getTableEntry(TableType type, String key) {
        return this.getTable(type).getEntry(key);
    }

    public void updateRobotState() {
        NetworkTableEntry entry = this.getTableEntry(TableType.kRobotState, "state");
        if (RobotState.isDisabled()) entry.setString("disabled");
        else if (RobotState.isEnabled()) {
            if (RobotState.isOperatorControl()) entry.setString("teleop");
            else if (RobotState.isAutonomous()) entry.setString("auto");
            else if (RobotState.isTest()) entry.setString("test");
        }
    }

    @Override
    public void run() {
        this.sendDataToServer();
    }

    Map<String, MotionProfiler> motionProfilesStarted = new HashMap<>();

    public void sendDataToServer() {
        // send network data
        this.updateRobotState();

        /* Coupled arm profiler */
        for (Map.Entry<String, MotionProfiler> entrySet: motionProfilesStarted.entrySet()) {
            if (entrySet.getValue() != null) {
                this.sendMotionProfileData(entrySet.getKey(), entrySet.getValue());
                motionProfilesStarted.put(entrySet.getKey(), null);
            }
            else {
                this.sendNullMotionProfileData(entrySet.getKey());
            }
        }
        
        NetworkTable motorSubtable = this.getTable(TableType.kMotorData).getSubTable("driveBaseLeft");
        motorSubtable.getEntry("percentOutput").setDouble(DriveTrain.getInstance().getLeftSpeed());
        /* motor data */
        // arm motors
        this.sendWristMotorData();
        this.sendElbowMotorData();
        this.sendShoulderMotorData();
    }

    private void sendMotionProfileData(String name, MotionProfiler motionProfiler) {
        NetworkTable motionProfilingTable = this.getTable(TableType.kMotionProfiles).getSubTable(name);
        ArrayList<Point> positions = motionProfiler.getPositionFunction();
        ArrayList<Point> velocities = motionProfiler.getVelocityFunction();
        ArrayList<Point> accelerations = motionProfiler.getAccelerationFunction();

        int size = positions.size();
        double[] timeFunction = new double[size];
        double[] positionValues = new double[size];
        double[] velocityValues = new double[size];
        double[] accelerationValues = new double[size];

        for (int i = 0; i < size; i++) {
            timeFunction[i] = positions.get(i).x;
            positionValues[i] = positions.get(i).y;
            velocityValues[i] = velocities.get(i).y;
            accelerationValues[i] = accelerations.get(i).y;
        }
        motionProfilingTable.getEntry("time").setDoubleArray(timeFunction);
        motionProfilingTable.getEntry("position").setDoubleArray(positionValues);
        motionProfilingTable.getEntry("velocity").setDoubleArray(velocityValues);
        motionProfilingTable.getEntry("acceleration").setDoubleArray(accelerationValues);
        motionProfilingTable.getEntry("timeStamp").setDouble(motionProfiler.startTime);
    }

    public void sendNullMotionProfileData(String key) {
        NetworkTable motionProfilingTable = this.getTable(TableType.kMotionProfiles).getSubTable(key);
        double[] empty = new double[0];
        motionProfilingTable.getEntry("time").setDoubleArray(empty);
        motionProfilingTable.getEntry("position").setDoubleArray(empty);
        motionProfilingTable.getEntry("velocity").setDoubleArray(empty);
        motionProfilingTable.getEntry("acceleration").setDoubleArray(empty);
    }

    private void sendWristMotorData() {
        WristMotor motor = ArmSubsystem.getInstance().getWristMotor();
        double output = motor.getMotorOutputPercent();
        double voltage = motor.getMotorOutputVoltage();
        double current = motor.getCurrent();
        int rawPosition = motor.getPosition();
        double relativeDegrees = motor.getCurrentDegrees();
        double absoluteDegrees = ArmSubsystem.getInstance().getAbsoluteWristDegrees();
        double rawVelocity = motor.getRawVelocity();
        double degreesVelocity = motor.getVelocityDegrees();
        boolean inPhase = motor.getPhase();
        boolean isInverted = motor.getInverted();
        
        NetworkTable motorSubtable = this.getTable(TableType.kMotorData).getSubTable(RobotMap.kArmWristMotorName);
        motorSubtable.getEntry("voltage").setDouble(voltage);
        motorSubtable.getEntry("percentOutput").setDouble(output);
        motorSubtable.getEntry("current").setDouble(current);
        motorSubtable.getEntry("rawPosition").setDouble(rawPosition);
        motorSubtable.getEntry("relativeDegrees").setDouble(relativeDegrees);
        motorSubtable.getEntry("absoluteDegrees").setDouble(absoluteDegrees);
        motorSubtable.getEntry("rawVelocity").setDouble(rawVelocity);
        motorSubtable.getEntry("degreesVelocity").setDouble(degreesVelocity);
        motorSubtable.getEntry("inPhase").setBoolean(inPhase);
        motorSubtable.getEntry("isInverted").setBoolean(isInverted);
        motorSubtable.getEntry("timeStamp").setDouble(Timer.getFPGATimestamp());
    }

    private void sendElbowMotorData() {
        RotationMotor motor = ArmSubsystem.getInstance().getRotationMotor(Motor.ELBOW_JOINT);
        double output = motor.getMotorOutputPercent();
        double voltage = motor.getMotorOutputVoltage();
        double current = motor.getCurrent();
        int rawPosition = motor.getPosition();
        double relativeDegrees = motor.getCurrentDegrees();
        double absoluteDegrees = ArmSubsystem.getInstance().getAbsoluteElbowDegrees();
        double rawVelocity = motor.getRawVelocity();
        double degreesVelocity = motor.getVelocityDegrees();
        boolean inPhase = motor.getPhase();
        boolean isInverted = motor.getInverted();
        
        NetworkTable motorSubtable = this.getTable(TableType.kMotorData).getSubTable(RobotMap.kArmElbowMotorName);
        motorSubtable.getEntry("voltage").setDouble(voltage);
        motorSubtable.getEntry("percentOutput").setDouble(output);
        motorSubtable.getEntry("current").setDouble(current);
        motorSubtable.getEntry("rawPosition").setDouble(rawPosition);
        motorSubtable.getEntry("relativeDegrees").setDouble(relativeDegrees);
        motorSubtable.getEntry("absoluteDegrees").setDouble(absoluteDegrees);
        motorSubtable.getEntry("rawVelocity").setDouble(rawVelocity);
        motorSubtable.getEntry("degreesVelocity").setDouble(degreesVelocity);
        motorSubtable.getEntry("inPhase").setBoolean(inPhase);
        motorSubtable.getEntry("isInverted").setBoolean(isInverted);
        motorSubtable.getEntry("timeStamp").setDouble(Timer.getFPGATimestamp());
    }

    private void sendShoulderMotorData() {
        RotationMotor motor = ArmSubsystem.getInstance().getRotationMotor(Motor.SHOULDER_JOINT);
        double output = motor.getMotorOutputPercent();
        double voltage = motor.getMotorOutputVoltage();
        double current = motor.getCurrent();
        int rawPosition = motor.getPosition();
        double degrees = motor.getCurrentDegrees();
        double rawVelocity = motor.getRawVelocity();
        double degreesVelocity = motor.getVelocityDegrees();
        boolean inPhase = motor.getPhase();
        boolean isInverted = motor.getInverted();

        NetworkTable motorSubtable = this.getTable(TableType.kMotorData).getSubTable(RobotMap.kArmShoulderMotorName);
        
        motorSubtable.getEntry("voltage").setDouble(voltage);
        motorSubtable.getEntry("percentOutput").setDouble(output);
        motorSubtable.getEntry("current").setDouble(current);
        motorSubtable.getEntry("rawPosition").setDouble(rawPosition);
        motorSubtable.getEntry("absoluteDegrees").setDouble(degrees);
        motorSubtable.getEntry("relativeDegrees").setDouble(degrees);
        motorSubtable.getEntry("rawVelocity").setDouble(rawVelocity);
        motorSubtable.getEntry("degreesVelocity").setDouble(degreesVelocity);
        motorSubtable.getEntry("inPhase").setBoolean(inPhase);
        motorSubtable.getEntry("isInverted").setBoolean(isInverted);
        motorSubtable.getEntry("timeStamp").setDouble(Timer.getFPGATimestamp());
    }

    public void sendSparkData(String name, Spark motor) {
        NetworkTable motorSubtable = this.getTable(TableType.kMotorData).getSubTable(name);
        
    }

    @Override
    public void beganMotionProfile(String command, MotionProfiler profiler) {
        this.motionProfilesStarted.put(command, profiler);
    }
}