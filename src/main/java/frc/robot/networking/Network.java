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
        this.sendArmMotorData(Motor.SHOULDER_JOINT, RobotMap.kArmShoulderMotorName);
        this.sendArmMotorData(Motor.ELBOW_JOINT, RobotMap.kArmElbowMotorName);
        this.sendArmMotorData(Motor.WRIST_JOINT, RobotMap.kArmWristMotorName);
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
        Setter.setDoubleArray(motionProfilingTable.getEntry("time"), timeFunction);
        Setter.setDoubleArray(motionProfilingTable.getEntry("position"), positionValues);
        Setter.setDoubleArray(motionProfilingTable.getEntry("velocity"), velocityValues);
        Setter.setDoubleArray(motionProfilingTable.getEntry("acceleration"), accelerationValues);
        Setter.setDouble(motionProfilingTable.getEntry("timeStamp"), motionProfiler.getStartTime());
    }

    public void sendNullMotionProfileData(String key) {
        NetworkTable motionProfilingTable = this.getTable(TableType.kMotionProfiles).getSubTable(key);
        double[] empty = new double[0];
        Setter.setDoubleArray(motionProfilingTable.getEntry("time"), empty);
        Setter.setDoubleArray(motionProfilingTable.getEntry("position"), empty);
        Setter.setDoubleArray(motionProfilingTable.getEntry("velocity"), empty);
        Setter.setDoubleArray(motionProfilingTable.getEntry("acceleration"), empty);
    }

    private void sendArmMotorData(Motor motor, String name) {
        ArmSubsystem arm = ArmSubsystem.getInstance();
        double output = arm.getMotorOutputPercent(motor);
        double voltage = arm.getMotorOutputVoltage(motor);
        double current = arm.getCurrent(motor);
        int rawPosition = arm.getPosition(motor);
        double relativeDegrees = arm.getDegrees(motor);
        double absoluteDegrees = arm.getAbsoluteDegrees(motor);
        double rawVelocity = arm.getRawVelocity(motor);
        double degreesVelocity = arm.getVelocityDegrees(motor);
        boolean inPhase = arm.getPhase(motor);
        boolean isInverted = arm.isInverted(motor);
        
        NetworkTable motorSubtable = this.getTable(TableType.kMotorData).getSubTable(name);
        Setter.setDouble(motorSubtable.getEntry("voltage"), voltage);
        Setter.setDouble(motorSubtable.getEntry("percentOutput"), output);
        Setter.setDouble(motorSubtable.getEntry("current"), current);
        Setter.setDouble(motorSubtable.getEntry("rawPosition"), rawPosition);
        Setter.setDouble(motorSubtable.getEntry("relativeDegrees"), relativeDegrees);
        Setter.setDouble(motorSubtable.getEntry("absoluteDegrees"), absoluteDegrees);
        Setter.setDouble(motorSubtable.getEntry("rawVelocity"), rawVelocity);
        Setter.setDouble(motorSubtable.getEntry("degreesVelocity"), degreesVelocity);
        Setter.setBoolean(motorSubtable.getEntry("inPhase"), inPhase);
        Setter.setBoolean(motorSubtable.getEntry("isInverted"), isInverted);
        Setter.setDouble(motorSubtable.getEntry("timeStamp"), Timer.getFPGATimestamp());
    }

    public void sendSparkData(String name, Spark motor) {
        NetworkTable motorSubtable = this.getTable(TableType.kMotorData).getSubTable(name);
    }

    @Override
    public void beganMotionProfile(String command, MotionProfiler profiler) {
        this.motionProfilesStarted.put(command, profiler);
    }
}