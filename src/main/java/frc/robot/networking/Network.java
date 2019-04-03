package frc.robot.networking;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ArmSubsystem.Motor;

public class Network {    
    // new thread for output data

    private static Network instance;
    private NetworkTableInstance networkInstance;

    public static Network getInstance() {
        if (instance == null) instance = new Network();
        return instance;
    }

    private Network() {
        networkInstance = NetworkTableInstance.getDefault();
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

    public void sendDataToServer() {
        // send network data
        this.updateRobotState();

        NetworkTable motorSubtable = this.getTable(TableType.kMotorData).getSubTable("driveBaseLeft");
        motorSubtable.getEntry("percentOutput").setDouble(DriveTrain.getInstance().getLeftSpeed());
        /* motor data */
        // arm motors
        this.sendArmMotorData(Motor.SHOULDER_JOINT, RobotMap.kArmShoulderMotorName);
        this.sendArmMotorData(Motor.ELBOW_JOINT, RobotMap.kArmElbowMotorName);
        this.sendArmMotorData(Motor.WRIST_JOINT, RobotMap.kArmWristMotorName);
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
        boolean isInverted = arm.getInverted(motor);
        
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
}