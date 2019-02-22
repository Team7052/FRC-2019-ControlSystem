package frc.robot.networking;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;
import frc.robot.tests.TestManager;
import frc.robot.tests.TestManagerState;
import frc.robot.helpers.RotationMotor;

public class Network {
    // get default instance of the network
    private final String kMotorDataTable = "motorData";
    private final String kArmKinematicsTable = "armKinematics";
    private final String kDriveBaseMotionProfilingTable = "driveBaseProfiling";
    private final String kRobotConfigTable = "robotInfo";
    private final String kCalibrationCommTable = "calibrationCommunication";

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
        switch(type) {
            case MOTOR_DATA:
                return networkInstance.getTable(this.kMotorDataTable);
            case ARM_KINEMATICS:
                return networkInstance.getTable(this.kArmKinematicsTable);
            case DRIVE_BASE_MOTION_PROFILING:
                return networkInstance.getTable(this.kDriveBaseMotionProfilingTable);
            case ROBOT_CONFIG_TABLE:
                return networkInstance.getTable(this.kRobotConfigTable);
        }
        return null;
    }

    public NetworkTableEntry getTableEntry(TableType type, String key) {
        return this.getTable(type).getEntry(key);
    }

    public void updateRobotState() {
        NetworkTableEntry entry = this.getTableEntry(TableType.ROBOT_CONFIG_TABLE, "robotState");

        if (RobotState.isDisabled()) entry.setString("disabled");
        else if (RobotState.isEnabled()) {
            if (RobotState.isOperatorControl()) entry.setString("teleop");
            else if (RobotState.isAutonomous()) entry.setString("auto");
            else if (RobotState.isTest()) entry.setString("test");
        }
    }

    /*
        Network architecture:
            /motorData  /<motor>    /<value>
            /robotInfo  /state
                        /<values>
                        /testManager/state
                                    /<values>
    */

    public void sendDataToServer() {
        /*ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
        TestManager testManager = TestManager.getInstance();

        this.sendMotorData(armSubsystem.getWristMotor(), RobotMap.kArmWristMotorName);
        this.sendMotorData(armSubsystem.getRotationMotor(Motor.ELBOW_JOINT), RobotMap.kArmElbowMotorName);
        this.sendMotorData(armSubsystem.getRotationMotor(Motor.SHOULDER_JOINT), RobotMap.kArmShoulderMotorName);

        this.getTable(TableType.ROBOT_CONFIG_TABLE).getSubTable("testManager").getEntry("state").setString(testManager.getState().toString());
        
        // send test manager data, calibration
        NetworkTable calibTable = this.getTable(TableType.CALIBRATION_COMM_TABLE);
        calibTable.getEntry("isCalibrating").setBoolean(testManager.getState() == TestManagerState.CALIBRATING);
        */
    }

    private void sendMotorData(VictorSPX victorSPXMotor, String name) {
        double output = victorSPXMotor.getMotorOutputPercent();
        double voltage = victorSPXMotor.getMotorOutputVoltage();
        NetworkTable motorSubtable = this.getTable(TableType.MOTOR_DATA).getSubTable(name);

        NetworkTableEntry voltageEntry = motorSubtable.getEntry("voltage");
        NetworkTableEntry outputEntry = motorSubtable.getEntry("percentOutput");
        voltageEntry.setDouble(voltage);
        outputEntry.setDouble(output);
    }

    private void sendMotorData(RotationMotor motor, String name) {
        double output = motor.getMotorOutputPercent();
        double voltage = motor.getMotorOutputVoltage();
        double current = motor.getOutputCurrent();
        int rawPosition = motor.getPosition();
        double degrees = motor.getCurrentDegrees();
        double velocity = motor.getVelocity();
        boolean inPhase = motor.getPhase();
        boolean isInverted = motor.getInverted();

        NetworkTable motorSubtable = this.getTable(TableType.MOTOR_DATA).getSubTable(name);
        
        motorSubtable.getEntry("voltage").setDouble(voltage);
        motorSubtable.getEntry("percentOutput").setDouble(output);
        motorSubtable.getEntry("current").setDouble(current);
        motorSubtable.getEntry("rawPosition").setDouble(rawPosition);
        motorSubtable.getEntry("degrees").setDouble(degrees);
        motorSubtable.getEntry("velocity").setDouble(velocity);
        motorSubtable.getEntry("inPhase").setBoolean(inPhase);
        motorSubtable.getEntry("isInverted").setBoolean(isInverted);
    }
}