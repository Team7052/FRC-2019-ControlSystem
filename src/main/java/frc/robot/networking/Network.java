package frc.robot.networking;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Network {
    // get default instance of the network
    private final String kMotorDataTable = "motorData";
    private final String kArmKinematicsTable = "armKinematics";
    private final String kDriveBaseMotionProfilingTable = "driveBaseProfiling";
    private static Network instance;
    private NetworkTableInstance networkInstance;


    //defining default tables

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
        }
    }

    public NetworkTableEntry getTableEntry(TableType type, String key) {
        return this.getTable(type).getEntry(key);

    }
}