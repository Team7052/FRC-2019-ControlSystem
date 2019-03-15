package frc.robot.networking;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;

public class Setter {
    public static void setDouble(NetworkTableEntry entry, double value) {
        if (entry.getType() == NetworkTableType.kDouble) {
            if (entry.getValue().getDouble() != value) entry.setDouble(value);
        }
        else if (entry.getType() == NetworkTableType.kUnassigned) {
            entry.setDouble(value);
        }
    }

    public static void setDoubleArray(NetworkTableEntry entry, double[] value) {
        if (entry.getType() == NetworkTableType.kDoubleArray) {
            if (entry.getValue().getDoubleArray().length != value.length) entry.setDoubleArray(value);
        }
        else if (entry.getType() == NetworkTableType.kUnassigned) {
            entry.setDoubleArray(value);
        }
    }

    public static void setString(NetworkTableEntry entry, String value) {
        if (entry.getType() == NetworkTableType.kString) {
            if (!entry.getValue().getString().equals(value)) entry.setString(value);
        }
        else if (entry.getType() == NetworkTableType.kUnassigned) {
            entry.setString(value);
        }
    }

    public static void setStringArray(NetworkTableEntry entry, String[] value) {
        if (entry.getType() == NetworkTableType.kStringArray) {
            if (entry.getValue().getStringArray().length != value.length) entry.setStringArray(value);
        }
        else if (entry.getType() == NetworkTableType.kUnassigned) {
            entry.setStringArray(value);
        }
    }

    public static void setBoolean(NetworkTableEntry entry, boolean value) {
        if (entry.getType() == NetworkTableType.kBoolean) {
            if (entry.getValue().getBoolean() != value) entry.setBoolean(value);
        }
        else if (entry.getType() == NetworkTableType.kUnassigned) {
            entry.setBoolean(value);
        }
    }
}