package frc.robot.motionProfiling;

import java.util.HashMap;
import java.util.Map;
import java.util.ArrayList;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;

public class MotionProfileManager {
    private final String path = "/home/lvuser/saved_data/motionProfiles.txt";
    private static MotionProfileManager instance;
    private static BufferedReader motionProfileReader;

    public MotionProfileManager getInstance() {
        if (instance == null) {
            instance = new MotionProfileManager();
        }
        return instance;
    }

    HashMap<String, ArrayList<Point>> motionProfiles;

    private MotionProfileManager() {
        // read from file

    }
    
    public void initializeFromFile() {

    }

    public void writeMotionProfilesFromMap() {
        File file = new File(path);
        try {
            // write to file
            BufferedWriter writer = new BufferedWriter(new FileWriter(file));

            for (Map.Entry<String, ArrayList<Point>> entry: motionProfiles.entrySet()) {
                String append = entry.getKey() + ":";
                ArrayList<Point> profile = entry.getValue();
                append += "[";
                for (int i = 0; i < profile.size(); i++) {
                    append += "(" + profile.get(i).x + "," + profile.get(i).y + ")";
                    if (i < profile.size() - 1) append += " ";
                }
                append += "]\n";
                writer.write(append);
            }

            writer.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void readFile() {
        File file = new File(path);
        try {
            BufferedReader reader = new BufferedReader(new FileReader(file));

            // parse the file
            while (reader.ready()) {
                String line = reader.readLine();
                line.split(":");
            }

            reader.close();
        }
        catch(IOException e) {
            e.printStackTrace();
        }
    }
}