package frc.robot.states;

import frc.robot.states.substates.DriveState;

public class DriveTrainSuperState extends SuperState<DriveState> {
    private static DriveTrainSuperState instance;
    public static DriveTrainSuperState getInstance() {
        if (instance == null) instance = new DriveTrainSuperState();
        return instance;
    }

    private DriveTrainSuperState() {
        this.systemState = DriveState.openLoopDrive;
    }

    @Override
    public void update() {
        if (systemState == DriveState.openLoopDrive) {
            // calculate tank drive
        }
    }
}