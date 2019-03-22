package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class LiftCommand {
    Climber climber;
    public LiftCommand() {
        climber = Climber.getInstance();
    }

    public void execute() {
        if (Robot.oi.dPad_RIGHT()) {
            // up
            climber.getLeg().setPercentOutput(0.3);
            System.out.println(climber.getLeg().getLinearPosition() + " " + climber.getLeg().getLinearVelocity());
        }
        else if (Robot.oi.dPad_LEFT()) {
            climber.getLeg().setPercentOutput(-0.3);
            System.out.println(climber.getLeg().getLinearPosition() + " " + climber.getLeg().getLinearVelocity());
        }
        else {
            climber.getLeg().setPercentOutput(0.0);
        }
    }
}