package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class LiftCommand {
    Climber climber;
    public LiftCommand() {
        climber = Climber.getInstance();
    }

    public void execute() {
        if (Robot.oi2.dPad_RIGHT()) {
            // up
            climber.getRack().setPercentOutput(0.3);
            System.out.println(climber.getRack().getLinearPosition() + " " + climber.getRack().getLinearVelocity());
        }
        else if (Robot.oi2.dPad_LEFT()) {
            climber.getRack().setPercentOutput(-0.3);
            System.out.println(climber.getRack().getLinearPosition() + " " + climber.getRack().getLinearVelocity());
        }
        else {
            climber.getRack().setPercentOutput(0.0);
        }
    }
}