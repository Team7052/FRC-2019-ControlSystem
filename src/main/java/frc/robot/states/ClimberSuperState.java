package frc.robot.states;

import frc.robot.commands.CoupledLiftProfiler;
import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionProfiler;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.sequencing.Sequence;
import frc.robot.states.substates.ClimberState;
import frc.robot.subsystems.Climber;
import frc.robot.util.physics.PhysicsWorld;

public class ClimberSuperState extends SuperState<ClimberState> {
    private static ClimberSuperState instance;
    public static ClimberSuperState getInstance() {
        if (instance == null) return instance = new ClimberSuperState();
        return instance;
    }

    public enum ClimberDeployState {
        notDeployed, shouldDeploy, deploy
    }

    MotionProfileState clawState = MotionProfileState.IDLE;
    MotionProfileState legState = MotionProfileState.IDLE;

    private ClimberDeployState deployState;

    private ClimberSuperStateDelegate clawDelegate;
    private ClimberSuperStateDelegate rackDelegate;


    private ClimberSuperState() {
        this.systemState = ClimberState.safelyStowed;
        this.deployState = ClimberDeployState.notDeployed;
    }

    public void setClawDelegate(ClimberSuperStateDelegate delegate) {
        this.clawDelegate = delegate;
    }

    public void setRackDelegate(ClimberSuperStateDelegate delegate) {
        this.rackDelegate = delegate;
    }

    @Override
    public void setState(ClimberState state) {
        if (state == this.systemState) return;
        if (this.systemState == ClimberState.safelyStowed) {
            this.systemState = state;
            this.deployState = ClimberDeployState.shouldDeploy;
        }
        else if (this.deployState == ClimberDeployState.deploy) {
            if (state == ClimberState.home) {
                this.systemState = state;
                this.deployState = ClimberDeployState.shouldDeploy;
            }
        }
    }

    @Override
    public void update() {
        if (this.deployState == ClimberDeployState.shouldDeploy) {
            this.deployState = ClimberDeployState.deploy;
            Pair<Sequence<MotionTriplet>> sequence = null;
                // generate sequence for a hab climb.
            if (systemState == ClimberState.hab2Climb) {
                sequence = HabClimbSequences.hab2ClimbSequence();
            }
            else if (systemState == ClimberState.hab3Climb) {
                sequence = HabClimbSequences.hab3ClimbSequence();
            }
            else if (systemState == ClimberState.home) {
                sequence = HabClimbSequences.homeSequence();
            }

            if (sequence != null) {
                this.clawDelegate.updateSequence(sequence.a);
                this.rackDelegate.updateSequence(sequence.b);
                this.clawState = MotionProfileState.RUNNING;
                this.legState = MotionProfileState.RUNNING;

                sequence.a.callback = () -> this.clawState = MotionProfileState.FINISHED;
                sequence.b.callback = () -> this.legState = MotionProfileState.FINISHED;
            }
        }
        if (this.clawState == MotionProfileState.FINISHED && this.legState == MotionProfileState.FINISHED) {
            if (this.systemState == ClimberState.home) this.systemState = ClimberState.safelyStowed;
        }
    }
}

class HabClimbSequences {
    private static Climber climber = Climber.getInstance();
    public static Pair<Sequence<MotionTriplet>> hab2ClimbSequence() {
        double initClaw = climber.getClaw().getDegrees() / 180.0 * Math.PI;
        double initRack = climber.getLeg().getLinearPosition();
        double midClaw = PhysicsWorld.getInstance().solveClimberClawAngleForHeight(0, 5.75);
        double midRack = 0;

        double endClaw = 90 / 180 * Math.PI;
        double endRack = 5.75 + midRack;

        Pair<MotionProfiler> step1 = CoupledLiftProfiler.generateProfiles(initClaw, midClaw, initRack, midRack);
        Pair<MotionProfiler> step2 = CoupledLiftProfiler.getHabClimbProfiles(midClaw, endClaw, midRack, endRack);
        Sequence<MotionTriplet> clawSequence = new Sequence<>();
        Sequence<MotionTriplet> rackSequence = new Sequence<>();

        clawSequence.addStep(step1.a);
        clawSequence.addStep(step2.a);

        rackSequence.addStep(step1.b);
        rackSequence.addStep(step2.b);

        return new Pair<>(clawSequence, rackSequence);
    }

    public static Pair<Sequence<MotionTriplet>> hab3ClimbSequence() {
        double initClaw = climber.getClaw().getDegrees() / 180.0 * Math.PI;
        double initRack = climber.getLeg().getLinearPosition();
        double midClaw = PhysicsWorld.getInstance().solveClimberClawAngleForHeight(0, 19);
        double midRack = 0;

        double endClaw = 90 / 180 * Math.PI;
        double endRack = 19 + midRack;

        Pair<MotionProfiler> step1 = CoupledLiftProfiler.generateProfiles(initClaw, midClaw, initRack, midRack);
        Pair<MotionProfiler> step2 = CoupledLiftProfiler.getHabClimbProfiles(midClaw, endClaw, midRack, endRack);
        Sequence<MotionTriplet> clawSequence = new Sequence<>();
        Sequence<MotionTriplet> rackSequence = new Sequence<>();

        clawSequence.addStep(step1.a);
        clawSequence.addStep(step2.a);

        rackSequence.addStep(step1.b);
        rackSequence.addStep(step2.b);

        return new Pair<>(clawSequence, rackSequence);
    }

    public static Pair<Sequence<MotionTriplet>> homeSequence() {
        double initClaw = climber.getClaw().getDegrees() / 180.0 * Math.PI;
        double initRack = climber.getLeg().getLinearPosition();
        double endClaw = 180 / 180 * Math.PI;
        double endRack = climber.getLeg().getHomeLinearPosition();

        Pair<MotionProfiler> step1 = CoupledLiftProfiler.generateProfiles(initClaw, endClaw, initRack, endRack);
        
        Sequence<MotionTriplet> clawSequence = new Sequence<>();
        Sequence<MotionTriplet> rackSequence = new Sequence<>();
        clawSequence.addStep(step1.a);
        rackSequence.addStep(step1.b);

        return new Pair<>(clawSequence, rackSequence);
    }
}