package frc.robot.states;

import frc.robot.helpers.Pair;
import frc.robot.motionProfiling.MotionProfileState;
import frc.robot.motionProfiling.MotionTriplet;
import frc.robot.motionProfiling.FilterOutputModifier;
import frc.robot.motionProfiling.TrapezoidShape;
import frc.robot.motionProfiling.TrapezoidalFunctions;
import frc.robot.motionProfiling.FilterStep;
import frc.robot.sequencing.Sequence;
import frc.robot.states.substates.ClimberState;
import frc.robot.subsystems.Climber;
import frc.robot.util.physics.PhysicsWorld;

public class ClimberSuperState extends SuperState<ClimberState> {
    public enum ClimberDeployState {
        notDeployed, shouldDeploy, deploy
    }

    MotionProfileState clawState = MotionProfileState.IDLE;
    MotionProfileState rackState = MotionProfileState.IDLE;

    private ClimberDeployState deployState;

    private ClimberSuperStateDelegate clawDelegate;
    private ClimberSuperStateDelegate rackDelegate;


    public ClimberSuperState() {
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
        if (this.systemState == ClimberState.home || this.systemState == ClimberState.safelyStowed) {
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
                this.rackState = MotionProfileState.RUNNING;

                sequence.a.callback = () -> this.clawState = MotionProfileState.FINISHED;
                sequence.b.callback = () -> this.rackState = MotionProfileState.FINISHED;
            }
        }
        if (this.clawState == MotionProfileState.FINISHED && this.rackState == MotionProfileState.FINISHED) {
            if (this.systemState == ClimberState.home) this.systemState = ClimberState.safelyStowed;
        }
    }
}

class HabClimbSequences {
    public static double clawMaxVelocity = Math.PI / 3;
    public static double clawMaxAcceleration = Math.PI / 2;
    public static double rackMaxVelocity = 2.5; // inches / s
    public static double rackMaxAcceleration = 1.5; // inches / s^2

    private static Climber climber = Climber.getInstance();

    public static Pair<Sequence<MotionTriplet>> hab2ClimbSequence() {
        return habClimbSequence(6.125);
    }
    public static Pair<Sequence<MotionTriplet>> hab3ClimbSequence() {
        return habClimbSequence(19.125);
    }
    public static Pair<Sequence<MotionTriplet>> habClimbSequence(double habHeight) {
        double initClaw = climber.getClaw().getDegrees() / 180.0 * Math.PI;
        double initRack = climber.getRack().getLinearPosition();
        double midClaw = PhysicsWorld.getInstance().solveClimberClawAngleForHeight(0, habHeight);
        double midRack = 0;

        double endClaw = 90.0 / 180.0 * Math.PI;
        double endRack = habHeight + midRack;

        System.out.println("Climb: " + habHeight);

        TrapezoidShape clawShape = TrapezoidalFunctions.generateTrapezoidShape(initClaw, midClaw, clawMaxVelocity, clawMaxAcceleration);
        TrapezoidShape rackShape = TrapezoidalFunctions.generateTrapezoidShape(initRack, midRack, rackMaxVelocity, rackMaxAcceleration);
        FilterStep<MotionTriplet> clawProfileStep1 = FilterStep.trapezoidalProfileFilter(clawShape, initClaw);
        FilterStep<MotionTriplet> rackProfileStep1 = FilterStep.trapezoidalProfileFilter(rackShape, initRack);

        Pair<FilterStep<MotionTriplet>> step2 = syncHabClimbProfiles(midClaw, endClaw, midRack, endRack);
        Sequence<MotionTriplet> clawSequence = new Sequence<>();
        Sequence<MotionTriplet> rackSequence = new Sequence<>();


        clawSequence.addStep(clawProfileStep1);
        clawSequence.addStep(step2.a);

        rackSequence.addStep(rackProfileStep1);
        rackSequence.addStep(step2.b);

        return new Pair<>(clawSequence, rackSequence);
    }

    public static Pair<Sequence<MotionTriplet>> homeSequence() {
        double initClaw = climber.getClaw().getDegrees() / 180.0 * Math.PI;
        double initRack = climber.getRack().getLinearPosition();
        double endClaw = 200.0 / 180.0 * Math.PI;
        double endRack = climber.getRack().getHomeLinearPosition();
        TrapezoidShape clawShape = TrapezoidalFunctions.generateTrapezoidShape(initClaw, endClaw, clawMaxVelocity, clawMaxAcceleration);
        TrapezoidShape rackShape = TrapezoidalFunctions.generateTrapezoidShape(initRack, endRack, rackMaxVelocity, rackMaxAcceleration);

        FilterStep<MotionTriplet> clawProfile = FilterStep.trapezoidalProfileFilter(clawShape, initClaw);
        FilterStep<MotionTriplet> rackProfile = FilterStep.trapezoidalProfileFilter(rackShape, initRack);

        Sequence<MotionTriplet> clawSequence = new Sequence<>();
        Sequence<MotionTriplet> rackSequence = new Sequence<>();
        clawSequence.addStep(clawProfile);
        rackSequence.addStep(rackProfile);

        return new Pair<>(clawSequence, rackSequence);
    }

    /* assumes that rack is touching ground and claw is touching the hab */
    private static Pair<FilterStep<MotionTriplet>> syncHabClimbProfiles(double initClaw, double endClaw, double initRack, double endRack) {
        // generate profiles
        TrapezoidShape clawShape = TrapezoidalFunctions.generateTrapezoidShape(initClaw, endClaw, clawMaxVelocity, clawMaxAcceleration);
        TrapezoidShape rackShape = TrapezoidalFunctions.generateTrapezoidShape(initRack, endRack, rackMaxVelocity, rackMaxAcceleration);

        FilterStep<MotionTriplet> clawProfile, rackProfile;
        double give = 10.0 / 180.0 * Math.PI;

        if (clawShape.totalTime() <= rackShape.totalTime()) {
            System.out.println("Solve with fixed rack heights");
            
            FilterOutputModifier<MotionTriplet> filter = (dt, endTime, triplet) -> {
                System.out.println(rackShape.getIntegralForTime(dt)+ initRack + " " + dt + " " + endTime + " " + rackShape.totalTime());
                double angle = PhysicsWorld.getInstance().solveClimberClawAngleForHeight(rackShape.getIntegralForTime(dt), endRack) - (dt / endTime) * give;
                return new MotionTriplet(angle, 0.0, 0.0);
            };

            rackProfile = FilterStep.trapezoidalProfileFilter(rackShape, initRack);
            clawProfile = new FilterStep<MotionTriplet>(filter, () -> rackShape.totalTime());
        }
        else {
            FilterOutputModifier<MotionTriplet> filter = (dt, endTime, triplet) -> {
                double height = PhysicsWorld.getInstance().solveClimberRackHeightForAngle(clawShape.getIntegralForTime(dt), endRack) - (dt / endTime) * give;
                return new MotionTriplet(height, 0.0, 0.0);
            };
            // solve based on claw
            System.out.println("Solve with fixed claw angles");
            clawProfile = FilterStep.trapezoidalProfileFilter(clawShape, initClaw);
            rackProfile = new FilterStep<MotionTriplet>(filter, () -> clawShape.totalTime());
        }

        return new Pair<>(clawProfile, rackProfile);
    }
}