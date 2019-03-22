package frc.robot.util.loops;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Notifier;

public class Looper implements Runnable {
    private final Notifier notifier;
    public static final double loopFrequency = 100.0;

    private boolean running;

    private ArrayList<Loop> loops;
    public Looper() {
        running = false;
        notifier = new Notifier(this);
        loops = new ArrayList<>();
    }

    public void register(Loop loop) {
        synchronized(this) {
            this.loops.add(loop);
        }
    }

    public void start() {
        if (!running) {
            synchronized(this) {
                running = true;
                for (Loop loop: loops) {
                    loop.onStart();
                }
            }
            notifier.startPeriodic(1.0 / loopFrequency);
        }
    }

    public void stop() {
        if (running) {
            notifier.stop();
            synchronized(this) {
                running = false;
                for (Loop loop: loops) {
                    loop.onStop();
                }
            }
        }
    }

    @Override
    public void run() {
        synchronized(this) {
            if (running) {
                for (Loop loop: loops) {
                    loop.onUpdate();
                }
            }
        }
    }
}