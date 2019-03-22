package frc.robot.util.loops;


public interface Loop {
    default public void onStart() {};
    public void onUpdate();
    default public void onStop() {};
}