package frc.robot.states;

public abstract class SuperState<T> {
    T systemState;
    public abstract void update();
    public void setState(T state) {
        if (this.systemState != state) this.systemState = state;
    }

    public T getState() {
        return this.systemState;
    }
    
}