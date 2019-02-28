package frc.robot.helpers;

public class Pair<T> {
    public T a;
    public T b;
    public Pair(T a, T b) {
        this.a = a;
        this.b = b;
    }

    @Override
    public String toString() {
        return "{ " + this.a + ", " + this.b + "}";
    }
}
