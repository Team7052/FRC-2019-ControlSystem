package frc.robot.helpers;

public class Triplet<T> {
    private T a;
    private T b; 
    private T c;

    public Triplet(T a, T b, T c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public T getFirst() {
        return a;
    }

    public T getSecond() {
        return b;
    }

    public T getThird() {
        return c;
    }

    @Override
    public String toString() {
        return "(" + a + ", " + b + ", " + c + ")";
    }
}