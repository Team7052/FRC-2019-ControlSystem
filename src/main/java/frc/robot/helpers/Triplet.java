package frc.robot.helpers;

public class Triplet<T> {
    public T a;
    public T b; 
    public T c;

    public Triplet(T a, T b, T c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }
    @Override
    public String toString() {
        return "(" + a + ", " + b + ", " + c + ")";
    }
}