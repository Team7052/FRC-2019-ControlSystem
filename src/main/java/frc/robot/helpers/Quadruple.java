package frc.robot.helpers;

public class Quadruple<T> {
    public T a;
    public T b; 
    public T c;
    public T d;

    public Quadruple(T a, T b, T c, T d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }
    @Override
    public String toString() {
        return "(" + a + ", " + b + ", " + c + ", " + d + ")";
    }
}