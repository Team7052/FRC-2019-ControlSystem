package frc.robot.helpers;

/**
 * Create a pair of 2 objects of any data type
 * Immutable, meaning you can't change any values once initialized
 * @param <T> Generic type to represent a pair of any object
 */
public class Pair<T> {
    /**
     * First item in the pair
     */
    private T a;
    /**
     * Second item in the pair
     */
    private T b;

    /**
     * 
     * @param a First item in pair
     * @param b Second item in pair
     */
    public Pair(T a, T b) {
        this.a = a;
        this.b = b;
    }

    public T getFirst() {
        return this.a;
    }
    public T getSecond() {
        return this.b;
    }

    @Override
    public String toString() {
        return "{ " + this.a + ", " + this.b + "}";
    }
}
