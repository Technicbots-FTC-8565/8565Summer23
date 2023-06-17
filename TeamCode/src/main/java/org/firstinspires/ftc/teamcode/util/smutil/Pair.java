package org.firstinspires.ftc.teamcode.util.smutil;

//Android pair has final members, this one does not
public class Pair<U, V> {
	public U first;       // the first field of a pair
	public V second;      // the second field of a pair

	// Constructs a new pair with specified values
	public Pair(U first, V second) {
		this.first = first;
		this.second = second;
	}

	@Override
	// Computes hash code for an object to support hash tables
	public int hashCode() {
		// use hash codes of the underlying objects
		return 31 * first.hashCode() + second.hashCode();
	}

	@Override
	public String toString() {
		return "(" + first + ", " + second + ")";
	}

	// Factory method for creating a typed Pair immutable instance
	public static <U, V> Pair<U, V> of(U a, V b) {
		// calls private constructor
		return new Pair<>(a, b);
	}
}