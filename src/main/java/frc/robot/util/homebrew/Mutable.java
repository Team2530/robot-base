package frc.robot.util.homebrew;

import java.util.function.Supplier;

public class Mutable<T> {
	private T value;

	public Mutable(T value) {
        this.value = value;
	}

    public void set(T value) {
        this.value = value;
    }

	public T get() {
		return value;
	}

	public Supplier<T> supplier() {
		return 	(() -> value);
    }
}
