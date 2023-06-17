package org.firstinspires.ftc.teamcode.util.smutil;

//TODO: Clean up super references and stuff
public class d4hm<T, K, V> extends d3hm<T, Pair<K, V>> {
	public d4hm(int a, float b) {
		super(a, b);
	}

	public d4hm() {
		super();
	}

	@Override
	public void append(T key, Pair<K, V> append) {
		super.append(key, append);
	}

	public void append(T key, K init, V append) {
		super.append(key, new Pair<>(init, append));
	}

	@Override
	public void appendToAll(Pair<K, V> append) {
		super.appendToAll(append);
	}

	public void appendToAll(K init, V append) {
		this.appendToAll(new Pair<>(init, append));
	}
}