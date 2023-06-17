package org.firstinspires.ftc.teamcode.util.smutil;

public class d5hm<T, K, V, M> extends d4hm<T, K, Pair<V, M>> {
	public d5hm() {
		super();
	}

	public d5hm(int a, float b) {
		super(a, b);
	}

	@Override
	public void append(T key, Pair<K, Pair<V, M>> append) {
		super.append(key, append);
	}

	public void append(T t, K k, V v, M m) {
		super.append(t, k, new Pair<>(v, m));
	}

	public void append(T t, K k, Pair<V, M> vmPair) {
		super.append(t, k, vmPair);
	}

	@Override
	public void appendToAll(Pair<K, Pair<V, M>> append) {
		super.appendToAll(append);
	}

	public void appendToAll(K k, V v, M m) {
		super.appendToAll(k, new Pair<>(v, m));
	}

	public void appendToAll(K k, Pair<V, M> vmPair) {
		super.appendToAll(k, vmPair);
	}

	public void appendTo(T key, K[] keys, Pair<V, M> vmPair) {
		for (K k : keys) {
			super.append(key, k, vmPair);
		}
	}

	public void appendTo(T key, K[] keys, V v, M m) {
		for (K k : keys) {
			super.append(key, k, new Pair<>(v, m));
		}
	}

	public void appendTo(K[] keys, Pair<V, M> vmPair) {
		for (T t : super.keySet()) {
			this.appendTo(t, keys, vmPair);
		}
	}

	public void appendTo(K[] keys, V v, M m) {
		for (T t : super.keySet()) {
			this.appendTo(t, keys, v, m);
		}
	}
}