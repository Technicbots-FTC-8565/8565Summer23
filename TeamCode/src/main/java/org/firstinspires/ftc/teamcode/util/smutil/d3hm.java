package org.firstinspires.ftc.teamcode.util.smutil;

import java.util.ArrayList;
import java.util.HashMap;

public class d3hm<T, K> extends HashMap<T, ArrayList<K>> {
	public d3hm() {
		super();
	}

	public d3hm(int a, float b) {
		super(a, b);
	}

	public void append(T key, K append) {
		if (super.containsKey(key)) super.get(key).add(append);
	}

	public void appendToAll(K append) {
		for (T ti : super.keySet()) {
			this.append(ti, append);
		}
	}
}