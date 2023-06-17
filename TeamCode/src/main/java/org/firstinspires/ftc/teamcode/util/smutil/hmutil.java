package org.firstinspires.ftc.teamcode.util.smutil;

import java.util.ArrayList;
import java.util.function.Supplier;

public class hmutil {
	public static <T extends Enum<T>> void resetConditionMap(d5hm<T, T, Integer, Supplier<Boolean>> map, T[] values) {
		for (T state : values) {
			map.put(state, new ArrayList<>());
			map.append(state, state, -1, () -> false);
		}
	}

	public static <T extends Enum<T>> void resetMap(d5hm<T, T, Integer, Runnable> map, T[] values) {
		for (T state : values) {
			map.put(state, new ArrayList<>());
			map.append(state, state, -1, () -> {
			});
		}
	}

	public static <T extends Enum<T>> void appendReverseCondition(d5hm<T, T, Integer, Supplier<Boolean>> map,
	                                                              T src, T dst, Integer repetitions, Supplier<Boolean> append) {
		map.append(src, dst, repetitions, () -> !append.get());
	}

	public static <T extends Enum<T>> void appendReverseConditionToAll(d5hm<T, T, Integer, Supplier<Boolean>> map,
	                                                                   T dst, Integer repetitions, Supplier<Boolean> append) {
		for (T src : map.keySet())
			appendReverseCondition(map, src, dst, repetitions, append);
	}
}