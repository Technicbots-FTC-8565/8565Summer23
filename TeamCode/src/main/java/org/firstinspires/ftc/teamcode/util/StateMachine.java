package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.smutil.Pair;
import org.firstinspires.ftc.teamcode.util.smutil.d5hm;
import org.firstinspires.ftc.teamcode.util.smutil.hmutil;

import java.util.Objects;
import java.util.function.Supplier;

/*
TODO: Add ability to add condition and more specific action execution with one command
 */

public class StateMachine<T extends Enum<T>> {
	public T state;
	public T prevstate;
	/**
	 * This state is modified whenever a source state is specified or a condition is added to the state machine.
	 * It is used to build states without having to specify source states, required previous states, or destination states.
	 * When the constructor is called, this state is set to either the specified initial state of the state machine
	 * or the first state in the enum.
	 */
	public T buildState;
	public Class<T> enom;
	public T[] values;
	// Internal timer for incorporating delays
	public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	// <execute on this state, only execute if prevstate = this state, execute this
	// many times (if less than 1, infinite), run this>
	public final d5hm<T, T, Integer, Runnable> beforeActionMap = new d5hm<>();
	public final d5hm<T, T, Integer, Runnable> afterActionMap = new d5hm<>();
	public final d5hm<T, T, Integer, Runnable> transitionActionMap = new d5hm<>();
	public final d5hm<T, T, Integer, Supplier<Boolean>> conditionMap = new d5hm<>();

	public StateMachine(T initState, Class<T> enom) {
		this.enom = enom;
		values = this.enom.getEnumConstants();
		this.state = initState;
		this.prevstate = this.getPrevChron(state);
		this.buildState = state;
		this.resetMaps();
	}

	public StateMachine(Class<T> enom) {
		this.enom = enom;
		values = this.enom.getEnumConstants();
		this.state = values[0];
		this.prevstate = this.getPrevChron(state);
		this.buildState = state;
		this.resetMaps();
	}

	/**
	 * Adds action to be executed when the {@link #buildState build state} is reached an infinite amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAction(Runnable runnable) {
		transitionActionMap.appendTo(buildState, values, -1, runnable);
		return this;
	}

	/**
	 * Adds action to be executed when the {@link #buildState build state} is reached a specified amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAction(int repetitions, Runnable runnable) {
		transitionActionMap.appendTo(buildState, values, repetitions, runnable);
		return this;
	}

	/**
	 * Adds action to be executed when a specified state is reached an infinite amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param state State to execute action at
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAction(T state, Runnable runnable) {
		transitionActionMap.appendTo(state, values, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed when a specified state is reached a specified amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param state State to execute action at
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAction(T state, int repetitions, Runnable runnable) {
		transitionActionMap.appendTo(state, values, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed when a specified state is reached an infinite amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param state State to execute action at
	 * @param reqPrev State to be accepted as previous state
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAction(T state, T reqPrev, Runnable runnable) {
		transitionActionMap.append(state, reqPrev, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed when a specified state is reached a specified amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param state State to execute action at
	 * @param reqPrev State to be accepted as previous state
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAction(T state, T reqPrev, int repetitions, Runnable runnable) {
		transitionActionMap.append(state, reqPrev, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed when any state is reached an infinite amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addActionToAll(Runnable runnable) {
		transitionActionMap.appendTo(values, -1, runnable);
		return this;
	}

	/**
	 * Adds action to be executed when any state is reached a specified amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addActionToAll(int repetitions, Runnable runnable) {
		transitionActionMap.appendTo(values, repetitions, runnable);
		return this;
	}

	/**
	 * Adds action to be executed when any state is reached an infinite amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param reqPrev State to be accepted as previous state
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addActionToAll(T reqPrev, Runnable runnable) {
		transitionActionMap.appendToAll(reqPrev, -1, runnable);
		return this;
	}

	/**
	 * Adds action to be executed when any state is reached a specified amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param reqPrev State to be accepted as previous state
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addActionToAll(T reqPrev, int repetitions, Runnable runnable) {
		transitionActionMap.appendToAll(reqPrev, repetitions, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the {@link #buildState build state}.
	 * before any conditions are tested, an infinite amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeAction(Runnable runnable) {
		beforeActionMap.appendTo(buildState, values, -1, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the {@link #buildState build state}.
	 * before any conditions are tested, a specified amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeAction(int repetitions, Runnable runnable) {
		beforeActionMap.appendTo(buildState, values, repetitions, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the specified state,
	 * before any conditions are tested, an infinite amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param state State to execute action at
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeAction(T state, Runnable runnable) {
		beforeActionMap.appendTo(state, values, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the specified state,
	 * before any conditions are tested, a specified amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param state State to execute action at
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeAction(T state, int repetitions, Runnable runnable) {
		beforeActionMap.appendTo(state, values, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the specified state,
	 * before any conditions are tested, an infinite amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param state State to execute action at
	 * @param reqPrev State to be accepted as previous state
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeAction(T state, T reqPrev, Runnable runnable) {
		beforeActionMap.append(state, reqPrev, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the specified state,
	 * before any conditions are tested, a specified amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param state State to execute action at
	 * @param reqPrev State to be accepted as previous state
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeAction(T state, T reqPrev, int repetitions, Runnable runnable) {
		beforeActionMap.append(state, reqPrev, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is any state,
	 * before any conditions are tested, an infinite amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeActionToAll(Runnable runnable) {
		beforeActionMap.appendTo(values, -1, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is any state,
	 * before any conditions are tested, a specified amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeActionToAll(int repetitions, Runnable runnable) {
		beforeActionMap.appendTo(values, repetitions, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is any state,
	 * before any conditions are tested, an infinite amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param reqPrev State to be accepted as previous state
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeActionToAll(T reqPrev, Runnable runnable) {
		beforeActionMap.appendToAll(reqPrev, -1, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is any state,
	 * before any conditions are tested, a specified amount of times,
	 * and executed with a specified previous state as an acceptable previous state.
	 * @param reqPrev State to be accepted as previous state
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addBeforeActionToAll(T reqPrev, int repetitions, Runnable runnable) {
		beforeActionMap.appendToAll(reqPrev, repetitions, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the {@link #buildState build state},
	 * after any conditions are tested, an infinite amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterAction(Runnable runnable) {
		afterActionMap.appendTo(buildState, values, -1, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the {@link #buildState build state},
	 * after all conditions are tested, a specified amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterAction(int repetitions, Runnable runnable) {
		afterActionMap.appendTo(buildState, values, repetitions, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the specified state,
	 * after all conditions are tested, an infinite amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterAction(T state, Runnable runnable) {
		afterActionMap.appendTo(state, values, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the specified state,
	 * after all conditions are tested, a specified amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param state State to execute action at
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterAction(T state, int repetitions, Runnable runnable) {
		afterActionMap.appendTo(state, values, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the specified state,
	 * after all conditions are tested, an infinite amount of times,
	 * and executed with the specified previous state an acceptable previous state.
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterAction(T state, T reqPrev, Runnable runnable) {
		afterActionMap.append(state, reqPrev, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is the specified state,
	 * after all conditions are tested, a specified amount of times,
	 * and executed with the specified previous state an acceptable previous state.
	 * @param state State to execute action at
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterAction(T state, T reqPrev, int repetitions, Runnable runnable) {
		afterActionMap.append(state, reqPrev, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is any state,
	 * after all conditions are tested, an infinite amount of times,
	 * and executed with all previous states as acceptable previous states.
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterActionToAll(Runnable runnable) {
		afterActionMap.appendTo(values, -1, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is any state,
	 * after all conditions are tested, a specified amount of times,
	 * and executed with all previous state as an acceptable previous state.
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterActionToAll(int repetitions, Runnable runnable) {
		afterActionMap.appendTo(values, repetitions, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is any state,
	 * after all conditions are tested, an infinite amount of times,
	 * and executed with the specified previous state an acceptable previous state.
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterActionToAll(T reqPrev, Runnable runnable) {
		afterActionMap.appendToAll(reqPrev, -1, runnable);
		return this;
	}

	/**
	 * Adds action to be executed every time {@link #update()} is called
	 * when the current state is any state,
	 * after all conditions are tested, a specified amount of times,
	 * and executed with the specified previous state an acceptable previous state.
	 * @param reqPrev State to be accepted as previous state
	 * @param repetitions Number of times to repeat action
	 * @param runnable Action to be executed
	 */
	public StateMachine<T> addAfterActionToAll(T reqPrev, int repetitions, Runnable runnable) {
		afterActionMap.appendToAll(reqPrev, repetitions, runnable);
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is the {@link #buildState build state},
	 * with the destination state set as the state chronologically after {@link #buildState build state},
	 * an infinite amount of times.
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addCondition(Supplier<Boolean> supplier) {
		conditionMap.append(buildState, this.getNextChron(buildState), -1, supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is the {@link #buildState build state},
	 * with the destination state set as the state chronologically after {@link #buildState build state},
	 * a specified amount of times.
	 * @param repetitions Number of times to test condition
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addCondition(int repetitions, Supplier<Boolean> supplier) {
		conditionMap.append(buildState, this.getNextChron(buildState), repetitions, supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is the {@link #buildState build state},
	 * with the destination state set as the specified state,
	 * an infinite amount of times.
	 * @param dst State to be set as destination state
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addCondition(T dst, Supplier<Boolean> supplier) {
		conditionMap.append(buildState, dst, -1, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is the {@link #buildState build state},
	 * with the destination state set as the specified state,
	 * a specified amount of times.
	 * @param dst State to be set as destination state
	 * @param repetitions Number of times to test condition
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addCondition(T dst, int repetitions, Supplier<Boolean> supplier) {
		conditionMap.append(buildState, dst, repetitions, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is the specified state,
	 * with the destination state set as the specified state,
	 * an infinite amount of times.
	 * @param src State to execute condition at
	 * @param dst State to be set as destination state
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addCondition(T src, T dst, Supplier<Boolean> supplier) {
		conditionMap.append(src, dst, -1, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is the specified state,
	 * with the destination state set as the specified state,
	 * a specified amount of times.
	 * @param src State to execute condition at
	 * @param dst State to be set as destination state
	 * @param repetitions Number of times to test condition
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addCondition(T src, T dst, int repetitions, Supplier<Boolean> supplier) {
		conditionMap.append(src, dst, repetitions, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is any state,
	 * with the destination state set as the {@link #buildState build state},
	 * an infinite amount of times.
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addConditionToAll(Supplier<Boolean> supplier) {
		conditionMap.appendToAll(buildState, -1, supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is any state,
	 * with the destination state set as the {@link #buildState build state},
	 * a specified amount of times.
	 * @param repetitions Number of times to test condition
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addConditionToAll(int repetitions, Supplier<Boolean> supplier) {
		conditionMap.appendToAll(buildState, repetitions, supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is any state,
	 * with the destination state set as the specified state,
	 * an infinite amount of times.
	 * @param dst State to be set as destination state
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addConditionToAll(T dst, Supplier<Boolean> supplier) {
		conditionMap.appendToAll(dst, -1, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Adds condition to be tested every time {@link #update()} is called
	 * when the current state is any state,
	 * with the destination state set as the specified state,
	 * a specified amount of times.
	 * @param dst State to be set as destination state
	 * @param repetitions Number of times to test condition
	 * @param supplier Condition to be tested
	 */
	public StateMachine<T> addConditionToAll(T dst, int repetitions, Supplier<Boolean> supplier) {
		conditionMap.appendToAll(dst, repetitions, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addCondition(Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseCondition(Supplier<Boolean> supplier) {
		hmutil.appendReverseCondition(conditionMap, buildState, this.getNextChron(buildState), -1,
				supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Basically {@link #addCondition(int, Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseCondition(int repetitions, Supplier<Boolean> supplier) {
		hmutil.appendReverseCondition(conditionMap, buildState, this.getNextChron(buildState),
				repetitions, supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Basically {@link #addCondition(T, Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseCondition(T dst, Supplier<Boolean> supplier) {
		hmutil.appendReverseCondition(conditionMap, buildState, dst, -1, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addCondition(T, int, Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseCondition(T dst, int repetitions, Supplier<Boolean> supplier) {
		hmutil.appendReverseCondition(conditionMap, buildState, dst, repetitions, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addCondition(T, T, Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseCondition(T src, T dst, Supplier<Boolean> supplier) {
		hmutil.appendReverseCondition(conditionMap, src, dst, -1, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addCondition(T, T, int, Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseCondition(T src, T dst, int repetitions, Supplier<Boolean> supplier) {
		hmutil.appendReverseCondition(conditionMap, src, dst, repetitions, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addConditionToAll(Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseConditionToAll(Supplier<Boolean> supplier) {
		hmutil.appendReverseConditionToAll(conditionMap, buildState, -1, supplier);
		return this;
	}

	/**
	 * Basically {@link #addConditionToAll(int, Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseConditionToAll(int repetitions, Supplier<Boolean> supplier) {
		hmutil.appendReverseConditionToAll(conditionMap, buildState, repetitions,
				supplier);
		return this;
	}

	/**
	 * Basically {@link #addConditionToAll(T, Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseConditionToAll(T dst, Supplier<Boolean> supplier) {
		hmutil.appendReverseConditionToAll(conditionMap, dst, -1, supplier);
		return this;
	}

	/**
	 * Basically {@link #addConditionToAll(T, int, Supplier)}, but with a supplier that returns false when true
	 * and true when false. <br> <br>
	 * Useful, for example, if you wanted to cycle states when Roadrunner has stopped;
	 * chassis.isBusy() returns false when you want the state machine to change state, but it
	 * must return true in order for the state machine to actually change state.
	 */
	public StateMachine<T> addReverseConditionToAll(T dst, int repetitions, Supplier<Boolean> supplier) {
		hmutil.appendReverseConditionToAll(conditionMap, dst, repetitions, supplier);
		return this;
	}

	/**
	 * Basically {@link #addAction(Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAction(Runnable... runnables) {
		for (Runnable runnable : runnables)
			transitionActionMap.appendTo(buildState, values, -1, runnable);
		return this;
	}

	/**
	 * Basically {@link #addAction(int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAction(int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			transitionActionMap.appendTo(buildState, values, repetitions, runnable);
		return this;
	}

	/**
	 * Basically {@link #addAction(T, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAction(T state, Runnable... runnables) {
		for (Runnable runnable : runnables)
			transitionActionMap.appendTo(state, values, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addAction(T, int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAction(T state, int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			transitionActionMap.appendTo(state, values, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addAction(T, T, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAction(T state, T reqPrev, Runnable... runnables) {
		for (Runnable runnable : runnables)
			transitionActionMap.append(state, reqPrev, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addAction(T, T, int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAction(T state, T reqPrev, int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			transitionActionMap.append(state, reqPrev, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addActionToAll(Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addActionToAll(Runnable... runnables) {
		for (Runnable runnable : runnables) transitionActionMap.appendTo(values, -1, runnable);
		return this;
	}

	/**
	 * Basically {@link #addActionToAll(int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addActionToAll(int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			transitionActionMap.appendTo(values, repetitions, runnable);
		return this;
	}

	/**
	 * Basically {@link #addActionToAll(T, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addActionToAll(T reqPrev, Runnable... runnables) {
		for (Runnable runnable : runnables) transitionActionMap.appendToAll(reqPrev, -1, runnable);
		return this;
	}

	/**
	 * Basically {@link #addActionToAll(T, int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addActionToAll(T reqPrev, int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			transitionActionMap.appendToAll(reqPrev, repetitions, runnable);
		return this;
	}

	/**
	 * Basically {@link #addBeforeAction(Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeAction(Runnable... runnables) {
		for (Runnable runnable : runnables)
			beforeActionMap.appendTo(buildState, values, -1, runnable);
		return this;
	}

	/**
	 * Basically {@link #addBeforeAction(int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeAction(int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			beforeActionMap.appendTo(buildState, values, repetitions, runnable);
		return this;
	}

	/**
	 * Basically {@link #addBeforeAction(T, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeAction(T state, Runnable... runnables) {
		for (Runnable runnable : runnables) beforeActionMap.appendTo(state, values, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addBeforeAction(T, int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeAction(T state, int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			beforeActionMap.appendTo(state, values, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addBeforeAction(T, T, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeAction(T state, T reqPrev, Runnable... runnables) {
		for (Runnable runnable : runnables) beforeActionMap.append(state, reqPrev, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addBeforeAction(T, T, int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeAction(T state, T reqPrev, int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			beforeActionMap.append(state, reqPrev, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addBeforeActionToAll(Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeActionToAll(Runnable... runnables) {
		for (Runnable runnable : runnables) beforeActionMap.appendTo(values, -1, runnable);
		return this;
	}

	/**
	 * Basically {@link #addBeforeActionToAll(int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeActionToAll(int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables) beforeActionMap.appendTo(values, repetitions, runnable);
		return this;
	}

	/**
	 * Basically {@link #addBeforeActionToAll(T, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeActionToAll(T reqPrev, Runnable... runnables) {
		for (Runnable runnable : runnables) beforeActionMap.appendToAll(reqPrev, -1, runnable);
		return this;
	}

	/**
	 * Basically {@link #addBeforeActionToAll(T, int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addBeforeActionToAll(T reqPrev, int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			beforeActionMap.appendToAll(reqPrev, repetitions, runnable);
		return this;
	}

	/**
	 * Basically {@link #addAfterAction(Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterAction(Runnable... runnables) {
		for (Runnable runnable : runnables)
			afterActionMap.appendTo(buildState, values, -1, runnable);
		return this;
	}

	/**
	 * Basically {@link #addAfterAction(int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterAction(int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			afterActionMap.appendTo(buildState, values, repetitions, runnable);
		return this;
	}

	/**
	 * Basically {@link #addAfterAction(T, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterAction(T state, Runnable... runnables) {
		for (Runnable runnable : runnables) afterActionMap.appendTo(state, values, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addAfterAction(T, int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterAction(T state, int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			afterActionMap.appendTo(state, values, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addAfterAction(T, T, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterAction(T state, T reqPrev, Runnable... runnables) {
		for (Runnable runnable : runnables) afterActionMap.append(state, reqPrev, -1, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addAfterAction(T, T, int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterAction(T state, T reqPrev, int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			afterActionMap.append(state, reqPrev, repetitions, runnable);
		buildState = state;
		return this;
	}

	/**
	 * Basically {@link #addAfterActionToAll(Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterActionToAll(Runnable... runnables) {
		for (Runnable runnable : runnables) afterActionMap.appendTo(values, -1, runnable);
		return this;
	}

	/**
	 * Basically {@link #addAfterActionToAll(int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterActionToAll(int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables) afterActionMap.appendTo(values, repetitions, runnable);
		return this;
	}

	/**
	 * Basically {@link #addAfterActionToAll(T, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterActionToAll(T reqPrev, Runnable... runnables) {
		for (Runnable runnable : runnables) afterActionMap.appendToAll(reqPrev, -1, runnable);
		return this;
	}

	/**
	 * Basically {@link #addAfterActionToAll(T, int, Runnable)}, but you can append any amount of actions inline.
	 */
	public StateMachine<T> addAfterActionToAll(T reqPrev, int repetitions, Runnable... runnables) {
		for (Runnable runnable : runnables)
			afterActionMap.appendToAll(reqPrev, repetitions, runnable);
		return this;
	}

	/**
	 * Basically {@link #addCondition(Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addCondition(Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			conditionMap.append(buildState, this.getNextChron(buildState), -1, supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Basically {@link #addCondition(int, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addCondition(int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			conditionMap.append(buildState, this.getNextChron(buildState), repetitions, supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Basically {@link #addCondition(T, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addCondition(T dst, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			conditionMap.append(buildState, dst, -1, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addCondition(T, int, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addCondition(T dst, int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			conditionMap.append(buildState, dst, repetitions, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addCondition(T, T, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addCondition(T src, T dst, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers) conditionMap.append(src, dst, -1, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addCondition(T, T, int, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addCondition(T src, T dst, int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			conditionMap.append(src, dst, repetitions, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addConditionToAll(Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addConditionToAll(Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			conditionMap.appendToAll(buildState, -1, supplier);
		return this;
	}

	/**
	 * Basically {@link #addConditionToAll(int, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addConditionToAll(int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			conditionMap.appendToAll(buildState, repetitions, supplier);
		return this;
	}

	/**
	 * Basically {@link #addConditionToAll(T, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addConditionToAll(T dst, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers) conditionMap.appendToAll(dst, -1, supplier);
		return this;
	}

	/**
	 * Basically {@link #addConditionToAll(T, int, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addConditionToAll(T dst, int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			conditionMap.appendToAll(dst, repetitions, supplier);
		return this;
	}

	/**
	 * Basically {@link #addReverseCondition(Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseCondition(Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseCondition(conditionMap, buildState, this.getNextChron(buildState), -1,
					supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Basically {@link #addReverseCondition(int, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseCondition(int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseCondition(conditionMap, buildState, this.getNextChron(buildState),
					repetitions, supplier);
		buildState = this.getNextChron(buildState);
		return this;
	}

	/**
	 * Basically {@link #addReverseCondition(T, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseCondition(T dst, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseCondition(conditionMap, buildState, dst, -1, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addReverseCondition(T, int, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseCondition(T dst, int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseCondition(conditionMap, buildState, dst, repetitions, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addReverseCondition(T, T, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseCondition(T src, T dst, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseCondition(conditionMap, src, dst, -1, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addReverseCondition(T, T, int, Supplier)}, but you can append any amount of conditions
	 * inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseCondition(T src, T dst, int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseCondition(conditionMap, src, dst, repetitions, supplier);
		buildState = dst;
		return this;
	}

	/**
	 * Basically {@link #addReverseConditionToAll(Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseConditionToAll(Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseConditionToAll(conditionMap, buildState, -1, supplier);
		return this;
	}

	/**
	 * Basically {@link #addReverseConditionToAll(int, Supplier)}, but you can append any amount of conditions
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseConditionToAll(int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseConditionToAll(conditionMap, buildState, repetitions,
					supplier);
		return this;
	}

	/**
	 * Basically {@link #addReverseConditionToAll(T, Supplier)}, but you can append any amount of conditions inline.
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseConditionToAll(T dst, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseConditionToAll(conditionMap, dst, -1, supplier);
		return this;
	}

	/**
	 * Basically {@link #addReverseConditionToAll(T, int, Supplier)}, but you can append any amount of conditions
	 */
	@SafeVarargs
	public final StateMachine<T> addReverseConditionToAll(T dst, int repetitions, Supplier<Boolean>... suppliers) {
		for (Supplier<Boolean> supplier : suppliers)
			hmutil.appendReverseConditionToAll(conditionMap, dst, repetitions, supplier);
		return this;
	}

	/**
	 * @param state Input state
	 * @return The state chronologically before the input state
	 */
	public T getPrevChron(T state) {
		if (state.ordinal() > 0)
			return values[state.ordinal() - 1];
		return values[values.length - 1];
	}

	/**
	 * @param state Input state
	 * @return The state chronologically after the input state
	 */
	public T getNextChron(T state) {
		if (state.ordinal() < values.length - 1)
			return values[state.ordinal() + 1];
		return values[0];
	}

	/**
	 * Resets all the maps
	 */
	public void resetMaps() {
		hmutil.resetMap(beforeActionMap, values);
		hmutil.resetMap(afterActionMap, values);
		hmutil.resetMap(transitionActionMap, values);
		hmutil.resetConditionMap(conditionMap, values);
		this.addActionToAll(timer::reset); // Resets the timer when the state is reached
	}

	/**
	 * Syntactic sugar
	 */
	public void build() {}

	/**
	 * Changes the state of the state machine to be the input state
	 * @param localize State to localize to
	 */
	public StateMachine<T> localizeBuild(T localize) {
		buildState = localize;
		return this;
	}

	/**
	 * Constructs a function that returns true if the state machine's internal timer reads that
	 * the time since the last state change is greater than the specified time
	 * @param ms Milliseconds to delay by
	 * @return The function
	 */
	public Supplier<Boolean> delay(double ms) { return () -> timer.milliseconds() > ms; }

	public StateMachine<T> addDelay(double ms) { return this.addCondition(this.delay(ms)); }

	/**
	 * Sets this state machine's state to the specified state, allowing it to step to a different point in the SM
	 * @param state State to jump/"localize" to
	 */
	public void localize(T state) {
		this.state = state;
	}

	/**
	 * Updates the state machine
	 */
	public void update() {
		if (state != prevstate && transitionActionMap.containsKey(state)) {
			for (Pair<T, Pair<Integer, Runnable>> pair : Objects.requireNonNull(transitionActionMap.get(state))) {
				if (prevstate == pair.first && pair.second.first != 0) {
					pair.second.second.run();
					if (pair.second.first > 0)
						pair.second.first--;
				}
			}
			prevstate = state;
		}
		Log.d("Internal Timer", String.valueOf(timer.milliseconds()));

		if (beforeActionMap.containsKey(state)) {
			for (Pair<T, Pair<Integer, Runnable>> pair : Objects.requireNonNull(beforeActionMap.get(state))) {
				if (prevstate == pair.first && pair.second.first != 0) {
					pair.second.second.run();
					if (pair.second.first > 0)
						pair.second.first--;
				}
			}
		}
		if (conditionMap.containsKey(state)) {
			for (Pair<T, Pair<Integer, Supplier<Boolean>>> pair : Objects.requireNonNull(conditionMap.get(state))) {
				if (pair.second.second.get() && pair.second.first != 0) {
					state = pair.first;
					if (pair.second.first > 0)
						pair.second.first--;
					break;
				}
			}
		}
		if (afterActionMap.containsKey(state)) {
			for (Pair<T, Pair<Integer, Runnable>> pair : Objects.requireNonNull(afterActionMap.get(state))) {
				if (prevstate == pair.first && pair.second.first != 0) {
					pair.second.second.run();
					if (pair.second.first > 0)
						pair.second.first--;
				}
			}
		}
	}
}