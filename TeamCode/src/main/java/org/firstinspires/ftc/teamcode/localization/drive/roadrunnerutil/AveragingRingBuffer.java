package org.firstinspires.ftc.teamcode.localization.drive.roadrunnerutil;

public class AveragingRingBuffer {
	private int capacity;
	private int size;
	private int index;

	private double total;
	private double[] samples;

	public AveragingRingBuffer(int capacity) {
		this.capacity = capacity;
		size = 0;
		index = 0;
		samples = new double[capacity];
	}

	public void add(double sample) {
		total -= samples[index];
		samples[index] = sample;
		total += sample;

		if (size < capacity) {
			size++;
		}

		index++;
		if (index == size) {
			index = 0;
		}
	}

	public double getAverage() {
		return total / size;
	}
}