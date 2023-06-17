package org.firstinspires.ftc.teamcode.util

class EventInfo {
    @Volatile private var justOccurred = false
    @Volatile private var justReleased = false
    @Volatile private var prevOccurred = false

    init {
        justOccurred = false;
    }

    fun update(currentState: Boolean) {
        justOccurred = currentState && !prevOccurred
        justReleased = !currentState && prevOccurred
        prevOccurred = currentState
    }

    fun justOccured(): Boolean {
        return justOccurred;
    }

    fun justReleased(): Boolean {
        return justReleased;
    }
}