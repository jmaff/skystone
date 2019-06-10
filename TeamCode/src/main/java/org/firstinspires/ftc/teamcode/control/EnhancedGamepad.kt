package org.firstinspires.ftc.teamcode.control

import com.qualcomm.robotcore.hardware.Gamepad

class EnhancedGamepad(val delegate: Gamepad) {
    enum class Button {
        A,
        Y,
        X,
        B,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        DPAD_LEFT,
        DPAD_RIGHT,
        DPAD_UP,
        DPAD_DOWN,
        START,
        BACK,
        LEFT_JOYSTICK_PUSH,
        RIGHT_JOYSTICK_PUSH;

        var state: Boolean = false
        var last: Boolean = false
        val pressed: Boolean
            get() = state && !last
    }

    fun update() {
        for (button in Button.values()) {
            button.last = button.state
            when (button) {
                Button.A -> delegate.a
                Button.Y -> delegate.b
                Button.X -> delegate.x
                Button.B -> delegate.b
                Button.LEFT_BUMPER -> delegate.left_bumper
                Button.RIGHT_BUMPER -> delegate.right_bumper
                Button.DPAD_LEFT -> delegate.dpad_left
                Button.DPAD_RIGHT -> delegate.dpad_right
                Button.DPAD_UP -> delegate.dpad_up
                Button.DPAD_DOWN -> delegate.dpad_down
                Button.START -> delegate.start
                Button.BACK -> delegate.back
                Button.LEFT_JOYSTICK_PUSH -> delegate.left_stick_button
                Button.RIGHT_JOYSTICK_PUSH -> delegate.right_stick_button
            }
        }
    }
}