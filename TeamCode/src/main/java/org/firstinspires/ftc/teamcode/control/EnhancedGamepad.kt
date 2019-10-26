package org.firstinspires.ftc.teamcode.control

import com.qualcomm.robotcore.hardware.Gamepad

class EnhancedGamepad(val delegate: Gamepad) {
    
    var A = Button()
    var Y = Button()
    var X = Button()
    var B = Button()
    var LEFT_BUMPER = Button()
    var RIGHT_BUMPER = Button()
    var DPAD_LEFT = Button()
    var DPAD_RIGHT = Button()
    var DPAD_UP = Button()
    var DPAD_DOWN = Button()
    var START = Button()
    var BACK = Button()
    var LEFT_JOYSTICK_PUSH = Button()
    var RIGHT_JOYSTICK_PUSH = Button()
    
    fun update() {
        A.last = A.state
        Y.last = Y.state
        X.last = X.state
        B.last = B.state
        LEFT_BUMPER.last = LEFT_BUMPER.state
        RIGHT_BUMPER.last = RIGHT_BUMPER.state
        DPAD_LEFT.last = DPAD_LEFT.state
        DPAD_RIGHT.last = DPAD_RIGHT.state
        DPAD_UP.last = DPAD_UP.state
        DPAD_DOWN.last = DPAD_DOWN.state
        START.last = START.state
        BACK.last = BACK.state
        LEFT_JOYSTICK_PUSH.last = LEFT_JOYSTICK_PUSH.state
        RIGHT_JOYSTICK_PUSH.last = RIGHT_JOYSTICK_PUSH.state

        A.state = delegate.a
        Y.state = delegate.y
        X.state = delegate.x
        B.state = delegate.b
        LEFT_BUMPER.state = delegate.left_bumper
        RIGHT_BUMPER.state = delegate.right_bumper
        DPAD_LEFT.state = delegate.dpad_left
        DPAD_RIGHT.state = delegate.dpad_right
        DPAD_UP.state = delegate.dpad_up
        DPAD_DOWN.state = delegate.dpad_down
        START.state = delegate.start
        BACK.state = delegate.back
        LEFT_JOYSTICK_PUSH.state = delegate.left_stick_button
        RIGHT_JOYSTICK_PUSH.state = delegate.right_stick_button
    }

    class Button {
        var state = false
        var last = false
        val pressed: Boolean
            get() = state && !last
    }
}