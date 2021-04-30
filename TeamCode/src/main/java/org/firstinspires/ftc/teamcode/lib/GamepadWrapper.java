package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;

public class GamepadWrapper {
    private HashMap<String, String> gamepadBindings = new HashMap<String, String>();
    private HashMap<String, Boolean> buttonDownState = new HashMap<String, Boolean>();
    private HashMap<String, Boolean> buttonPressedState = new HashMap<String, Boolean>();
    private String inputs[] = {"a", "b", "x", "y", "dpad_up", "dpad_down", "dpad_left",
            "dpad_right", "left_bumper", "right_bumper", "left_trigger", "right_trigger", "start"};

    public GamepadWrapper (String[] keys, String[] functions) {
        String gamepads[] = {"g1", "g2"};
        for (String gamepad : gamepads) {
            for (String input : inputs) {
                buttonDownState.put(gamepad+"_"+input, false);
                buttonPressedState.put(gamepad+"_"+input, false);
            }
        }

        for (int i = 0; i < Math.min(keys.length, functions.length); ++i) {
            gamepadBindings.put(functions[i], keys[i]);
        }
    }

    public void updateGamepadInputs (Gamepad gamepad1, Gamepad gamepad2) {
        // unfortunately, i can't find any intelligent and efficient method of creating this method
        // so, i'm going to have to hardcode it... death is also an option...
        // also, i realised that this whole idea for debouncing is predicated upon the controller states
        // being accurate. this is why we avoid embedded systems like the plague :)

        if (gamepad1.a) { buttonDownState.put("g1_a", true); }
        if (gamepad1.b) { buttonDownState.put("g1_b", true); }
        if (gamepad1.x) { buttonDownState.put("g1_x", true); }
        if (gamepad1.y) { buttonDownState.put("g1_y", true); }
        if (gamepad1.dpad_up) { buttonDownState.put("g1_dpad_up", true); }
        if (gamepad1.dpad_down) { buttonDownState.put("g1_dpad_down", true); }
        if (gamepad1.dpad_left) { buttonDownState.put("g1_dpad_left", true); }
        if (gamepad1.dpad_right) { buttonDownState.put("g1_dpad_right", true); }
        if (gamepad1.left_bumper) { buttonDownState.put("g1_left_bumper", true); }
        if (gamepad1.right_bumper) { buttonDownState.put("g1_right_bumper", true); }
        if (gamepad1.left_trigger > 0.7) { buttonDownState.put("g1_left_trigger", true); }
        if (gamepad1.right_trigger > 0.7) { buttonDownState.put("g1_right_trigger", true); }
        if (gamepad1.start) { buttonDownState.put("g1_start", true); }

        if (!gamepad1.a) {
            buttonDownState.put("g1_a", false);
            buttonPressedState.put("g1_a", false);
        }
        if (!gamepad1.b) {
            buttonDownState.put("g1_b", false);
            buttonPressedState.put("g1_b", false);
        }
        if (!gamepad1.x) {
            buttonDownState.put("g1_x", false);
            buttonPressedState.put("g1_x", false);
        }
        if (!gamepad1.y) {
            buttonDownState.put("g1_y", false);
            buttonPressedState.put("g1_y", false);
        }
        if (!gamepad1.dpad_up) {
            buttonDownState.put("g1_dpad_up", false);
            buttonPressedState.put("g1_dpad_up", false);
        }
        if (!gamepad1.dpad_down) {
            buttonDownState.put("g1_dpad_down", false);
            buttonPressedState.put("g1_dpad_down", false);
        }
        if (!gamepad1.dpad_left) {
            buttonDownState.put("g1_dpad_left", false);
            buttonPressedState.put("g1_dpad_left", false);
        }
        if (!gamepad1.dpad_right) {
            buttonDownState.put("g1_dpad_right", false);
            buttonPressedState.put("g1_dpad_right", false);
        }
        if (!gamepad1.left_bumper) {
            buttonDownState.put("g1_left_bumper", false);
            buttonPressedState.put("g1_left_bumper", false);
        }
        if (!gamepad1.right_bumper) {
            buttonDownState.put("g1_right_bumper", false);
            buttonPressedState.put("g1_right_bumper", false);
        }
        if (!(gamepad1.left_trigger > 0.7)) {
            buttonDownState.put("g1_left_trigger", false);
            buttonPressedState.put("g1_left_trigger", false);
        }
        if (!(gamepad1.right_trigger > 0.7)) {
            buttonDownState.put("g1_right_trigger", false);
            buttonPressedState.put("g1_right_trigger", false);
        }
        if (!gamepad1.start) {
            buttonDownState.put("g1_start", false);
            buttonPressedState.put("g1_start", false);
        }

        // gamepad2 is equally important UwU

        if (gamepad2.a) { buttonDownState.put("g2_a", true); }
        if (gamepad2.b) { buttonDownState.put("g2_b", true); }
        if (gamepad2.x) { buttonDownState.put("g2_x", true); }
        if (gamepad2.y) { buttonDownState.put("g2_y", true); }
        if (gamepad2.dpad_up) { buttonDownState.put("g2_dpad_up", true); }
        if (gamepad2.dpad_down) { buttonDownState.put("g2_dpad_down", true); }
        if (gamepad2.dpad_left) { buttonDownState.put("g2_dpad_left", true); }
        if (gamepad2.dpad_right) { buttonDownState.put("g2_dpad_right", true); }
        if (gamepad2.left_bumper) { buttonDownState.put("g2_left_bumper", true); }
        if (gamepad2.right_bumper) { buttonDownState.put("g2_right_bumper", true); }
        if (gamepad2.left_trigger > 0.7) { buttonDownState.put("g2_left_trigger", true); }
        if (gamepad2.right_trigger > 0.7) { buttonDownState.put("g2_right_trigger", true); }
        if (gamepad2.start) { buttonDownState.put("g2_start", true); }

        if (!gamepad2.a) {
            buttonDownState.put("g2_a", false);
            buttonPressedState.put("g2_a", false);
        }
        if (!gamepad2.b) {
            buttonDownState.put("g2_b", false);
            buttonPressedState.put("g2_b", false);
        }
        if (!gamepad2.x) {
            buttonDownState.put("g2_x", false);
            buttonPressedState.put("g2_x", false);
        }
        if (!gamepad2.y) {
            buttonDownState.put("g2_y", false);
            buttonPressedState.put("g2_y", false);
        }
        if (!gamepad2.dpad_up) {
            buttonDownState.put("g2_dpad_up", false);
            buttonPressedState.put("g2_dpad_up", false);
        }
        if (!gamepad2.dpad_down) {
            buttonDownState.put("g2_dpad_down", false);
            buttonPressedState.put("g2_dpad_down", false);
        }
        if (!gamepad2.dpad_left) {
            buttonDownState.put("g2_dpad_left", false);
            buttonPressedState.put("g2_dpad_left", false);
        }
        if (!gamepad2.dpad_right) {
            buttonDownState.put("g2_dpad_right", false);
            buttonPressedState.put("g2_dpad_right", false);
        }
        if (!gamepad2.left_bumper) {
            buttonDownState.put("g2_left_bumper", false);
            buttonPressedState.put("g2_left_bumper", false);
        }
        if (!gamepad2.right_bumper) {
            buttonDownState.put("g2_right_bumper", false);
            buttonPressedState.put("g2_right_bumper", false);
        }
        if (!(gamepad2.left_trigger > 0.7)) {
            buttonDownState.put("g2_left_trigger", false);
            buttonPressedState.put("g2_left_trigger", false);
        }
        if (!(gamepad2.right_trigger > 0.7)) {
            buttonDownState.put("g2_right_trigger", false);
            buttonPressedState.put("g2_right_trigger", false);
        }
        if (!gamepad2.start) {
            buttonDownState.put("g2_start", false);
            buttonPressedState.put("g2_start", false);
        }
    }

    public boolean isDown (String button) {
        if (gamepadBindings.containsKey(button)) {
            return buttonDownState.get(gamepadBindings.get(button));
        }
        return buttonDownState.get(button);
    }

    public boolean isPressed (String button) {
        if (gamepadBindings.containsKey(button)) {
            button = gamepadBindings.get(button);
        }
        if (buttonPressedState.get(button)) {
            return false;
        } else if (buttonDownState.get(button)) {
            buttonPressedState.put(button, true);
            return true;
        } else {
            return false;
        }
    }
}
