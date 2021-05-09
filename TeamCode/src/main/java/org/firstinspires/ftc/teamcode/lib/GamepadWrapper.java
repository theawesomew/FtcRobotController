package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.*;
import java.security.Policy;
import java.util.ArrayList;
import java.util.HashMap;

public class GamepadWrapper {
    private Class gamepadClass = Gamepad.class;
    private HashMap<String, String> gamepadBindings = new HashMap<String, String>();
    private HashMap<String, Boolean> buttonDownState = new HashMap<String, Boolean>();
    private HashMap<String, Boolean> buttonPressedState = new HashMap<String, Boolean>();
    private HashMap<String, Boolean> debouncedState = new HashMap<String, Boolean>();
    private HashMap<String, Double> debounceTime = new HashMap<String, Double>();
    private ElapsedTime debounceTimer = new ElapsedTime();
    private String inputs[] = {"a", "b", "x", "y", "dpad_up", "dpad_down", "dpad_left",
            "dpad_right", "left_bumper", "right_bumper", "left_trigger", "right_trigger", "start"};
    private String gamepadStrings[] = {"g1", "g2"};

    public GamepadWrapper (String[] keys, String[] functions) {
        debounceTimer.reset();

        for (String gamepad : gamepadStrings) {
            for (String input : inputs) {
                buttonDownState.put(gamepad+"_"+input, false);
                buttonPressedState.put(gamepad+"_"+input, false);
                debouncedState.put(gamepad+"_"+input, false);
            }
        }

        for (int i = 0; i < Math.min(keys.length, functions.length); ++i) {
            gamepadBindings.put(functions[i], keys[i]);
        }
    }

    public GamepadWrapper () {
        debounceTimer.reset();

        for (String gamepad : gamepadStrings) {
            for (String input : inputs) {
                buttonDownState.put(gamepad+"_"+input, false);
                buttonPressedState.put(gamepad+"_"+input, false);
                debouncedState.put(gamepad+"_"+input, false);
            }
        }
    }

    public void updateGamepadInputs (Gamepad ...gamepads) {
        for (int i = 0; i < Math.min(gamepads.length, gamepadStrings.length); ++i) {
            for (String buttonName : inputs) {
                if (debounceTime.containsKey(gamepadStrings[i]+"_"+buttonName)) {
                    if (debounceTimer.milliseconds() - debounceTime.get(gamepadStrings[i]+"_"+buttonName) >= 50) {
                        debouncedState.put(gamepadStrings[i]+"_"+buttonName, true);
                    }
                }

                try {
                    Field button = gamepadClass.getField(buttonName);
                    button.setAccessible(true);

                    boolean down = (buttonName != "left_trigger" && buttonName != "right_trigger") ? (Boolean) button.get(gamepads[i]) : (float) button.get(gamepads[i]) > 0.7;

                    if (down && !debounceTime.containsKey(gamepadStrings[i]+"_"+buttonName)) {
                        buttonDownState.put(gamepadStrings[i] + "_" + buttonName, true);
                        debounceTime.put(gamepadStrings[i] + "_" + buttonName, debounceTimer.milliseconds());
                    }

                    if (debouncedState.get(gamepadStrings[i]+"_"+buttonName)) {
                        if (down) {
                            buttonDownState.put(gamepadStrings[i] + "_" + buttonName, true);
                            debouncedState.put(gamepadStrings[i] + "_" + buttonName, false);
                            debounceTime.put(gamepadStrings[i] + "_" + buttonName, debounceTimer.milliseconds());
                        } else {
                            buttonDownState.put(gamepadStrings[i] + "_" + buttonName, false);
                            buttonPressedState.put(gamepadStrings[i] + "_" + buttonName, false);
                        }
                    }
                } catch (Throwable e) {
                    // what do you want me to do? if it gets here, just cry bro; trust.
                    // had to ameliorate this comment somehow
                }

            }
        }

        /*if (gamepad1.a) { buttonDownState.put("g1_a", true); }
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
        }*/
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
