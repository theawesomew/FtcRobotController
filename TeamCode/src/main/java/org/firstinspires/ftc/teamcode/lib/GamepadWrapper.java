package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.*;
import java.security.Policy;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * This class is designated to process gamepad inputs from both gamepads
 **/

public class GamepadWrapper {
    private Class gamepadClass = Gamepad.class;
    private HashMap<String, String> gamepadBindings = new HashMap<String, String>();
    private HashMap<String, Boolean> buttonDownState = new HashMap<String, Boolean>();
    private HashMap<String, Boolean> buttonPressedState = new HashMap<String, Boolean>();
    private HashMap<String, Boolean> debouncedState = new HashMap<String, Boolean>();
    private HashMap<String, Double> debounceTime = new HashMap<String, Double>();
    private ElapsedTime debounceTimer = new ElapsedTime();
    private final String[] gamepadButtons = {"a", "b", "x", "y", "dpad_up", "dpad_down", "dpad_left",
            "dpad_right", "left_bumper", "right_bumper", "left_trigger", "right_trigger", "start"};
    private final String[] gamepadStrings = {"g1", "g2"};

    public GamepadWrapper (String[] keys, String[] functions) {
        debounceTimer.reset();

        for (String gamepad : gamepadStrings) {
            for (String input : gamepadButtons) {
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
            for (String gamepadButton : gamepadButtons) {
                buttonDownState.put(gamepad+"_"+gamepadButton, false);
                buttonPressedState.put(gamepad+"_"+gamepadButton, false);
                debouncedState.put(gamepad+"_"+gamepadButton, false);
            }
        }
    }

    public void updateGamepadInputs (Gamepad ...gamepads) {
        for (int i = 0; i < Math.min(gamepads.length, gamepadStrings.length); ++i) {
            for (String buttonName : gamepadButtons) {
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
