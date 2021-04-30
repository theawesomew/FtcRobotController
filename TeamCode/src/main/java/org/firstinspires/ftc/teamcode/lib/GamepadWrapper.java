package org.firstinspires.ftc.teamcode.lib;

import java.util.HashMap;

public class GamepadWrapper {
    private HashMap<String, String> gamepadBindings = new HashMap<String, String>();
    private HashMap<String, Boolean> buttonDownState = new HashMap<String, Boolean>();
    private HashMap<String, Boolean> buttonPressedState = new HashMap<String, Boolean>();

    public GamepadWrapper () {}

    public void updateGamepadInputs () {}

    public boolean isDown (String button) {}

    public boolean isPressed (String button) {}
}
