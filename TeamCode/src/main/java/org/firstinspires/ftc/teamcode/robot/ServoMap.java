package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class ServoMap {
    private HashMap<String, Servo> servoMap = new HashMap<String, Servo>();

    public ServoMap (HardwareMap hardwareMap, String ...mapValues) {
        for (String value : mapValues) {
            servoMap.put(value, hardwareMap.servo.get(value));
        }
    }

    public HashMap<String, Servo> GetServoMap () {
        return servoMap;
    }
}
