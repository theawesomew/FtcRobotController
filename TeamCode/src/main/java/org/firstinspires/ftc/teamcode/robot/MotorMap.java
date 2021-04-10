package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class MotorMap {
    private HashMap<String, DcMotor> motorMap = new HashMap<String, DcMotor>();

    public MotorMap (HardwareMap hardwareMap, String ...mapValues) {
        for (String value : mapValues) {
            motorMap.put(value, hardwareMap.get(DcMotor.class, value));
        }
    }

    public HashMap<String, DcMotor> GetMotorMap () {
        return motorMap;
    }
}
