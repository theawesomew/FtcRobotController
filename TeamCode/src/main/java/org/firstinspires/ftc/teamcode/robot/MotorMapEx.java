package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class MotorMapEx {
    private HashMap<String, DcMotorEx> motorMap = new HashMap<String, DcMotorEx>();

    public MotorMapEx (HardwareMap hardwareMap, String ...mapValues) {
        for (String value : mapValues) {
            motorMap.put(value, hardwareMap.get(DcMotorEx.class, value));
        }

        for (DcMotor motor : motorMap.values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public HashMap<String, DcMotorEx> GetMotorMap () {
        return motorMap;
    }
}
