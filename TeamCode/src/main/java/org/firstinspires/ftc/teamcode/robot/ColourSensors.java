package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColourSensors extends Sensors {

    private ColorSensor colorSensorRight1;
    private ColorSensor colorSensorRight4;


    private int red = 100; //value is anything around 110 to trigger, resting point for the bottom sensor is 90 and the top sensor is 60, probably higher for bottom because of floor

    public ColourSensors (HardwareMap hardwareMap, String colorSensorRightOne, String colorSensorRightFour) {
        colorSensorRight1 = hardwareMap.colorSensor.get(colorSensorRightOne);
        colorSensorRight4 = hardwareMap.colorSensor.get(colorSensorRightFour);
    }

    public int GetRed () {
        int ringsSensed = 0;

        if (colorSensorRight1.red() > red && colorSensorRight4.red() > red) {
            ringsSensed = 4;
        } else if (colorSensorRight1.red() > red && colorSensorRight4.red() < red) {
            ringsSensed = 1;
        } else {
            ringsSensed = 0;
        }
        return ringsSensed;
    }





}
