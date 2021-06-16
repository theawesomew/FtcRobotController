package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColourSensors extends Sensors {

    private ColorSensor colorSensorRight1;
    private ColorSensor colorSensorRight4;

    private int ringsSensed = 0;
    private int red = 0; //value needs to be found by figuring out what value of red is detected when sensors are put in front of rings

    public ColourSensors (HardwareMap hardwareMap, String colorSensorRightOne, String colorSensorRightFour) {
        colorSensorRight1 = hardwareMap.colorSensor.get(colorSensorRightOne);
        colorSensorRight4 = hardwareMap.colorSensor.get(colorSensorRightFour);
    }

    public int GetRed () {
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
