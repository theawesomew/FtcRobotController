package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.GamepadWrapper;
import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="TestOpModeColourSensors", group="LinearOpMode")
public class AutonomousColorSensorTest extends OpMode {

    private MotorMap motorMap;
    private GamepadWrapper gamepadWrapper;
    private Robot robot;

    private ColorSensor right1;
    private ColorSensor right2;

    private int red1;
    private int red2;

    @Override
    public void init() {

        right1 = hardwareMap.get(ColorSensor.class, "colorSensorRight1");
        right2 = hardwareMap.get(ColorSensor.class, "colorSensorRight4");



    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        red1 = right1.red();
        red2 = right2.red();

        telemetry.addData("Red Value1: ", red1);
        telemetry.addData("Red Value2: ", red2);

    }

    @Override
    public void stop() {

    }



}
