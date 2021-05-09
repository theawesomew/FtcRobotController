package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Shooter extends Mechanisms {
    private DcMotor shooter;
    private boolean motorsMoving = false;
    private int motorDistance = 0;

    public boolean within (int value, int setValue, int error) {
        if (Math.abs(value-setValue) < error) {
            return true;
        }
        return false;
    }

    public Shooter(MotorMap mechMap) {
        shooter = mechMap.GetMotorMap().get("shoot");
    }

    public void SetPower(double power) {
        shooter.setPower(power);
    }

    /*public boolean SetDistance(double distance) {
        if (!motorsMoving) {
            motorDistance = (int) (distance);

            shooter.setTargetPosition(shooter.getCurrentPosition() + motorDistance);
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            SetPower(1);
            motorsMoving = true;
        } else if ( within(shooter.getCurrentPosition(), shooter.getTargetPosition(), 10)){
            motorsMoving = false;
            return true;
        }

        return  false;

    }*/

    public boolean Shoot () {
        if (!shooter.isBusy()) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setTargetPosition(10 * 1440);
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shooter.setPower(1);
        }
        return true;
    }


}
