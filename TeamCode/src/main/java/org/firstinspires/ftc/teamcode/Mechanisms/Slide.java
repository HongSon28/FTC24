package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;

import java.util.PrimitiveIterator;

public class Slide {
    private DcMotor motor;
    private final int POSITION = -5250;
    private double SPEED = 0.4;
    private final double EXTEND_PERCENTAGE = 0.5;
    public Slide (HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.DC_SLIDE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setPower(-SPEED);

        motor.setTargetPosition(0);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setState(int state) {
        if (state == 2) {
            motor.setTargetPosition(POSITION);
        } else if (state == 1) {
            motor.setTargetPosition((int) (POSITION * EXTEND_PERCENTAGE));
        } else {
            motor.setTargetPosition(0);
        }
    }
    public void setNewPower() {
        SPEED = 1;
        motor.setPower(-SPEED);
    }
}
