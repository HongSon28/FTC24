package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Intake {
    private DcMotor motor;
    private final double SPEED = 1.0;
    private final double REV_SPEED = -0.6;
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.DC_INTAKE);
        motor.setPower(0);
    }
    public void setMotor(int state) {
        if (state == 2) motor.setPower(SPEED);
        else if (state == 1) motor.setPower(REV_SPEED);
        else motor.setPower(0);
    }
}
