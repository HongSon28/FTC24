package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Outtake {
    private Servo servo;
    private final double SERVO_POSITION = 1;
    public Outtake (HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, RobotConfig.SERVO_OUTTAKE);
        servo.setPosition(0);
    }
    public void setState (boolean state) {
        if (state) servo.setPosition(SERVO_POSITION);
        else servo.setPosition(0);
    }
}
