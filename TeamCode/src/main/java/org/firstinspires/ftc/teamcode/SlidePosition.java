package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Slide Position Calibration")
public class SlidePosition extends LinearOpMode {
    private DcMotor slide;

    @Override
    public void runOpMode() {
        slide = hardwareMap.get(DcMotor.class, RobotConfig.DC_SLIDE);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            slide.setPower(-gamepad1.left_stick_y/5);
            telemetry.addData("Left slide pos.", slide.getCurrentPosition());
            telemetry.update();
        }
    }
}
