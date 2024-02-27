package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotConfig {
    /* drivetrain motor names */
    public static final String DRIVE_LEFT_BACK = "driveLeftBack";
    public static final String DRIVE_RIGHT_BACK = "driveRightBack";
    public static final String DRIVE_LEFT_FRONT = "driveLeftFront";
    public static final String DRIVE_RIGHT_FRONT = "driveRightFront";

    /* IMU name and Control Hub pose */
    public static final String IMU_NAME = "imu"; // default name
    public static RevHubOrientationOnRobot.LogoFacingDirection CH_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection CH_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    /* webcam names */
    public static final String WEBCAM_BACK = "webcamBack";
    public static final String WEBCAM_FRONT = "webcamFront";

    /* servo names */
    public static final String SERVO_LAUNCHER = "servoLauncher";
    public static final String SERVO_OUTTAKE = "servoOuttake";
    /* motor names */
    public static final String DC_INTAKE = "DCIntake";

    /* slide motor names */
    public static final String DC_SLIDE = "DCSlide";

    /* Sensor names */
    public static final String SENSOR = "colorSensor";
}
