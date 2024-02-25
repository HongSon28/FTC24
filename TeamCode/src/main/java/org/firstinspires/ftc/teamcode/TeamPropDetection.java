package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ControlAlgorithms.PIDController;
import org.firstinspires.ftc.teamcode.Drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrivetrain;

@TeleOp(name = "Team Prop Detection")
public class TeamPropDetection extends LinearOpMode {
    private MecanumDrive drivetrain;
    private NormalizedColorSensor colorSensor;
    private IMU imu;
    private final double STARTING_DISTANCE = 23;
    private double currentHeading;
    private static final double DETECT_RANGE = 17.5;
    private final double SCAN_ANGLE = 20;
    private final double MAX_TURNING_SPEED = 0.5;
    private final float gain = 2;
    @Override
    public void runOpMode() {
        /* initialize IMU */
        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RobotConfig.CH_LOGO_DIRECTION, RobotConfig.CH_USB_DIRECTION));
        imu.initialize(imuParams);
        imu.resetYaw(); // heading

        colorSensor = hardwareMap.get(NormalizedColorSensor.class,RobotConfig.SENSOR);
        colorSensor.setGain(gain);

        waitForStart();
        /* Run to Spike Mark */
        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Vector2d destCord = new Vector2d(
                STARTING_DISTANCE,
                0
        );
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).strafeToConstantHeading(destCord).build());
        drivetrain.updatePoseEstimate();
        if (!Detected()) {
            Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(Math.toRadians(40)).build());
            drivetrain.updatePoseEstimate();
            if (!Detected()) {
                Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(Math.toRadians(-80)).build());
                drivetrain.updatePoseEstimate();
            }
        }
        currentHeading = Math.toDegrees(drivetrain.pose.heading.toDouble());
        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)*2);
        telemetry.addData("Detected Heading:", "%.3f deg", currentHeading);
        telemetry.update();
    }
    private boolean Detected() {
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) * 2;
        telemetry.addData("RANGE = ",distance);
        telemetry.update();
        return distance <= DETECT_RANGE;
    }
}
