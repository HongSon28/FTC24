package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.Mechanisms.Slide;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Autonomous OpMode Blue")
public class AutonomousBlue extends LinearOpMode
{
    private MecanumDrive drivetrain;

    /* camera/CV */
    private VisionPortal visionPortal;
    private boolean cameraStreamingPaused = false;
    private AprilTagProcessor aprilTag; // AprilTag detector

    private static int DESIRED_TAG_ID;
    private static final double DESIRED_DISTANCE = 4; // inch
    private ElapsedTime time;
    private Slide slide;
    private Intake intake;
    private Outtake outtake;
    private NormalizedColorSensor colorSensor;
    private IMU imu;
    private static final double DETECT_RANGE = 15.75;
    private final double STARTING_DISTANCE = 22;
    private final float gain = 2;

    private final double TIMEOUT = 3.0;
    @Override
    public void runOpMode() {
        slide = new Slide(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        outtake.setState(true);
        time = new ElapsedTime();

        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RobotConfig.CH_LOGO_DIRECTION, RobotConfig.CH_USB_DIRECTION));
        imu.initialize(imuParams);
        imu.resetYaw(); // heading

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.SENSOR);
        colorSensor.setGain(gain);

        /* initialize AprilTag */
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.WEBCAM_FRONT))
                .addProcessor(aprilTag)
                .build();

        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        setManualExposure(4, 80);

        waitForStart();

        //Move to spike mark
        Vector2d destCord = new Vector2d(
                STARTING_DISTANCE,
                0
        );
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).strafeToConstantHeading(destCord).build());
        drivetrain.updatePoseEstimate();

        //Scan for team prop
        if (!Detected()) {
            Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(Math.toRadians(40)).build());
            drivetrain.updatePoseEstimate();
            if (!Detected()) {
                Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(Math.toRadians(-80)).build());
                drivetrain.updatePoseEstimate();
                DESIRED_TAG_ID = 3;
            } else DESIRED_TAG_ID = 1;
        } else DESIRED_TAG_ID = 2;

        /* Drop pixel on spikemark */
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(Math.toRadians(180)).build());
        drivetrain.updatePoseEstimate();
        intake.setMotor(1);
        sleep(3000);
        intake.setMotor(0);

        /* Turn to observe point */
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turnTo(Math.toRadians(90)).build());
        drivetrain.updatePoseEstimate();
        ElapsedTime time = new ElapsedTime(); time.reset();

        /* detect AprilTags */
        AprilTagDetection detectedTag = null;
        Vector2d destCoord = null;
        double destHeading = Double.NaN;
        while(detectedTag == null && time.seconds() < TIMEOUT) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for(AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == DESIRED_TAG_ID) { // found our tag
                    detectedTag = detection;
                    break;
                }
            }

            if(detectedTag != null) {
                double bearingRad = Math.toRadians(detectedTag.ftcPose.bearing);
                double yawRad = Math.toRadians(detectedTag.ftcPose.yaw);
                double heading = drivetrain.pose.heading.toDouble();
                double gamma = heading + bearingRad + Math.PI / 2;
                destHeading = yawRad + heading;
                destCoord = new Vector2d(
                        drivetrain.pose.position.x + detectedTag.ftcPose.range * Math.sin(gamma) - DESIRED_DISTANCE * Math.cos(destHeading),
                        drivetrain.pose.position.y - detectedTag.ftcPose.range * Math.cos(gamma) - DESIRED_DISTANCE * Math.sin(destHeading)
                );
            }
        }

        if(detectedTag != null && detectedTag.ftcPose.range > DESIRED_DISTANCE) {
            Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).splineTo(destCoord, destHeading).build());

            BackDropAction();

            while(opModeIsActive()); // we have nothing left to do, so we'll wait while the OP mode is still active
        } else {
            Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).strafeToConstantHeading(new Vector2d(0, -2 * 24)).build());
        }

        drivetrain.updatePoseEstimate();

        telemetry.addData("x", drivetrain.pose.position.x);
        telemetry.addData("y", drivetrain.pose.position.y);
        telemetry.addData("Heading", Math.toDegrees(drivetrain.pose.heading.toDouble()));

        telemetry.update();
    }

    private void BackDropAction() {
        slide.setState(1);

        sleep(2500);

        outtake.setState(false);
        sleep(1000);

        Vector2d destCord = new Vector2d(
                drivetrain.pose.position.x + 4.5,
                drivetrain.pose.position.y + 10.5
        );
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).strafeToConstantHeading(destCord).build());
        drivetrain.updatePoseEstimate();

        outtake.setState(true);
        slide.setState(0);
    }
    private void setManualExposure(int msec, int gain) {
        if(visionPortal == null) return;

        if(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while(!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING))
                sleep(20); // wait until camera goes live
        }

        if(!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if(exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual); // set to manual exposure
                sleep(50);
            }
            exposureControl.setExposure((long)msec, TimeUnit.MILLISECONDS);
            sleep(20);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    private boolean Detected() {
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) * 2;
        telemetry.addData("RANGE = ",distance);
        telemetry.update();
        return distance <= DETECT_RANGE;
    }
}