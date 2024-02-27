package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ControlAlgorithms.PIDController;
import org.firstinspires.ftc.teamcode.Drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.Mechanisms.Slide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Autonomous(name = "Autonomous OpMode")
public class AutonomousOpMode extends DriveTeleOp{
    private MecanumDrive drivetrain;

    /* camera/CV */
    private VisionPortal visionPortal;
    private boolean cameraStreamingPaused = false;
    private AprilTagProcessor aprilTag; // AprilTag detector

    private static int DESIRED_TAG_ID;
    private static final double DESIRED_DISTANCE = 5; // inch
    private ElapsedTime time;
    private Slide slide;
    private Intake intake;
    private Outtake outtake;
    private NormalizedColorSensor colorSensor;
    private IMU imu;
    private static final double DETECT_RANGE = 17.5;
    private final double STARTING_DISTANCE = 23;
    private final double SCAN_ANGLE = 20;
    private final double MAX_TURNING_SPEED = 0.5;

    private boolean alliance; //false = red, blue = true;
    private boolean startingPosition; //false = closer, true = further
    private boolean init;
    @Override
    public void runOpMode() {
        slide = new Slide(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        time = new ElapsedTime();
        init = false;

        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(RobotConfig.CH_LOGO_DIRECTION, RobotConfig.CH_USB_DIRECTION));
        imu.initialize(imuParams);
        imu.resetYaw(); // heading

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.SENSOR);

        /* initialize AprilTag */
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2); // TODO: change this to adapt to gameplay conditions
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.WEBCAM_FRONT))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        setManualExposure(6, 150);

        waitForStart();

        while(opModeIsActive()) {
            if (!init) {
                while (true) {
                    /* initialize starting position */
                    if (gamepad1.triangle) {
                        alliance = true;
                        startingPosition = false;
                        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(84, 8.5, Math.toRadians(90)));
                        telemetry.addData("> ", "blue close");
                        break;
                    } else if (gamepad1.square) {
                        alliance = true;
                        startingPosition = true;
                        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(36, 8.5, Math.toRadians(90)));
                        telemetry.addData("> ", "blue far");
                        break;
                    } else if (gamepad1.cross) {
                        alliance = false;
                        startingPosition = true;
                        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(36, 135.5, Math.toRadians(-90)));
                        telemetry.addData("> ", "red far");
                        break;
                    } else if (gamepad1.circle) {
                        alliance = false;
                        startingPosition = false;
                        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(84, 135.5, Math.toRadians(-90)));
                        telemetry.addData("> ", "red close");
                        break;
                    }
                }
                init = true;
            }
            telemetry.addData("> ", "Init complete");
            telemetry.update();

            /* detect Team Prop */
            Vector2d destCord = new Vector2d(
                    drivetrain.pose.position.x + (((alliance) ? STARTING_DISTANCE : -STARTING_DISTANCE)),
                    drivetrain.pose.position.y
            );
            Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).strafeToConstantHeading(destCord).build());
            drivetrain.updatePoseEstimate();

            //Scan for april tag
            if (!Detected()) {
                Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(Math.toRadians(40)).build());
                drivetrain.updatePoseEstimate();
                if (!Detected()) {
                    Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(Math.toRadians(-80)).build());
                    drivetrain.updatePoseEstimate();
                    DESIRED_TAG_ID = 3;
                } else DESIRED_TAG_ID = 1;
            } else DESIRED_TAG_ID = 2;
            if (!alliance)  DESIRED_TAG_ID += 3;

            /* Drop pixel on spikemark */
            Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).turn(Math.toRadians(180)).build());
            drivetrain.updatePoseEstimate();
            intake.setMotor(1);

            /* move to observation point */
            if (alliance) Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).splineTo(new Vector2d(84,60), Math.toRadians(-20)).build());
            else Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).splineTo(new Vector2d(84,84), Math.toRadians(20)).build());
            drivetrain.updatePoseEstimate();

            /* detect AprilTags */
            AprilTagDetection detectedTag = null;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for(AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == DESIRED_TAG_ID) { // found our tag
                    detectedTag = detection;
                    break;
                }
            }

            Vector2d destCoord = null;
            double destHeading = Double.NaN;

            if(detectedTag != null) {
                telemetry.addData(">", "Hold left bumper to drive towards detected tag (right bumper for unattended driving):");
                telemetry.addData("Tag", "ID %d (%s)", detectedTag.id, detectedTag.metadata.name);
                telemetry.addData("Range", "%5.1f in.", detectedTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f deg.", detectedTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f deg.", detectedTag.ftcPose.yaw);

                double bearingRad = Math.toRadians(detectedTag.ftcPose.bearing);
                double yawRad = Math.toRadians(detectedTag.ftcPose.yaw);
                double heading = drivetrain.pose.heading.toDouble();
                double gamma = heading + bearingRad + Math.PI / 2;
                destHeading = yawRad + heading;
                destCoord = new Vector2d(
                        drivetrain.pose.position.x + detectedTag.ftcPose.range * Math.sin(gamma) - DESIRED_DISTANCE * Math.cos(destHeading),
                        drivetrain.pose.position.y - detectedTag.ftcPose.range * Math.cos(gamma) - DESIRED_DISTANCE * Math.sin(destHeading)
                );

                telemetry.addData("Dest. X", destCoord.x);
                telemetry.addData("Dest. Y", destCoord.y);
                telemetry.addData("Dest. hdg", Math.toDegrees(destHeading));
            } else telemetry.addData(">", "Drive robot around to pick up a tag");

            if(gamepad1.right_bumper && detectedTag != null && detectedTag.ftcPose.range > DESIRED_DISTANCE) {
                Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).splineTo(destCoord, destHeading).build());

                BackDropAction();

                while(opModeIsActive()); // we have nothing left to do, so we'll wait while the OP mode is still active
            }

        }
        drivetrain.updatePoseEstimate();

        telemetry.addData("x", drivetrain.pose.position.x);
        telemetry.addData("y", drivetrain.pose.position.y);
        telemetry.addData("Heading", Math.toDegrees(drivetrain.pose.heading.toDouble()));

        telemetry.update();
    }

    private void BackDropAction() {
        slide.setState(1);

        sleep(5000);

        outtake.setState(true);

        sleep(2000);

        outtake.setState(false);
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
