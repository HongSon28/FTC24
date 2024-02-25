package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.ControlAlgorithms.PIDController;
import org.firstinspires.ftc.teamcode.Drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Slide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Drive to AprilTag")
public class DriveToAprilTag extends LinearOpMode {
    private MecanumDrive drivetrain;

    /* camera/CV */
    private VisionPortal visionPortal;
    private boolean cameraStreamingPaused = false;
    private AprilTagProcessor aprilTag; // AprilTag detector

    private static final int DESIRED_TAG_ID = 1;
    private static final double DESIRED_DISTANCE = 5; // inch
    private static final double RR_STOPPING_DISTANCE = 5;

    private static final double DISTANCE_ERR_TOLERANCE = 0.15;
    private static final double HEADING_ERR_TOLERANCE = 0.5;
    private static final double YAW_ERR_TOLERANCE = 0.5;
    private static final double MAX_SPEED = 0.5;
    private ElapsedTime time;
    private Slide slide;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        slide = new Slide(hardwareMap);
        time = new ElapsedTime();

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

        boolean unattendedDrive = false;
        boolean rbPressed = false;
        while(opModeIsActive()) {
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
                        drivetrain.pose.position.x + detectedTag.ftcPose.range * Math.sin(gamma) - Math.max(DESIRED_DISTANCE, RR_STOPPING_DISTANCE) * Math.cos(destHeading),
                        drivetrain.pose.position.y - detectedTag.ftcPose.range * Math.cos(gamma) - Math.max(DESIRED_DISTANCE, RR_STOPPING_DISTANCE) * Math.sin(destHeading)
                );

                telemetry.addData("Dest. X", destCoord.x);
                telemetry.addData("Dest. Y", destCoord.y);
                telemetry.addData("Dest. hdg", Math.toDegrees(destHeading));
            } else telemetry.addData(">", "Drive robot around to pick up a tag");
            if(gamepad1.right_bumper && detectedTag != null && detectedTag.ftcPose.range > Math.min(DESIRED_DISTANCE, RR_STOPPING_DISTANCE)) {
                if(detectedTag.ftcPose.range > RR_STOPPING_DISTANCE) Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose).splineTo(destCoord, destHeading).build());

                /* Detect Tag again */
                if(detectedTag.ftcPose.range > DESIRED_DISTANCE) {
                    detectedTag = null;
                    currentDetections = aprilTag.getDetections();
                    for(AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null && detection.id == DESIRED_TAG_ID) { // found our tag
                            detectedTag = detection;
                            break;
                        }
                    }
                    telemetry.addData(">","Track Tag again");
                    if (gamepad1.right_bumper && detectedTag != null) trackTag(detectedTag);
                }

                slide.setState(2);
            }

            /* normal driving */
            drivetrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drivetrain.updatePoseEstimate();

            telemetry.addData("x", drivetrain.pose.position.x);
            telemetry.addData("y", drivetrain.pose.position.y);
            telemetry.addData("Heading", Math.toDegrees(drivetrain.pose.heading.toDouble()));

            telemetry.update();
        }
    }

    private void trackTag(AprilTagDetection targetTag) {
        /* PID controller */
        PIDCoefficients DRIVE_X_YAW_PID = new PIDCoefficients(0.005, 0.0, 0.000005);
        PIDController driveController = new PIDController(DrivetrainConstants.DRIVE_Y_PID, -MAX_SPEED, MAX_SPEED);
        PIDController strafeController = new PIDController(DRIVE_X_YAW_PID, -MAX_SPEED, MAX_SPEED);
        PIDController turnController = new PIDController(DrivetrainConstants.DRIVE_ROT_PID, -MAX_SPEED, MAX_SPEED);

        double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE;
        double headingError = -targetTag.ftcPose.bearing; // positive bearing = turn counter-clockwise = negative turn value
        double yawError = targetTag.ftcPose.yaw;
        while (Math.abs(rangeError) > DISTANCE_ERR_TOLERANCE || Math.abs(headingError) > HEADING_ERR_TOLERANCE || Math.abs(yawError) > YAW_ERR_TOLERANCE) {

            double x = strafeController.control(yawError);
            double y = driveController.control(rangeError);
            double rot = turnController.control(headingError);

            drivetrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            y,
                            x
                    ),
                    rot
            ));
            rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE;
            headingError = -targetTag.ftcPose.bearing; // positive bearing = turn counter-clockwise = negative turn value
            yawError = targetTag.ftcPose.yaw;
            telemetry.addData("x", rangeError);
            telemetry.addData("y", headingError);
            telemetry.addData("Heading", yawError);
        }

        drivetrain.updatePoseEstimate();
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
}
