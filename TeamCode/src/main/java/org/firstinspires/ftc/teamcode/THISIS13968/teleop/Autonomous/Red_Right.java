package org.firstinspires.ftc.teamcode.THISIS13968.teleop.Autonomous;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.THISIS13968.Camera.TeamPropDetectPipeline;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name="Camera Testing", group="Autonomous")
public class Red_Right extends OpMode {
    Robot13968 robot;

    private VisionPortal visionPortal;
    private TeamPropDetectPipeline propDetect;
    private AprilTagProcessor atagProcessor;

    @Override
    public void init() {
        robot = Robot13968.resetInstance(); //resets bot

        robot.init(hardwareMap, true); //initializes robot for driving with imu

        robot.setDetectColor(Robot13968.DetectColor.RED);


        double minArea = 200; // the minimum area for the detection to consider for your prop
        // controller = new PIDController(p,i,d);
        atagProcessor = new AprilTagProcessor.Builder().build();
        propDetect = new TeamPropDetectPipeline(
                robot.getDetectColor(),
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame

        );

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera")) // the camera on your robot is named "Webcam 1" by default
                .addProcessors(atagProcessor, propDetect)
                .build();

        // Tell the driver that initialization is complete.]
        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", propDetect.getRecordedPropPosition());
        telemetry.addData("Currently Detected Mass Center", "x: " + propDetect.getLargestContourX() + ", y: " + propDetect.getLargestContourY());


    }
    @Override
    public void start() {


        TeamPropDetectPipeline.PropPositions recordedPropPosition = propDetect.getRecordedPropPosition();
       //if (recordedPropPosition == UNFOUND) {recordedPropPosition = MIDDLE;}  // a guess

        Pose2d startPose = new Pose2d(-33.89, 63.75, Math.toRadians(270.00));

        robot.driveTrain.setPoseEstimate(startPose);

        TrajectorySequence left =  robot.driveTrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-28.89, 38.0), Math.toRadians(-45))
                .setReversed(true)
                .splineTo( new Vector2d(-27.0, 58.5), Math.toRadians(0.0))
                .back(62)
                .lineTo(new Vector2d(48,34))
                //  .lineTo(new Vector2d(0,36))
                .build();

        TrajectorySequence middle = robot.driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-33.89, 32.0))
                .setReversed(true)
                .back(20)
                .splineTo( new Vector2d(-21.0, 58.5), Math.toRadians(0.0))
                .back(60)
                .lineTo(new Vector2d(48,34))
                .build();

        TrajectorySequence right = robot.driveTrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-42.5, 42.0), Math.toRadians(225.0))
                .setReversed(true)
                .splineTo( new Vector2d(-21.0, 58.5), Math.toRadians(0.0))
                .back(60)
                .lineTo(new Vector2d(48,34))
                // .lineTo(new Vector2d(0,36))
                .build();


        switch (recordedPropPosition) {
            case LEFT:
                robot.driveTrain.followTrajectorySequence(left);
                break;

            case MIDDLE:
                robot.driveTrain.followTrajectorySequence(middle);
                break;
            case RIGHT:
                robot.driveTrain.followTrajectorySequence(right);
                break;
            case UNFOUND:
                telemetry.addLine("Unfound");
        }

    }

    @Override
    public void loop(){

        telemetryAprilTag();

        propDetect.close();

    }
    @Override
    public void stop(){


    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = atagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}
