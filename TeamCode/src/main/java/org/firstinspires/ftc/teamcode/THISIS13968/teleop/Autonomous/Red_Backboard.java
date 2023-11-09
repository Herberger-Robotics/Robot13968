package org.firstinspires.ftc.teamcode.THISIS13968.teleop.Autonomous;


import static org.firstinspires.ftc.teamcode.THISIS13968.Camera.TeamPropDetectPipeline.PropPositions.MIDDLE;
import static org.firstinspires.ftc.teamcode.THISIS13968.Camera.TeamPropDetectPipeline.PropPositions.UNFOUND;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.THISIS13968.Camera.TeamPropDetectPipeline;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name="Red Right", group="Autonomous")
public class Red_Backboard extends OpMode {
    Robot13968 robot;

    private VisionPortal visionPortal;
    private TeamPropDetectPipeline propDetect;
    private AprilTagProcessor atagProcessor;
    private double INIT_X = 14;

    private double INIT_Y = -61;

    private double INIT_HEADING = 90;
    private SampleMecanumDrive drive;
    @Override
    public void init() {
        //initialization of drivetrain, telemetry, and vision
        robot = Robot13968.resetInstance(); //resets bot
        robot.init(hardwareMap); //initializes robot
        drive= new SampleMecanumDrive(hardwareMap); //this is where the motors live
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.setDetectColor(Robot13968.DetectColor.RED);
        double minArea = 200; // the minimum area for the detection to consider for your prop

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


    }
    @Override
    public void start() {

            // gets prop position
        TeamPropDetectPipeline.PropPositions recordedPropPosition = propDetect.getRecordedPropPosition();
        if (recordedPropPosition == UNFOUND) {recordedPropPosition = MIDDLE;}  // a guess

        Pose2d startPose = new Pose2d(INIT_X, INIT_Y, Math.toRadians(INIT_HEADING)); // 14,-61, 90
        drive.setPoseEstimate(startPose);


        // subject to change
        TrajectorySequence left =  drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(INIT_X-3, INIT_Y+31, Math.toRadians(INIT_HEADING+90)))
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(INIT_X +  00,INIT_Y +28))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(INIT_X +  7,INIT_Y +23))
                .build();


        switch (recordedPropPosition) {
            case LEFT:
                drive.followTrajectorySequence(left);
                break;

            case MIDDLE:
                drive.followTrajectorySequence(middle);
                break;
            case RIGHT:
                drive.followTrajectorySequence(right);
                break;
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
