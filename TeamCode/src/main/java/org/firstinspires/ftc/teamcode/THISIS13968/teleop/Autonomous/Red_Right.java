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
public class Red_Right extends OpMode {
    Robot13968 robot;

    private VisionPortal visionPortal;
    private TeamPropDetectPipeline propDetect;
    private AprilTagProcessor atagProcessor;
    private double INIT_X = 0;

    private double INIT_Y = 0;

    private double INIT_HEADING = 0;
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


    }
    @Override
    public void start() {

            // gets prop position
        TeamPropDetectPipeline.PropPositions recordedPropPosition = propDetect.getRecordedPropPosition();
        if (recordedPropPosition == UNFOUND) {recordedPropPosition = MIDDLE;}  // a guess

        //initial pose, variable based so this can be edited
        //going to start at startpose 0,0 and only pre-code heading
        Pose2d startPose = new Pose2d(-33.89, 63.75, 0);
        drive.setPoseEstimate(startPose);
            // these trajectories are to be edited and are random, currently all functional/in progress trajectories are in Trajectory Testing
        TrajectorySequence left =  drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-28.89, 38.0), Math.toRadians(-45))
                .setReversed(true)
                .splineTo( new Vector2d(-27.0, 58.5), Math.toRadians(0.0))
                .back(62)
                .lineTo(new Vector2d(48,34))
                //  .lineTo(new Vector2d(0,36))
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-33.89, 32.0))
                .setReversed(true)
                .back(20)
                .splineTo( new Vector2d(-21.0, 58.5), Math.toRadians(0.0))
                .back(60)
                .lineTo(new Vector2d(48,34))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
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
