package org.firstinspires.ftc.teamcode.THISIS13968.teleop.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.THISIS13968.Camera.TSEDetectorPipeline;
import org.firstinspires.ftc.teamcode.THISIS13968.Camera.TeamPropDetectPipeline;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;



@Config
@Autonomous(name="Camera Testing", group="Autonomous")
public class CameraTest extends OpMode {
    Robot13968 robot;

    private VisionPortal visionPortal;
    private TeamPropDetectPipeline propDetect;
    private AprilTagProcessor atagProcessor;
    @Override
    public void init() {
        robot = Robot13968.resetInstance(); //resets bot

        robot.init(hardwareMap, true); //initializes robot for manual driving with imu

        //Gamepad Initialization

        // Tell the driver that initialization is complete.]
        telemetry.addData("Status", "Initialized");
        robot.setDetectColor(Robot13968.DetectColor.RED);
        telemetry.addData("Color Detection Mode", robot.getDetectColor());


        double minArea = 200; // the minimum area for the detection to consider for your prop
       // controller = new PIDController(p,i,d);
        atagProcessor = new AprilTagProcessor.Builder().build();
        propDetect = new TeamPropDetectPipeline(
            robot.getDetectColor(),
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame

        );
                //new TSEDetectorPipeline(Robot13968.DetectColor.RED);


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera")) // the camera on your robot is named "Webcam 1" by default
                .addProcessors(atagProcessor, propDetect)
                .build();

    }

    @Override
    public void init_loop() {
        //telemetry.addData("Camera State", visionPortal.getCameraState());
        //telemetry.addData("Currently Detected Mass Area", propProcessor.getLargestContourArea());


    }
    @Override
    public void loop(){
        telemetry.addData("Currently Recorded Position", propDetect.getRecordedPropPosition());

        telemetryAprilTag();

        propDetect.close();

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
