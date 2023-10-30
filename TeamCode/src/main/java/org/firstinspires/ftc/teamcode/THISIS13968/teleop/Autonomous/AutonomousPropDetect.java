package org.firstinspires.ftc.teamcode.THISIS13968.teleop.Autonomous;
/*

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.THISIS13968.Camera.TeamPropDetectPipeline;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Config
@Autonomous(name="AutonomousPropDetect", group="AutoDetection")
public class AutonomousPropDetect extends OpMode {
    Robot13968 robot;

    private VisionPortal visionPortal;
    private TeamPropDetectPipeline propProcessor;
    private AprilTagProcessor atagProcessor;
    @Override
    public void init() {
        robot = Robot13968.resetInstance(); //resets bot

        robot.init(hardwareMap, true); //initializes robot for manual driving with imu

        //Gamepad Initialization

        // Tell the driver that initialization is complete.]
        telemetry.addData("Status", "Initialized");
        // values are for blue
        // not consistent at all.
        //Scalar lower = new Scalar(97,100,100);
        //Scalar upper = new Scalar(125,255,255);

        // values are for red
        // very consistent
        Scalar lower = new Scalar(150, 100, 100);
        Scalar upper = new Scalar(180, 255, 255);
        double minArea = 200; // the minimum area for the detection to consider for your prop
       // controller = new PIDController(p,i,d);
        atagProcessor = new AprilTagProcessor.Builder().build();
        propProcessor = new TeamPropDetectPipeline(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera")) // the camera on your robot is named "Webcam 1" by default
                .addProcessors(atagProcessor,propProcessor)
                .build();

    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", propProcessor.getRecordedPropPosition());
        //telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + propProcessor.getLargestContourX() + ", y: " + propProcessor.getLargestContourY());
        //telemetry.addData("Currently Detected Mass Area", propProcessor.getLargestContourArea());
        //telemetryAprilTag();

    }
    

}
*/