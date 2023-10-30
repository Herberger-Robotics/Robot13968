package org.firstinspires.ftc.teamcode.THISIS13968.Camera;

import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;


import org.openftc.easyopencv.OpenCvCameraFactory;


public class Camera {
    public OpenCvCamera controlHubCamera;
    public VisionPortal visionPortal;

    public AprilTagProcessor atagProcessor;
    private static int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    // Calculate the distance using the formula

    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
    public TSEDetectorPipeline pipeline;
//
    public Camera(final HardwareMap hwMap)
    {

        Robot13968 robot = Robot13968.getInstance();

        atagProcessor = new AprilTagProcessor.Builder().build();
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        controlHubCamera= OpenCvCameraFactory.getInstance().createWebcam(
                hwMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        pipeline = new TSEDetectorPipeline(robot.detectColor );
        controlHubCamera.setPipeline(pipeline);
//
//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
        controlHubCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
       {
            @Override
            public void onOpened()
            {

                controlHubCamera.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);


            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
    public TSEPosition getPosition() {
        return pipeline.getObjPosition();
    }
    public TSEDetectorPipeline.Colors getColorDetected() {
        return pipeline.getColorDetected();

    }
    public double[] getColorsInfo() {
        return pipeline.getTelemetryInfo();

    }
    public void  setCameraWidth(int width){
        CAMERA_WIDTH = width;

    }
    public void  setCameraHeight(int height){
        CAMERA_HEIGHT = height;

    }

    public void endCamera()
    {
        controlHubCamera.closeCameraDevice();

    }


}