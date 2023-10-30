package org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.THISIS13968.Camera.Camera;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain.DriveTrain90;
import org.firstinspires.ftc.teamcode.roadrunnertuningfiles.DriveConstants;

import java.util.List;



    public class Robot13968 {

        // Static variable reference of single_instance
        // of type Singleton
        //
        private static org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968 single_instance = null; //declares an instance of the robot

        HardwareMap hwMap = null; //declare hardware map
        private ElapsedTime period = new ElapsedTime(); //new  time tracking variable, for auto

        public IMU imu; //declare imu
        //public Camera camera = null;

        //drivetrain
        public DcMotorEx rightFront = null;
        public DcMotorEx leftFront = null;
        public DcMotorEx rightBack = null;
        public DcMotorEx leftBack = null;
        public DriveTrain90 driveTrain = null;
        public Camera camera;
        public DetectColor detectColor = DetectColor.BLUE; //default
        public enum DetectColor {
            BLUE,
            RED

        }

        public void setDetectColor(DetectColor color){

            detectColor = color;
        }
        public DetectColor getDetectColor(DetectColor color){

            return detectColor;
        }


    // Constructor
        // Here we will be creating private constructor
        // restricted to this class itself
        private Robot13968()
        {

        }

        // Static method
        // Static method to create instance of Singleton class
        public static Robot13968 getInstance()
        {
            if (single_instance == null)
                single_instance = new Robot13968();

            return single_instance;
        }

        public static Robot13968 resetInstance()
        { // resets robot
                single_instance = new Robot13968();
                return single_instance;
        }
    public void clearBulkCache() {
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

        for(LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
        public void init(HardwareMap ahwMap,boolean initIMU, boolean initCamera)
        {

            /*
            initiallizes robot with hardware map, drive mode,
            and whether or not there is an IMU; expansion hub has built in imu
            */
        CommandScheduler.getInstance().reset(); //reset command scheduler
            hwMap = ahwMap;
            //camera = new Camera(hwMap);
            driveTrain = new DriveTrain90(hwMap); //init drive train
                //????
            List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

            for(LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            if(initIMU) {
                imu = hwMap.get(IMU.class, "imu");
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
                imu.initialize(parameters);
            }
            if(initCamera) {

                  camera = new Camera(hwMap);
            }
        }
}
