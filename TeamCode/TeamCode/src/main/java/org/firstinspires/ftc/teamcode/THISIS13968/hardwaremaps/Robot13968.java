package org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.THISIS10111.Camera.Camera;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.motors.CoolMotor101;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain101;

import java.util.List;

public class Robot13968 {

        // Static variable reference of single_instance
        // of type Singleton
    //
        private static Robot13968 single_instance = null; //declares an instance of the robot

        HardwareMap hwMap = null; //declare hardware map
        private ElapsedTime period = new ElapsedTime(); //new  time tracking variable

        public BNO055IMU imu; //declare imu

        //drivetrain
        public CoolMotor101 rightFront = null;
        public CoolMotor101 leftFront = null;
        public CoolMotor101 rightBack = null;
        public CoolMotor101 leftBack = null;



        //public Camera camera = null;


        public DriveTrain101 driveTrain = null;

        private static RunType lastRan = RunType.MANUAL;

    // function to set run type
        public static RunType setLastRan(RunType toSet) {
            lastRan = toSet;
            return lastRan;
        }
        public static RunType getLastRan() {
            return lastRan;
        } //returns the run type

        //possiblities for run type
    public enum RunType {
            AUTONOMOUS,
            MANUAL
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

        public void init(HardwareMap ahwMap, DriveTrain101.DriveMode driveMode,boolean initIMU)
        {

            /*
            initiallizes robot with hardware map, drive mode,
            and whether or not there is an IMU; expansion hub has built in imu
            */
        CommandScheduler.getInstance().reset(); //reset command scheduler
            hwMap = ahwMap;
            //camera = new Camera(hwMap);
            driveTrain = new DriveTrain101(hwMap, driveMode); //init drive train

            List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

            for(LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            if(initIMU) {
                //initializes imu if it exists
                imu = hwMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //gets params of imu from class function
                imu.initialize(parameters);
            }
        }
/*
        public void initDumb(HardwareMap ahwMap, DriveTrain.DriveMode driveMode, boolean initIMU) {

            CommandScheduler.getInstance().reset();
            hwMap = ahwMap;
            driveTrain = new DriveTrain(hwMap, driveMode);
            liftArm = new LiftArm(hwMap);

            List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

            for(LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            if(initIMU) {
                imu = hwMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                imu.initialize(parameters);
            }
        }

        public void clearBulkCache() {
            List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

            for(LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }
*/
}
