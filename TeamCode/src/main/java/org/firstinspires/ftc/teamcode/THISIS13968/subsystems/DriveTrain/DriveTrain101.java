package org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.motors.CoolMotor101;


///finished editing in from prev yr
//import org.firstinspires.ftc.teamcode.THISIS10111.hardwaremaps.HowlersHardware;

//import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

public class DriveTrain101 extends SubsystemBase {
    @Config
    public static class DriveTrainConstants {
        public static PIDCoefficients FORWARD_PID = new PIDCoefficients(0.00075,0,0); // trains forward movement
        public static double FORWARD_kStatic = 0.1;
        public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.0025,0,0); //trains angle movement: turning
        public static double HEADING_kStatic = 0.0125;
    }

        //also related to the motor groups, not necessary w Mecanum drive
    //private MotorGroup leftMotors;
    //private MotorGroup rightMotors;
    private MecanumDrive driveTrain;

    // drivemode settings
        private DriveMode driveMode;
        public DriveMode setDriveMode(DriveMode driveMode) {
            this.driveMode = driveMode;
            return this.driveMode;
        }
        public DriveMode getDriveMode() {
        return driveMode;
    }
        public enum DriveMode {
            MANUAL,
                AUTOIDLE,
            AUTOFORWARD,
            AUTOTURN,
            AUTODIAGONAL
         }

    private double speed = 0.6;
    public double setSpeed(double setter) {
        speed = setter;
        return speed;
    }
    public double getSpeed() {
        return speed;
    }

    //variables to get
    public double manualForward = 0;
    private double manualTurn = 0;
    private double manualStrafe = 0;


    double forwardCalculation;
    double turnCalculation;
    double strafeCalculation;


    public PIDFController forwardPID;
    public PIDFController headingPID;
    double targetAngle = 0;
    double targetX = 0;
    Orientation angles;
    Position position;


    public DriveTrain101(final HardwareMap hwMap, DriveMode driveMode) {
        Robot13968 robot = Robot13968.getInstance();
        CommandScheduler.getInstance().registerSubsystem(DriveTrain101.this);

        this.driveMode = driveMode;
        forwardPID = new PIDFController(DriveTrainConstants.FORWARD_PID, 0, 0, DriveTrainConstants.FORWARD_kStatic);
        headingPID = new PIDFController(DriveTrainConstants.HEADING_PID, 0, 0, DriveTrainConstants.HEADING_kStatic);

        robot.rightFront = new CoolMotor101(hwMap, "rightFront", 134.4);
        robot.rightBack = new CoolMotor101(hwMap, "rightBack", 134.4);
        robot.leftBack = new CoolMotor101(hwMap, "leftBack", 134.4);
        robot.leftFront = new CoolMotor101(hwMap, "leftFront", 134.4);
        //robot.leftBack.setInverted(true);
       // robot.leftFront.setInverted(true);
        //robot.rightBack.setInverted(true);
        //robot.rightFront.setInverted(true);
        //leftMotors = new MotorGroup(robot.rightFront, robot.rightBack);
        //rightMotors = new MotorGroup(robot.leftBack, robot.leftFront);

        //driveTrain = new DifferentialDrive(leftMotors, rightMotors);  : only use if using the motor groups
        driveTrain = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack);
        driveTrain.setMaxSpeed(0.8);

    }
    public void setManualDrive(double strafe, double forward, double turn) {
        //manually driving,specifiying amt of strafing, forward, and turn, used in manual teleop to set vals
        manualForward = forward;
        manualTurn = turn;
        manualStrafe = -strafe;
    }

    public void update() {


        Robot13968 robot = Robot13968.getInstance();

        //angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //position = robot.imu.getPosition();
        switch (driveMode) {
            case MANUAL:
                forwardCalculation = manualForward;
                turnCalculation = manualTurn;
                strafeCalculation = manualStrafe;
                break;
            case AUTOFORWARD:
                forwardCalculation = forwardPID.update(robot.leftFront.getEncoderCount());
                turnCalculation = 0;
                strafeCalculation = 0;
                break;
            case AUTODIAGONAL:
            case AUTOTURN:
                forwardCalculation = 0;
                turnCalculation = headingPID.update(currentHeading());
                strafeCalculation = 0;
                break;
            default:
                forwardCalculation = 0;
                turnCalculation = 0;
                strafeCalculation = 0;
                break;
        }
        //
        driveTrain.driveRobotCentric(strafeCalculation, forwardCalculation, turnCalculation);

        //robot.arm.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public double getForwardCalculation(){

        return forwardCalculation;
    }
    public double getStrafeCalculation(){

        return strafeCalculation;
    }
    public double getTurnCalculation(){

        return turnCalculation;
    }
    public double getManualForward(){

        return manualForward;
    }
    public double getManualTurn(){

        return manualTurn;
    }
    public double getManualStrafe(){

        return manualStrafe;
    }
    public void setHeading(double target) {

        // sets where the robot is currently headed : angle
        targetAngle = target;
        headingPID.setTargetPosition(target);
    }

    public double currentHeading() {

        // seems to return a turn angle, not sure what
        Robot13968 robot = Robot13968.getInstance();

        double currentHeading;

       // angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //if(angles.firstAngle <= 0) currentHeading = 360 - Math.abs(angles.firstAngle);
        currentHeading = -angles.firstAngle;

        return currentHeading;
    }
    public void setX(double target) {
        //sets
        this.targetX = target;
        forwardPID.setTargetPosition(target);

    }

    //@Override
    public void periodic() {

        update();
        //System.out.println("new update");
    }

    public void resetEncoders() {
        Robot13968 robot = Robot13968.getInstance();

        robot.leftBack.resetEncoder();
        robot.rightBack.resetEncoder();
        robot.leftFront.resetEncoder();
        robot.rightFront.resetEncoder();

    }

    public void slow() { driveTrain.setMaxSpeed(0.25); }
    public void fast() {
        driveTrain.setMaxSpeed(0.75);
    }


    public void stop()
    {
        Robot13968 robot = Robot13968.getInstance();

        robot.rightBack.set(0);
        robot.leftBack.set(0);
        robot.rightFront.set(0);
        robot.leftFront.set(0);

    }
    public boolean isBusy() {
        Robot13968 robot = Robot13968.getInstance();
        boolean isBusy;
        if(robot.rightFront.busy() && robot.leftBack.busy() && robot.rightBack.busy() && robot.leftFront.busy()) isBusy = true;
        else isBusy = false;
        return isBusy;
    }

}