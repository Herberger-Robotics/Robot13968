package org.firstinspires.ftc.teamcode.roadrunnertuningfiles.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.DriveConstants101;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class ArmTesting extends LinearOpMode {

     ElapsedTime PIDTimer = new ElapsedTime();

    public static double p = 0.056, i = 0, d = 0.001;    //PID gains to be tuned

    public static double f = 0.01;
    public static PIDController pidfController   =  new PIDController(p,i,d);

    public static int armTarget1 = 250;
    double ticks_in_degrees= DriveConstants101.TICKS_PER_REV/180;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

       DcMotorEx arm = drive.arm;

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        if (isStopRequested()) return;



        while (Math.abs(armTarget1 - arm.getCurrentPosition())>10){
            double curr_pos = arm.getCurrentPosition();
            double pid = pidfController.calculate(curr_pos,armTarget1);
            double ff = Math.cos(Math.toRadians(armTarget1/ticks_in_degrees))*f;
            double power = pid+ff;
            arm.setPower(power);
            telemetry.addData("current", curr_pos);
            telemetry.addData("target", curr_pos);
            telemetry.addData("error", armTarget1 - arm.getCurrentPosition());
            telemetry.update();

        }
    }
}
