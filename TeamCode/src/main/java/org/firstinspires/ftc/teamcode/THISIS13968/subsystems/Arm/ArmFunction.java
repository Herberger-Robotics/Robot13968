package org.firstinspires.ftc.teamcode.THISIS13968.subsystems.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.DriveConstants101;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;

import java.util.concurrent.TimeUnit;
/*
public class ArmFunction extends CommandBase {

    DcMotorEx arm;

    double armTarget;

    ElapsedTime elapsedTime;


    double startTime;
    double integral = 0;
    double ticks_in_degrees= DriveConstants101.TICKS_PER_REV/180;
    PIDController pidfController = new PIDController(0.056,0,0.001);
            double f=0.01;


    public ArmFunction(DcMotorEx Arm, ElapsedTime elapsedTime, double height)  {
        this.arm = Arm;
        this.elapsedTime = new ElapsedTime();
        this.armTarget = height;
    }

    @Override
    public void initialize() {
        Robot13968 robot = Robot13968.getInstance();
        elapsedTime.reset();

    }

    @Override
    public void execute() {
        Robot13968 robot = Robot13968.getInstance();
        double error = arm.getCurrentPosition();
        double lastError = 0;
        double timePassed;

        while(Math.abs(error)>9 ){
            double curr_pos = arm.getCurrentPosition();
            double pid = pidfController.calculate(curr_pos,armTarget);
            double ff = Math.cos(Math.toRadians(armTarget/ticks_in_degrees))*f;
            double power = pid+ff;
            arm.setPower(power);
        }

    }

    @Override
    public void end(boolean interrupted) {
        Robot13968 robot = Robot13968.getInstance();

        robot.driveTrain.arm.setPower(0);
    }

*/


