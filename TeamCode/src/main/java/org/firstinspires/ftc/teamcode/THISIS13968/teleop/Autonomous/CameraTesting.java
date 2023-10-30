package org.firstinspires.ftc.teamcode.THISIS13968.teleop.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.THISIS13968.Camera.Camera;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="Camera Testing", group="Auto Tests")
public class CameraTesting extends LinearOpMode {

    Robot13968 robot;
    Camera cam;
    @Override
    public void runOpMode(){
        robot = Robot13968.resetInstance(); //resets bot
        robot.init(hardwareMap,true,  true);
        robot.setDetectColor(Robot13968.DetectColor.BLUE);
        cam = robot.camera;
        waitForStart();
        if(isStopRequested()) return;


    }
}
