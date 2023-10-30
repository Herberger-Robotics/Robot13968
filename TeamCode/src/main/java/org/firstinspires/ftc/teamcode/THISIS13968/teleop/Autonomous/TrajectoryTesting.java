package org.firstinspires.ftc.teamcode.THISIS13968.teleop.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.THISIS13968.Camera.Camera;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain.DriveTrain90;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Vector;


@Config
@Autonomous(name="Trajectory Testing", group="Auto Tests")
public class TrajectoryTesting extends LinearOpMode {

    Robot13968 robot;
    Camera cam;
    @Override
    public void runOpMode(){
        robot = Robot13968.resetInstance(); //resets bot
        robot.init(hardwareMap,true,  true);
        robot.setDetectColor(Robot13968.DetectColor.RED);
        cam = robot.camera;
        waitForStart();
        if(isStopRequested()) return;

       /* Trajectory myTraj  = robot.driveTrain.trajectoryBuilder(new Pose2d())
                .forward(10) //clearly NOT measured in inches
                .build();*/
        Pose2d startPose  = new Pose2d(0,0,0);
        TrajectorySequence myTraj2  = robot.driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(15,15)) //clearly NOT measured in inches
                .build();
       /* Trajectory myTraj  = robot.driveTrain.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(20,20),Math.toRadians(0)) //clearly NOT measured in inches
                .build();
*/
        /*Trajectory myTraj2  = robot.driveTrain.trajectoryBuilder(new Pose2d())
            .strafeRight(2)
                .build();*/
        robot.driveTrain.followTrajectorySequence(myTraj2);

        //robot.driveTrain.followTrajectory(myTraj2);


    }
}
