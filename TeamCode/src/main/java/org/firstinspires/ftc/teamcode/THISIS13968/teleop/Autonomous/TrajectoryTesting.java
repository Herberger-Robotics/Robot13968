package org.firstinspires.ftc.teamcode.THISIS13968.teleop.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain.DriveTrain90;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Vector;


@Config
@Autonomous(name="Trajectory Testing", group="Auto Tests")
public class TrajectoryTesting extends LinearOpMode {

    Robot13968 robot;
    @Override
    public void runOpMode(){
        robot = Robot13968.resetInstance(); //resets bot
        robot.init(hardwareMap,true);
        robot.setDetectColor(Robot13968.DetectColor.RED);
        waitForStart();
        if(isStopRequested()) return;

       /* Trajectory myTraj  = robot.driveTrain.trajectoryBuilder(new Pose2d())
                .forward(10) //clearly NOT measured in inches
                .build();*/
        Pose2d startPose = new Pose2d(-33.89, 63.75, Math.toRadians(270.00));
        TrajectorySequence left =  robot.driveTrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-28.89, 38.0), Math.toRadians(-45))
                .setReversed(true)
                .splineTo( new Vector2d(-27.0, 58.5), Math.toRadians(0.0))
                .back(62)
                .lineTo(new Vector2d(48,34))
                //  .lineTo(new Vector2d(0,36))
                .build();
       /* Trajectory myTraj  = robot.driveTrain.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(20,20),Math.toRadians(0)) //clearly NOT measured in inches
                .build();
*/
        /*Trajectory myTraj2  = robot.driveTrain.trajectoryBuilder(new Pose2d())
            .strafeRight(2)
                .build();*/
        robot.driveTrain.followTrajectorySequence(left);

        //robot.driveTrain.followTrajectory(myTraj2);


    }
}
