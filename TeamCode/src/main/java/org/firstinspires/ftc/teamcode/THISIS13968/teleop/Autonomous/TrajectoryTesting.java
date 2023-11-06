package org.firstinspires.ftc.teamcode.THISIS13968.teleop.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.roadrunnertuningfiles.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(name="Trajectory Testing", group="Auto Tests")
public class TrajectoryTesting extends LinearOpMode {

    Robot13968 robot;
    public static double INIT_X = 0;
    public static double INIT_Y = 0;
    public static double HEADING = 0;

    public static double DISTANCE = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        /*robot = Robot13968.resetInstance(); //resets bot
        robot.init(hardwareMap);
        robot.setDetectColor(Robot13968.DetectColor.RED);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d rightStartPose = new Pose2d(INIT_X, INIT_Y, Math.toRadians(HEADING));
         rightStartPose = new Pose2d();
        TrajectorySequence left =  robot.driveTrain.trajectorySequenceBuilder(rightStartPose)
                .forward(DISTANCE)
                .build();
        waitForStart();
        if(isStopRequested()) return;/*

         */


       /* Trajectory myTraj  = robot.driveTrain.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(20,20),Math.toRadians(0)) //clearly NOT measured in inches
                .build();
*/
        /*Trajectory myTraj2  = robot.driveTrain.trajectoryBuilder(new Pose2d())
            .strafeRight(2)
                .build();*/
        // robot.driveTrain.followTrajectorySequence(left);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();


        while (!isStopRequested() && opModeIsActive()) {

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
        //robot.driveTrain.followTrajectory(myTraj2);


    }
}
