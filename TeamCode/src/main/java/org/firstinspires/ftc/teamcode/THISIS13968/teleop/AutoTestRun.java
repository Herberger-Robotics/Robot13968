package org.firstinspires.ftc.teamcode.THISIS13968.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.THISIS13968.Camera.Camera;
import org.firstinspires.ftc.teamcode.THISIS13968.Camera.TSEPosition;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain.AutoDrive;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain.DriveTrain101;

import java.util.concurrent.TimeUnit;

public class AutoTestRun {




    @Autonomous(name="AutoTestRun prev red duck")
    public class RedDuck extends LinearOpMode {

        Robot13968 robot;
        ElapsedTime elapsedTime = new ElapsedTime();
        //Camera camera;
        double duckDistance;

        @Override
        public void runOpMode() {

            robot = Robot13968.resetInstance();
            CommandScheduler.getInstance().reset();
            robot.init(hardwareMap, DriveTrain101.DriveMode.AUTOIDLE,true);

            robot.leftBack.resetEncoder();
            //camera = new Camera(hardwareMap);
            //TSEPosition duckPosition = camera.getPosition();
            //FtcDashboard.getInstance().startCameraStream(camera.phoneCam,0);

            while(!isStarted() && !isStopRequested()) {
                TelemetryPacket packet = new TelemetryPacket();
                robot.clearBulkCache();
                    //sends information on eveerything
                packet.put("Forward Drive Target", robot.driveTrain.forwardPID.getTargetPosition());
                packet.put("Forward Drive Position", robot.leftFront.getEncoderCount());
                packet.put("Heading Target", robot.driveTrain.headingPID.getTargetPosition());
                packet.put("Current Heading",robot.driveTrain.currentHeading());
                packet.put("Elapsed Time(Milliseconds)", elapsedTime.time(TimeUnit.MILLISECONDS));
                packet.put("Drive Train Operating Mode", robot.driveTrain.getDriveMode());
                //packet.put("Duck Position",duckPosition);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                //duckPosition = camera.getPosition();
                //telemetry.addData("Duck Position", duckPosition);
            }

            //FtcDashboard.getInstance().stopCameraStream();

            elapsedTime.reset();
            //camera.endCamera();

            //Duck Position
            /*
            switch(duckPosition) {
                case LEFT:
                    duckHeight = LiftHeight.BOTTOM;
                    duckDistance = 30;
                    break;
                case CENTER:
                    duckHeight = LiftHeight.MIDDLE;
                    duckDistance = 0;
                    break;
                case RIGHT:
                    duckHeight = LiftHeight.TOP;
                    duckDistance = 50;
                    break;
            }*/

            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(

                    new AutoDrive(robot.driveTrain,1700)
                            .withTimeout(3000)
                            /*
                    new GoToHeading(robot.driveTrain,90)
                            .withTimeout(5000),

                    new AutoDrive(robot.driveTrain,duckDistance)
                            .withTimeout(3000),

                    new InstantCommand(robot.liftArm::intakeReversed),
                    new WaitCommand(1000),
                    new InstantCommand(robot.liftArm::stopIntake),

                    new AutoDrive(robot.driveTrain,-1000 - duckDistance)
                            .withTimeout(5000)
                            .alongWith(new Lift(robot.liftArm,LiftHeight.DRIVE)),
                    new GoToHeading(robot.driveTrain,0)
                            .withTimeout(3000),

                    new AutoDrive(robot.driveTrain,-1370)
                            .withTimeout(5000),
                    new GoToHeading(robot.driveTrain, 45)
                            .withTimeout(3000)
                            .alongWith(new InstantCommand(robot.duckWheel::run)),
                    new AutoDrive(robot.driveTrain, -100)
                            .withTimeout(2000),
                    new WaitCommand(6000),
                    new InstantCommand(robot.duckWheel::stop),
                    new GoToHeading(robot.driveTrain,0)
                            .withTimeout(3000),

                    new AutoDrive(robot.driveTrain,600)
                            .withTimeout(3000),
                    new GoToHeading(robot.driveTrain,0)
                            .withTimeout(3000)*/
            ));

            telemetry.addData("Auto Status", "STARTED");
            telemetry.speak("AUTONOMOUS HAS STARTED");

            while(!isStopRequested()) {
                robot.clearBulkCache();
                CommandScheduler.getInstance().run();
                TelemetryPacket packet = new TelemetryPacket();

                packet.put("Forward Drive Target", robot.driveTrain.forwardPID.getTargetPosition());
                packet.put("Forward Drive Position", robot.leftFront.getEncoderCount());
                packet.put("Heading Target", robot.driveTrain.headingPID.getTargetPosition());
                packet.put("Current Heading",robot.driveTrain.currentHeading());/*
                packet.put("liftPID Lift Target", robot.liftArm.getPIDTarget());
                packet.put("LiftArm Lift Position", robot.lift.getEncoderCount());
                packet.put("liftPID Error", robot.liftArm.currentError);*/
                packet.put("Elapsed Time(Milliseconds)", elapsedTime.time(TimeUnit.MILLISECONDS));
                packet.put("Drive Train Operating Mode", robot.driveTrain.getDriveMode());
                //packet.put("Duck Position",duckPosition);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            CommandScheduler.getInstance().cancelAll();
        }
    }


}
