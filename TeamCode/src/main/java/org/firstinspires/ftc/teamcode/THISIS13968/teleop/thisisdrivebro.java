/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.THISIS13968.teleop;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.THISIS13968.Camera.TeamPropDetectPipeline;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="thisisdrivebro", group="Iterative Opmode")

public class thisisdrivebro extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Robot13968 robot;

    //  declare gamepads
    GamepadEx driverOp = null;
    GamepadEx toolOp = null;
    double currentVelocity = 0;
    double setPoint = 0;

    boolean aButtonHeld = false;
    boolean triggerHeld = false;

    //self explanatory
    private VisionPortal visionPortal;
    private TeamPropDetectPipeline propDetect;
    private AprilTagProcessor atagProcessor;

    double armup = 0; //initializes amount the arm goes up manually to 0
    public static int REDLOW = 0;
    public static int REDHIGH = 20;
    public static int REDLOW_NEW = 0;
    public static int REDHIGH_NEW = 20;
    private double minArea = 200;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = Robot13968.resetInstance(); //resets bot

        robot.init(hardwareMap); //initializes robot for manual driving with imu

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Gamepad Initialization
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);


        // Tell the driver that initialization is complete.
        //telemetry.addData("Calibration Status", robot.imu.getSystemStatus());

        telemetry.addData("REDLOW INIT", REDLOW);
        telemetry.addData("REDHIGH INIT", REDHIGH);
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        boolean isB = driverOp.getButton(GamepadKeys.Button.B); //if B is pressed on tool gamepad, true, else false
        boolean isA = driverOp.getButton(GamepadKeys.Button.A); //if A is pressed on tool gamepad, true, else false

        boolean isX = driverOp.getButton(GamepadKeys.Button.X); //if B is pressed on tool gamepad, true, else false
        boolean isY = driverOp.getButton(GamepadKeys.Button.Y); //if A is pressed on tool gamepad, true, else false



        //telemetry sends info to driver station phone, just for testing purposes

    }



    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset(); //resets time elapsed to 0
        //robot.imu.startAccelerationIntegration(null, null , 100 );
       // if (robot.driveTrain.getDriveMode() == DriveTrain101.DriveMode.MANUAL)


        //CommandScheduler.getInstance().schedule(basicDrive);
        //CommandScheduler.getInstance().schedule(manualTurretController);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        CommandScheduler.getInstance().run(); //calls subsystem periodic methods
        driveTrainController(); //just the driving function below
/*
        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new Spin(robot.intaketm,runtime, Spin.Side.BLUE));
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new Spin(robot.intaketm,runtime, Spin.Side.RED));

*/

    }


    public void driveTrainController() {


        boolean leftBumperState = driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumperState = driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);


        boolean isB = driverOp.getButton(GamepadKeys.Button.B); //if B is pressed on tool gamepad, true, else false
        boolean isA = driverOp.getButton(GamepadKeys.Button.A); //if A is pressed on tool gamepad, true, else false

        boolean isX = driverOp.getButton(GamepadKeys.Button.X); //if B is pressed on tool gamepad, true, else false
        boolean isY =driverOp.getButton(GamepadKeys.Button.Y); //if A is pressed on tool gamepad, true, else false

        double leftTrigger = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        //decimal amt (0 to 1) that right trigger (in back) of tool op is pressed
        double rightTrigger = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        double spinForward = toolOp.getLeftX()*1.2;
        robot.driveTrain.intake.setPower(spinForward);

        /*
        Sets a gradient slow down function for the bot's driving: the more right trigger is pressed,
        the slower the speed is
        */
        double speed;
        if(rightTrigger > 0.9) {speed=0.2;}
        else if(rightTrigger>0.75) {speed=0.3;}
        else if(rightTrigger>0.6) {speed=0.4;}
        else if(rightTrigger>0.45) {speed=0.5;}
        else if(rightTrigger>0.3) {speed=0.6;}
        else if(rightTrigger>0.15) {speed=0.7;}
        else{ speed=0.8;}




        double strafe; //init strafe val
        double turn = driverOp.getRightX() * speed; // sets turn val to right joystick (driver gamepad) horizontal amt
        double forward = driverOp.getLeftY() * speed; // sets forward val to left joystick (driver gamepad) vertical amt



        if (Math.abs(driverOp.getLeftX()) > 0.2) {  // makes strafing less sensitive  to accidental movements on triggers
            strafe = driverOp.getLeftX() * (-1) * speed; //sets strafe  to left joystick (driver gamepad) horizontal amt
        }
        else {strafe = 0;}

        robot.driveTrain.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        strafe,
                        -turn
                )
        );


    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        //resets vals to 0, initial position
        Robot13968 robot = Robot13968.getInstance();
       // robot.intaketm.stop();
       // robot.imu.stopAccelerationIntegration();
        robot.driveTrain.setMotorPowers(0,0,0,0);
    }


}