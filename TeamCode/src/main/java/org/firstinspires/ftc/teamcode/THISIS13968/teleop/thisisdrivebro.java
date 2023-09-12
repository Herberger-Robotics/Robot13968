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

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain.DriveTrain101;

@TeleOp(name="thisisdrivebro", group="Iterative Opmode")

public class thisisdrivebro extends OpMode
{
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
    public enum DriveModes {
        SLOW,
        FAST,
    }

    public DriveModes driveMode = DriveModes.FAST; //initializes driving to fast speed

    double armup = 0; //initializes amount the arm goes up manually to 0


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = Robot13968.resetInstance(); //resets bot

        robot.init(hardwareMap, DriveTrain101.DriveMode.MANUAL, false); //initializes robot for manual driving with imu

        //Gamepad Initialization
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        // Tell the driver that initialization is complete.
        //telemetry.addData("Calibration Status", robot.imu.getSystemStatus());
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
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

        //telemetry sends info to driver station phone, just for testing purposes
        telemetry.addData("Speed", robot.driveTrain.getSpeed());
        //telemetry.addData("Status", robot.driveTrain.getDriveMode());



    }


    public void driveTrainController() {


        boolean leftBumperState = driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumperState = driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);


        boolean isB = toolOp.getButton(GamepadKeys.Button.B); //if B is pressed on tool gamepad, true, else false
        boolean isA = toolOp.getButton(GamepadKeys.Button.A); //if A is pressed on tool gamepad, true, else false

        double leftTrigger = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        //decimal amt (0 to 1) that right trigger (in back) of tool op is pressed
        double rightTrigger = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (isB) {
            //if B is clicked, close claw
            telemetry.addData("B",true);
        }
        if (isA) {
            //if A is clicked, open claw
            telemetry.addData("A",false);
        }

        /*
        Sets a gradient slow down function for the bot's driving: the more right trigger is pressed,
        the slower the speed is
        */

        if(rightTrigger > 0.9) {robot.driveTrain.setSpeed(0.2);}
        else if(rightTrigger>0.75) {robot.driveTrain.setSpeed(0.3);}
        else if(rightTrigger>0.6) {robot.driveTrain.setSpeed(0.4);}
        else if(rightTrigger>0.45) {robot.driveTrain.setSpeed(0.5);}
        else if(rightTrigger>0.3) {robot.driveTrain.setSpeed(0.6);}
        else if(rightTrigger>0.15) {robot.driveTrain.setSpeed(0.7);}
        else{robot.driveTrain.setSpeed(0.8);}


        double speed = robot.driveTrain.getSpeed(); //gets speed fromm drivetrain

        double strafe; //init strafe val
        double turn = driverOp.getRightX() * speed; // sets turn val to right joystick (driver gamepad) horizontal amt
        double forward = driverOp.getLeftY() * speed; // sets forward val to left joystick (driver gamepad) vertical amt



        if (Math.abs(driverOp.getLeftX()) > 0.2) {  // makes strafing less sensitive  to accidental movements on triggers
            strafe = driverOp.getLeftX() * (-1) * speed; //sets strafe  to left joystick (driver gamepad) horizontal amt
        }
        else {strafe = 0;}


        robot.driveTrain.setManualDrive( strafe,  forward,  turn); //uses drivetrain funct (has a drive robot centric funct)

        telemetry.addData("Strafe",strafe );
        telemetry.addData("Forward", forward );
        telemetry.addData("Turn",turn );

        telemetry.addData("Drive Mode", robot.driveTrain.getDriveMode());
        telemetry.addData("Manual Drive", robot.driveTrain.manualForward);
        telemetry.addData("Forward Calculation", robot.driveTrain.getForwardCalculation());

        //telemetry.addData("Robot Position", robot.imu.getPosition());

      //  telemetry.addData("Robot Position", robot.imu.getAngularOrientation());

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        //resets vals to 0, initial position
        Robot13968 robot = Robot13968.getInstance();
       // robot.imu.stopAccelerationIntegration();
        robot.rightBack.set(0);
        robot.leftBack.set(0);
        robot.rightFront.set(0);
        robot.leftFront.set(0);
    }


}