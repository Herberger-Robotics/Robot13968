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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.THISIS13968.Camera.TeamPropDetectPipeline;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.DriveConstants101;
import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="thisisdrive", group="Iterative Opmode")

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

    double spinForward;

    //self explanatory
    private VisionPortal visionPortal;
    private TeamPropDetectPipeline propDetect;
    private AprilTagProcessor atagProcessor;
    private DcMotorEx armLeft, armRight;


    public static PIDController pidfController = new PIDController(0.056,0,0.001);

    double f = 0.01;

    double ticks_in_degrees= DriveConstants101.TICKS_PER_REV/180;


    private double armTarget =0;

    //Servo leftClaw, rightClaw, ramp, plane;

    double armup = 0; //initializes amount the arm goes up manually to 0

    double servoTest=0;
    double initPos;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = Robot13968.resetInstance(); //resets bot

        robot.init(hardwareMap); //initializes robot for manual driving with imu
        this.armLeft = robot.driveTrain.armLeft;
        this.armRight = robot.driveTrain.armRight;
      //  plane = hardwareMap.get(Servo.class, "launch");


       armRight.setZeroPowerBehavior(BRAKE);

        armLeft.setZeroPowerBehavior(BRAKE);

        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       robot.driveTrain.setZeroPowerBehavior(BRAKE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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

         spinForward=0;
         //plane.setPosition(1);
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

        //runArm();
       // ramp.setPosition()
        telemetry.update();

/*
        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new Spin(robot.intaketm,runtime, Spin.Side.BLUE));
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new Spin(robot.intaketm,runtime, Spin.Side.RED));

*/

    }


    public void driveTrainController() {

        //  intake is driverOp left stick - X based
        //claws are bumper/trigger on toolOp
        //arm will be right stick toolOp

        boolean leftBumperState = driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumperState = driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);



        boolean isdriverB = driverOp.getButton(GamepadKeys.Button.B); //if B is pressed on tool gamepad, true, else false
        boolean isdriverA = driverOp.getButton(GamepadKeys.Button.A); //if A is pressed on tool gamepad, true, else false

        boolean isdriverX = driverOp.getButton(GamepadKeys.Button.X); //if B is pressed on tool gamepad, true, else false
        boolean isdriverY =driverOp.getButton(GamepadKeys.Button.Y); //if A is pressed on tool gamepad, true, else false


        //figures out whether claws should be open/close
            boolean isRT;
            boolean  isLT;
        boolean isRB = toolOp.getButton(GamepadKeys.Button.RIGHT_BUMPER); //if B is pressed on tool gamepad, true, else false
        boolean isLB = toolOp.getButton(GamepadKeys.Button.LEFT_BUMPER); //if B is pressed on tool gamepad, true, else false

        if  (toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.2){ isRT = TRUE;}
        else { isRT = FALSE;}
        if  (toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.2){ isLT = TRUE;}
        else { isLT = FALSE;}

        double leftTrigger = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        //decimal amt (0 to 1) that right trigger (in back) of tool op is pressed
        double rightTrigger = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

// spins intake with driver buttons Y and A
        if (isdriverY){spinForward = 1;}
            else if (isdriverA){spinForward = -0.7;}
            else {spinForward = 0;}
        robot.driveTrain.intake.setPower(spinForward);

            //brings up viper slide with toolop using right stick
        armLeft.setPower(toolOp.getRightY());
        armRight.setPower(toolOp.getRightY());

        //uses left and right trigger/bumper for open/close of claw
       /* if (isLT) {leftClaw.setPosition(0);//open
            }

        if (isLB) {leftClaw.setPosition(0.1);//close
            }

        if (isRT) {rightClaw.setPosition(0.185);//open
       }

        if (isRB) {rightClaw.setPosition(0.1);//close
          }*/
        boolean upArrTool = toolOp.getButton(GamepadKeys.Button.DPAD_UP);
        boolean  downArrTool= toolOp.getButton(GamepadKeys.Button.DPAD_DOWN);

      /*  if (upArrTool) {ramp.setPosition(0.65);//up
        }
        if (downArrTool){ramp.setPosition(0.55);}

        if (driverOp.getButton(GamepadKeys.Button.DPAD_UP)){plane.setPosition(0);}*/




        /*
        Sets a gradient slow down function for the bot's driving: the more right trigger is pressed,
        the slower the speed is
        */
        double speed;
        if(rightTrigger > 0.9) {speed=0.85;}
        else if(rightTrigger>0.75) {speed=0.80;}
        else if(rightTrigger>0.6) {speed=0.75;}
        else if(rightTrigger>0.45) {speed=0.7;}
        else if(rightTrigger>0.3) {speed=0.65;}
        else{ speed=0.6;}




        double strafe; //init strafe val
        double turn = driverOp.getRightX() * speed; // sets turn val to right joystick (driver gamepad) horizontal amt
        double forward = driverOp.getLeftY() * speed; // sets forward val to left joystick (driver gamepad) vertical amt



        if (Math.abs(driverOp.getLeftX()) > 0.2) {  // makes strafing less sensitive  to accidental movements on triggers
            strafe = driverOp.getLeftX() * (-1) * speed; //sets strafe  to left joystick (driver gamepad) horizontal amt
        }
        else {strafe = 0;}
        //arm.setPower(0.6*toolOp.getRightY());

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

        robot.driveTrain.intake.setPower(0);

        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armRight.setPower(0);
        armLeft.setPower(0);
    }


}