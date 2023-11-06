package org.firstinspires.ftc.teamcode.THISIS13968.subsystems.Wheel;



import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.roadrunnertuningfiles.SampleMecanumDrive;

public class Intaketm extends SubsystemBase {

    Robot13968 robot;

    public Intaketm(HardwareMap hwMap) {
        robot = Robot13968.getInstance();
        robot.intake = new Motor(hwMap, "intake", Motor.GoBILDA.RPM_1150);
        robot.intake.setInverted(true);
        robot.intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void run() {
        robot.intake.set(0.2);
    }

    public void stop() {
        robot.intake.set(0);
    }

    public void runInverted() {
        robot.intake.set(-0.2);
    }

}
