package org.firstinspires.ftc.teamcode.THISIS13968.subsystems.Wheel;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;

import java.util.concurrent.TimeUnit;

import javax.annotation.meta.When;
/*
public class Spin extends CommandBase {

    Intaketm wheel;
    ElapsedTime elapsedTime;
    double startTime;

    public enum Side {
        BLUE,
        RED
    }

    Side side;


    public Spin(Intaketm Wheel, ElapsedTime elapsedTime, Side side)  {
        this.wheel = Wheel;
        this.elapsedTime = elapsedTime;
        this.side = side;
    }

    @Override
    public void initialize() {
        Robot13968 robot = Robot13968.getInstance();

        startTime = elapsedTime.time(TimeUnit.SECONDS);
    }

    @Override
    public void execute() {
        Robot13968 robot = Robot13968.getInstance();


        switch (side) {
            case BLUE: robot.intake.set(-0.25); break;
            case RED: robot.intake.set(0.25); break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot13968 robot = Robot13968.getInstance();

        robot.intake.set(0);
    }

*/
