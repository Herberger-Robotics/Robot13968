package org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.firstinspires.ftc.teamcode.THISIS13968.subsystems.DriveTrain.DriveTrain101;

public class AutoDrive extends CommandBase {

    DriveTrain101 driveTrain;
    double distance;
    double adjustedDistance;
    boolean isSet = false;

    public AutoDrive(DriveTrain101 driveTrain, double distance) {
        Robot13968 robot = Robot13968.getInstance();
        this.driveTrain = driveTrain;
        this.distance = distance;
    }
//
    @Override
    public void initialize() {
        Robot13968 robot = Robot13968.getInstance();
        adjustedDistance = distance;
        driveTrain.setX(distance);
        driveTrain.setDriveMode(DriveTrain101.DriveMode.AUTOFORWARD);
        robot.leftFront.resetEncoder();
    }

    @Override
    public void execute() {
        driveTrain.update();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setDriveMode(DriveTrain101.DriveMode.AUTOIDLE);
    }

    @Override
    public boolean isFinished() {
        Robot13968 robot = Robot13968.getInstance();
        return Math.abs(robot.leftFront.getEncoderCount() - adjustedDistance) < 20;
    }

}
