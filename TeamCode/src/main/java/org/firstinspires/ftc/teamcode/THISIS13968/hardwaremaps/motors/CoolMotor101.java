package org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.motors;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
// functions for the motors which control the wheels and the arm
public class CoolMotor101 extends Motor {
    private DcMotorEx m_motor; //declare motor
    private double resetVal;

    public static double TICKS_PER_REV; // final constant holding the number of ticks per rev of wheel

    private String name;
    public String getName() { return name; }

    public CoolMotor101(HardwareMap hMap, String name, double TPR) {
        m_motor = hMap.get(DcMotorEx.class, name); // get motor from hardware map config
        TICKS_PER_REV = TPR; //set ticks per revolution to motor TPR val
        this.name = name;
    }

    public void set(double speed) {
        m_motor.setPower(speed);
    } //sets speed of motor

    public double get() {
        return m_motor.getPower();
    } // returns motor speed

    public void setInverted(boolean isInverted) {
        //reverses motor if bool is true, else sets to false
        m_motor.setDirection(!isInverted ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
    }

    //returns whether or not motor is reversed
    public boolean getInverted() {
        return m_motor.getDirection() == DcMotor.Direction.REVERSE;
    }

    //turns off motor
    public void disable() {
        m_motor.close();
    }


    public String getDeviceType() {
        return null;
    }

    public void pidWrite(double output) {
        set(output);
    }

    public void stopMotor() {
        set(0);
    }

    public void setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        m_motor.setZeroPowerBehavior(behavior);
    }

    public double getEncoderCount() {
        return m_motor.getCurrentPosition();
    } //returns encoder val

    public void resetEncoder() {
        //resets encoder
        m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    //
    public double getNumRevolutions() {
        //calculates num of revolutions
        return getEncoderCount() / TICKS_PER_REV;
    }

    public void setTarget(int target){m_motor.setTargetPosition(target);}

    public boolean busy(){return m_motor.isBusy(); }

    public void runToPosition(){m_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);} // allows motor to go to target position

    public void runUsingEncoder(){m_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);} //runs motor using encoders

    public void runWithoutEncoder(){m_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}


    public double getVelocity() {
        return m_motor.getVelocity();
    } //returns velocity of motor


    public void setVelocity(double velocity) {
        m_motor.setVelocity(velocity);
    }
}