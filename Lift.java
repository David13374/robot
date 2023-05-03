package org.firstinspires.ftc.teamcode.drive.opmode.Subsystems;

import android.text.method.Touch;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config

public class Lift {

    public enum STATE {
        LOOKING_FOR_ZERO,
        IDLE,
        LOADED,
        RISING,
        FALLING,
        UP;
    }


    public Lift.STATE state;
    public static double p = 0.014, i = 0.001, d = 0.0002;
    public static double kG = 0.;

    public static double offset=0;

    public static double liftlowercap = 0.6;

    public static com.acmerobotics.roadrunner.control.PIDCoefficients coeffs= new PIDCoefficients(p, i ,d);
    public PIDFController controller;


    //private final double ticks_in_degree = 145.1/180;
    public DcMotorEx Arm_Lift;
    public TouchSensor limit_switch;
    private boolean active;


    public Lift(HardwareMap hardwareMap) {

        Arm_Lift = hardwareMap.get(DcMotorEx.class, "domnedomne");

        //controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> kG);
        limit_switch = hardwareMap.get(TouchSensor.class, "limit_switch");
        controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> kG);
        state=STATE.LOOKING_FOR_ZERO;

    }

    public void setPosition(double position) {
        controller.setTargetPosition(position-offset);
        if(position<100) kG=0;
        else kG=0.1;
    }
    public void setPower(double power) {
        Arm_Lift.setPower(power);
    }
    public void startEnc() {
        Arm_Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void pidState(boolean active) {
        this.active = active;
    }

    public boolean getPidState() {
        return active;
    }

    public void initController() {
        controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> kG);
    }

    public void update() {
        double correction = controller.update(getPos());
        if(correction<-liftlowercap) correction=-liftlowercap;
        setPower(correction);
    }

    public void resetEnc() {
        Arm_Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetOffset() {
        offset=getPos();
    }

    public double getVel() {
        return Arm_Lift.getVelocity();
    }



    public void setState(STATE statein) {
        state = statein;
    }

    public STATE getSTATE() {
        return state;
    }
    public boolean limit() {
        return limit_switch.isPressed();
    }

    public double getPos() {
        return (Arm_Lift.getCurrentPosition()-offset);
    }
    public double getPower() {
        return Arm_Lift.getPower();
    }

    public void setLowerCap(double lowerCap) {
        liftlowercap = lowerCap;
    }

    public double getLowerCap() {
        return  liftlowercap;
    }


}
