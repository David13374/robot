package org.firstinspires.ftc.teamcode.drive.opmode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class Claw
{
    public enum state {
        OPEN,
        CLOSED,
        INIT;
    }
    public Servo Arm_Servo;
    public ColorRangeSensor claw_color;
    public DistanceSensor claw_distance;
    public state claw_state;

    public static double closed_val = 0.15;
    public static double open_val = 0;

    public Claw(HardwareMap hardwaremap) {
        Arm_Servo = hardwaremap.get(Servo.class, "Arm_Servo");
        claw_color = hardwaremap.get(ColorRangeSensor.class, "claw_color");
        //claw_distance = hardwaremap.get(DistanceSensor.class, "claw_color");
        claw_state = state.INIT;
    }

    public double getDist() {
        return claw_color.getDistance(DistanceUnit.CM);
    }

    public double getColorBlue() {
        return claw_color.blue();
    }

    public void setClaw(double pos) {
        Arm_Servo.setPosition(pos);
    }

    public void close() {
        claw_state = state.CLOSED;
        setClaw(closed_val);
    }
    public void open() {
        claw_state = state.OPEN;
        setClaw(open_val);
    }

    public state getState() {
        return claw_state;
    }

    public boolean isCone() {
        if(getColorBlue() > 250 && getDist()<6) return true;
        else return false;
    }


}

