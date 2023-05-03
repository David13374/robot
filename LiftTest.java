package org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.TestOpModes;

import android.text.method.Touch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.Lift;

@Config
@TeleOp
public class LiftTest extends LinearOpMode {
    public static int target=0;
    public static double kG=0.01;
    public static double Servo_pos=0;

    DcMotorEx Arm_Lift;
    TouchSensor limit_switch;
    PIDFController controller;
    //Servo Arm_Servo;
    //ColorRangeSensor color_claw;
    public static PIDCoefficients amog= new PIDCoefficients (0.014, 0.00095, 0.0002);
    double offset=0;
    public static double clawvalue=0.2;
    ElapsedTime amoga = new ElapsedTime();
    double lasttarget=target;
    public static double maxvel=0.1, maxaccel=0.1, maxjerk=0.1;
    public static int idle_target=150;
    MotionProfile profile;

    @Override
    public void runOpMode() {
        ElapsedTime elapsedTime = new ElapsedTime();
        Arm_Lift = hardwareMap.get(DcMotorEx.class, "domnedomne");
        Arm_Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm_Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limit_switch = hardwareMap.get(TouchSensor.class, "limit_switch");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDFController(amog, 0, 0, 0, (x, v) -> kG);
        Claw claw = new Claw(hardwareMap);

        elapsedTime.startTime();
        waitForStart();
        while(opModeIsActive()) {
            if(limit_switch.isPressed()) offset=Arm_Lift.getCurrentPosition();
            /*if(target!=lasttarget) {
                elapsedTime.reset();
                profile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(Arm_Lift.getCurrentPosition(), 0, 0),
                        new MotionState(target, 0, 0),
                        maxvel,
                        maxaccel,
                        maxjerk
                );}
            try{
                MotionState state = profile.get(elapsedTime.milliseconds());
                controller.setTargetPosition(state.getX());
                controller.setTargetVelocity(state.getV());
                controller.setTargetAcceleration(state.getA());
            }
            catch (NullPointerException exception) {

            }*/

            controller.setTargetPosition(target+offset);
            double correction = controller.update(Arm_Lift.getCurrentPosition());
            Arm_Lift.setPower(correction);
            amoga.startTime();
            if(claw.isCone()) {
                claw.setClaw(clawvalue);
                if(amoga.seconds()>0.5) {target=idle_target;};
            }
            else {claw.setClaw(Servo_pos); amoga.reset();};

            telemetry.addData("target", target);
            telemetry.addData("current", Arm_Lift.getCurrentPosition()+offset);
            telemetry.addData("power", Arm_Lift.getPower());
            telemetry.addData("p, i, d", amog);
            telemetry.addData("blue", claw.getColorBlue());
            telemetry.addData("dist", claw.getDist());
            telemetry.addData("iscone", claw.isCone());
            telemetry.addData("timer", amoga);
            telemetry.update();

            lasttarget=target;
        }
    }
}
