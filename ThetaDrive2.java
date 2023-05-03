package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.Lift;

@TeleOp(name="ThetaDrive2", group="TeleOp")
@Config

public class ThetaDrive2 extends LinearOpMode {
    BNO055IMU imu;
    //double speed = 1000;
    double front_left_motor;
    double back_left_motor;
    double front_right_motor;
    double back_right_motor;
    double gamepadDegrees;
    double theta, sin, cos;
    double power, max, x, y;
    double turn;
    double coef=0.75;
    double robotDegree;
    boolean driverCentric=true;
    double turningScale;
    double gyro_offset = 0;
    double targetpos =0;
    boolean overflow1;
    boolean overflow;

    public static double low_pos = 440, mid_pos=720, high_pos=1025;
    public static int[] stackPos = {0, 10, 50, 85, 125, 162};

    public static boolean manualToggle = false;

    public static double IDLE_POS=10;
    public static boolean isRaiseRequested=false;

    public static int conelevel = 1;


    public static boolean autoPickup=false;

    SampleMecanumDrive robot;
    Lift lift;
    Claw claw;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode()
    {
        //Config();
        lift = new Lift(hardwareMap);
        robot = new SampleMecanumDrive(hardwareMap);
        claw = new Claw(hardwareMap);

        while(!isStarted()&& !isStopRequested()) {
            if(lift.getSTATE()== Lift.STATE.LOOKING_FOR_ZERO) lift.setPower(-0.31);
            if(lift.limit()) {
                lift.setState(Lift.STATE.IDLE);
                lift.resetEnc();
                //lift.resetOffset();
                lift.setPower(0);
            }
            telemetry.addData("aaa", lift.getPower());
            telemetry.addData("limit", lift.limit());
            telemetry.addData("state", lift.getSTATE());
            telemetry.update();
        }

        waitForStart();
        lift.startEnc();
        while (opModeIsActive())
        {
            /*if(front_left_motor.getPower()>1 || front_left_motor.getPower()<-1 || front_right_motor.getPower()>1 || front_right_motor.getPower()<-1 || back_left_motor.getPower()>1 || back_left_motor.getPower()<-1 || back_right_motor.getPower()>1 || back_right_motor.getPower()<-1) {
                overflow1 = true;
                overflow = true;
            }
            */

            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (Exception e) {
            }




            if(front_left_motor>1 || front_left_motor<-1 || front_right_motor>1 || front_right_motor<-1 || back_left_motor>1 || back_left_motor<-1 || back_right_motor>1 || back_right_motor<-1) {
                overflow1 = true;
                overflow = true;
            }
            Move1();
            if(currentGamepad1.b) driverCentric=true;
            else if(currentGamepad1.a) driverCentric=false;

            if(currentGamepad1.right_bumper) coef =0.5;
            else coef=0.75;



            /*if(gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
                lift.pidState(false);
                lift.setPower(gamepad2.right_stick_y+0.15);
            }*/
            /*else {
                if(lift.getPidState()==false) lift.pidState(true);
                if(targetpos!=lasttargetpos) {
                    lift.setPosition(targetpos);
                }
            }*/

            fsm();

            /*if (currentGamepad1.dpad_left) {
                coef = 0.5;
            }
            if (currentGamepad1.dpad_right) {
                coef = 0.75;
            }

            */

            //if(claw.isCone()) claw.setClaw(0.3);

            if(currentGamepad2.left_bumper) claw.open();;
            if(currentGamepad2.right_bumper) claw.close();

            if(currentGamepad2.left_stick_button && !previousGamepad2.left_stick_button) autoPickup=!autoPickup;
            if(currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) manualToggle=!manualToggle;

            if(currentGamepad1.y) reset_gyro();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("conelevel", conelevel);
            telemetry.addData("autopickup" , autoPickup);
            telemetry.addData("manualtog", manualToggle);
            telemetry.addData("IDLE_POS", IDLE_POS);
            telemetry.addData("liftstat", lift.getSTATE());
            telemetry.addData("limit", lift.limit());
            telemetry.addData("offset", lift.offset);
            telemetry.addData("position",lift.getPos());
            telemetry.addData("lift power", lift.getPower());
            telemetry.addData("lift target", targetpos);
            telemetry.addData("coef", coef);
            telemetry.addData("theta", Math.toDegrees(theta));
            telemetry.addData("power", power);
            telemetry.addData("turn", turn);
            telemetry.addData("back_right_motor", back_right_motor);
            telemetry.addData("back_left_motor", back_left_motor);
            telemetry.addData("front_right_motor", front_right_motor);
            telemetry.addData("front_left_motor", front_left_motor);
            telemetry.addData("corrector", power + Math.abs(turn));
            telemetry.addData("turning scale", turningScale);
            telemetry.addData("overflow detected", overflow1);
            telemetry.addData("overflow", overflow);
            telemetry.update();
            overflow=false;
        }
    }

    /*private void Config()
    {
        //rotile
        {
            front_left_motor = hardwareMap.dcMotor.get("front_left_motor");
            front_right_motor = hardwareMap.dcMotor.get("front_right_motor");
            back_left_motor = hardwareMap.dcMotor.get("back_left_motor");
            back_right_motor = hardwareMap.dcMotor.get("back_right_motor");

            front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            front_left_motor.setDirection((DcMotor.Direction.REVERSE));
            front_right_motor.setDirection((DcMotor.Direction.FORWARD));
            back_left_motor.setDirection((DcMotor.Direction.REVERSE));
            back_right_motor.setDirection((DcMotor.Direction.FORWARD));

            front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }

    } */
    private void Move1() {
        x = currentGamepad1.left_stick_x;
        y = -currentGamepad1.left_stick_y;
        turn = Math.max(-currentGamepad1.left_trigger, currentGamepad1.right_trigger)
        + Math.min(-currentGamepad1.left_trigger, currentGamepad1.right_trigger);
        if(turn<-1) turn=-1;


        robotDegree = getAngle();
        gamepadDegrees = Math.atan2(y, x);
        power = Math.hypot(x, y);
        if(power>1) power=1;

        if(driverCentric) theta = gamepadDegrees - robotDegree;
        else theta = gamepadDegrees;

        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));


        front_left_motor = power * cos/max + turn;
        front_right_motor = power * sin/max - turn;
        back_left_motor = power * sin/max + turn;
        back_right_motor = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1 && turn > 0) {
            front_left_motor /= power + turn;
            front_right_motor /= power + turn;
            back_left_motor /= power + turn;
            back_right_motor /= power + turn;
        }

        if ((power + Math.abs(turn)) > 1 && turn < 0) {
            front_left_motor /= power - turn;
            front_right_motor /= power - turn;
            back_left_motor /= power - turn;
            back_right_motor /= power - turn;
        }


        robot.setMotorPowers(coef*front_left_motor, coef*back_left_motor, coef*back_right_motor, coef*front_right_motor);
        //robot.setMotorPowers(0.1, 0.1, 0.1, 0.1);

    }
    /*public double getAngle() {
        return (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - gyro_offset);
    }

    void reset_gyro() {
        gyro_offset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
    */

    private void fsm() {
        switch(lift.getSTATE()){
            case LOOKING_FOR_ZERO: {
                lift.setPower(-0.2);
                if(lift.limit()) {
                    lift.setPower(0);
                    lift.resetOffset();
                    lift.setState(Lift.STATE.IDLE);
                }
            }
            case IDLE: {
                IDLE_POS = stackPos[conelevel];
                lift.setPosition(IDLE_POS);
                lift.update();
                if(claw.isCone()) {
                    if(autoPickup) claw.close();
                    lift.setState(Lift.STATE.LOADED);
                }
            }
            case LOADED: {
                if(currentGamepad2.b) {
                    targetpos = low_pos;
                    onexit();
                }
                if(currentGamepad2.x) {
                    targetpos = mid_pos;
                    onexit();
                }
                if(currentGamepad2.y) {
                    targetpos = high_pos;
                    onexit();
                }

            }
            case FALLING: {
                lift.setLowerCap(0.3);
                if(lift.getPos()<targetpos+10) {
                    isRaiseRequested=false;
                    lift.setState(Lift.STATE.UP);
                    lift.setLowerCap(0.6);
                }
            }

            case RISING: {
                if(lift.getPos()>targetpos-10) {
                    isRaiseRequested=false;
                    lift.setState(Lift.STATE.UP);
                    lift.setLowerCap(0.6);
                }
            }
            case UP: {
                if(currentGamepad2.b) {
                    targetpos = low_pos;
                    if(targetpos < lift.getPos()) onexitlower();
                    else onexit();
                }
                if(currentGamepad2.x) {
                    targetpos = mid_pos;
                    if(targetpos < lift.getPos()) onexitlower();
                    else onexit();
                }
                if(currentGamepad2.y) {
                    targetpos = high_pos;
                    if(targetpos < lift.getPos()) onexitlower();
                    else onexit();
                }

                if(lift.getPos()<170&&!isRaiseRequested) lift.setState(Lift.STATE.IDLE);
                if(currentGamepad2.right_stick_y*-1 !=0 && manualToggle==true) {
                    targetpos=targetpos+currentGamepad2.left_stick_y*-1;
                }

                if(currentGamepad2.a) {
                    lift.setState(Lift.STATE.IDLE);
                    lift.setLowerCap(0.6);
                }
            }

            //default: lift.setState(Lift.STATE.LOOKING_FOR_ZERO);
        }
        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
            if(conelevel != 1) conelevel = conelevel-1;
        }
        if(currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            if(conelevel != 5) conelevel = conelevel+1;
        }
        if(currentGamepad2.dpad_down) conelevel = 1;
        if(lift.getSTATE()!= Lift.STATE.LOOKING_FOR_ZERO) lift.update();
    }

    private double getAngle() {
        return (robot.getExternalHeading() - gyro_offset);
    }

    void onexit() {
        lift.setPosition(targetpos);
        lift.setState(Lift.STATE.RISING);
        isRaiseRequested = true;
    }
    void onexitlower() {
        lift.setPosition(targetpos);
        lift.setState(Lift.STATE.FALLING);
        isRaiseRequested =true;
    }
    void reset_gyro() {
        gyro_offset = robot.getExternalHeading();
    }
}
