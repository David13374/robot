package org.firstinspires.ftc.teamcode.drive.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class AutoRightDump extends LinearOpMode
{
    SampleMecanumDrive drive;

    enum State {
        IDLE,
        TRAJECTORY1,
        TRAJECTORY2,
        TRAJECTORY3,
        WAIT1,
        WAIT2;
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(35.38, -62.49, Math.toRadians(90.00));

    double waitTime1 = 1.5;
    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();
    ElapsedTime startTimer = new ElapsedTime();

    Lift lift;
    Claw claw;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        Trajectory Traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(35.38, -35.38, Math.toRadians(120)))
                .build();

        Trajectory Traj2 = drive.trajectoryBuilder(Traj1.end())
                .lineTo(new Vector2d(30.7, -30.7))
                .build();
        Trajectory Traj3 = drive.trajectoryBuilder(Traj2.end())
                .lineToSplineHeading(new Pose2d(36.44, -35.74, Math.toRadians(90.00)))
                .build();
        drive.setPoseEstimate(startPose);
        while (opModeInInit() && !isStopRequested())
        {
            if (lift.getSTATE() == Lift.STATE.LOOKING_FOR_ZERO) lift.setPower(-0.2);
            if (lift.limit())
            {
                lift.setState(Lift.STATE.IDLE);
                lift.resetEnc();
                lift.resetOffset();
                lift.setPower(0);
                lift.startEnc();
            }
            if(lift.getSTATE() == Lift.STATE.IDLE) {
                lift.setPosition(30);
                claw.close();
                lift.update();
            }
        }


        waitForStart();

        if(isStopRequested()) return;


        if(startTimer.seconds()>=1) {
            currentState = State.TRAJECTORY1;
            drive.followTrajectoryAsync(Traj1);
        }


        while (!isStopRequested() && opModeIsActive()) {
            switch(currentState) {
                case TRAJECTORY1:
                    if(!drive.isBusy()) {
                        currentState = State.WAIT1;
                        lift.setPosition(705);
                        waitTimer1.reset();
                    }
                    break;
                case WAIT1: {
                    if(waitTimer1.seconds()>waitTime1) {
                        currentState = State.TRAJECTORY2;
                        drive.followTrajectoryAsync(Traj2);
                    }
                }

                case TRAJECTORY2: {
                    if(!drive.isBusy()) {
                        currentState = State.WAIT2;
                        waitTimer2.reset();
                    }
                }

                case WAIT2: {
                    if(waitTimer2.seconds()>4) claw.open();
                    if(waitTimer2.seconds()>7) {
                        currentState = State.TRAJECTORY3;
                        drive.followTrajectoryAsync(Traj3);
                    }
                }
                case TRAJECTORY3: {

                }
            }

            drive.update();
            lift.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("starttimer", startTimer.seconds());
            telemetry.update();
        }




    }
}
