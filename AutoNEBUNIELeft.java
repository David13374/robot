package org.firstinspires.ftc.teamcode.drive.Auton;


import static org.firstinspires.ftc.teamcode.drive.Auton.AutoNEBUNIELeft.State.Going_to_stack;
import static org.firstinspires.ftc.teamcode.drive.Auton.AutoNEBUNIELeft.State.Initial_Traj;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoNEBUNIELeft extends LinearOpMode {
    Lift lift;
    SampleMecanumDrive drive;
    Claw claw;
    GetDetection detec = new GetDetection();

    Pose2d startPose = new Pose2d(-35.56, -62.67, Math.toRadians(90.00));

    enum State {
        Initial_Traj,
        Going_to_stack
    }

    State currentState=Initial_Traj;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        detec.initCamera(hardwareMap);

        drive.setPoseEstimate(startPose);
        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-35.56, -62.67, Math.toRadians(90.00)))
                .setTurnConstraint(2.5, Math.toRadians(60)/2)

                .lineTo(new Vector2d(-35.5, -59))
                .lineToLinearHeading(new Pose2d(-35.56, -60, Math.toRadians(0)))
                .lineTo(new Vector2d(-38, 0))
                .lineTo(new Vector2d(-38, -5))
                .lineToLinearHeading(new Pose2d(-33, -6.14, Math.toRadians(70)))
                .addTemporalMarker(3.5, () -> {
                    lift.setPosition(1025);
                })
                .lineTo(new Vector2d(-29, -2.14))
                .addTemporalMarker(5, () -> {
                    claw.open();
                })
                .waitSeconds(0.5)
                .build();
        TrajectorySequence toStack = drive.trajectorySequenceBuilder(untitled0.end())
                .lineToLinearHeading(new Pose2d(-36.18, -9.35, Math.toRadians(180)))
                .addTemporalMarker(0.2, () -> {
                    lift.setPosition(162);
                })
                .lineTo(new Vector2d(-58, -9.35))
                .build();


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
            detec.Detect();
        }

        lift.setPosition(100);
        drive.followTrajectorySequenceAsync(untitled0);
        while (opModeIsActive()) {

            switch(currentState) {
                case Initial_Traj: {
                    if(!drive.isBusy()) { currentState = Going_to_stack; drive.followTrajectorySequenceAsync(toStack); }
                }
            }




            lift.update();
            drive.update();
        }

    }

}