package org.firstinspires.ftc.teamcode.drive.opmode.Subsystems.TestOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous

public class doamnefereste extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(35.74, -62.49, Math.toRadians(90.00)))
                .splineTo(new Vector2d(34.15, -32.22), Math.toRadians(137.73))
                .build();

        drive.setPoseEstimate(untitled0.start());
        waitForStart();
        if(isStopRequested()) return;
            drive.followTrajectorySequence(untitled0);
    }
}
