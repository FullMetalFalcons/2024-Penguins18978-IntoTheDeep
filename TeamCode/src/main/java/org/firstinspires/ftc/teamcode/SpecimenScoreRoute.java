package org.firstinspires.ftc.teamcode;

// RoadRunner Specific Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
public class SpecimenScoreRoute extends LinearOpMode {
    public void runOpMode() {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        PenguinsArm arm = new PenguinsArm(hardwareMap, telemetry);

        // Close the claw on initialization
        arm.setClawPosition(arm.CLAW_CLOSED);


        Action toBarTrajectory;

        toBarTrajectory = drive.actionBuilder(drive.pose)
                .lineToY(11)
                .build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        toBarTrajectory,
                        arm.armToPosition(arm.ARM_SPECIMEN_READY_DEGREES, arm.SLIDE_SPECIMEN_READY_INCHES),
                        arm.armToPosition(arm.ARM_SPECIMEN_SCORE_DEGREES, arm.SLIDE_SPECIMEN_READY_INCHES),
                        arm.armToPosition(arm.ARM_SPECIMEN_SCORE_DEGREES, arm.SLIDE_SPECIMEN_SCORE_INCHES),
                        arm.clawToPosition(arm.CLAW_OPEN),
                        new SleepAction(0.2),
                        arm.clawToPosition(arm.CLAW_CLOSED),
                        arm.armToPosition(arm.ARM_RESET_DEGREES, arm.SLIDE_RESET_INCHES)
                )
        );
    }
}