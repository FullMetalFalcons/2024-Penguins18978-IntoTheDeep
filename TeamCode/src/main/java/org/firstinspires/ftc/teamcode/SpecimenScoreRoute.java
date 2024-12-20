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
        // Set up constants for the size of the robot
        int botWidth = 18;

        // Set up constants for "preset" field locations
        int STARTING_POSITION_Y = -70 + (botWidth/2);
        int STARTING_POSITION_X = 9;

        int SCORING_POSITION_X = STARTING_POSITION_X;
        int SCORING_POSITION_Y = STARTING_POSITION_Y + 11;

        int PICKUP_POSITION_X = 40;
        int PICKUP_POSITION_Y = STARTING_POSITION_Y + 3;

        int PARKING_POSITION_X = 48;
        int PARKING_POSITION_Y = PICKUP_POSITION_Y;


        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(STARTING_POSITION_X, STARTING_POSITION_Y, Math.toRadians(90)));
        PenguinsArm arm = new PenguinsArm(hardwareMap, telemetry);

        // Close the claw on initialization
        arm.setClawPosition(arm.CLAW_CLOSED);


        Action fullScoringTrajectory;

        fullScoringTrajectory = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d( SCORING_POSITION_X, SCORING_POSITION_Y) )
                // Pause to score specimen
                .waitSeconds(10)
                .splineToLinearHeading(new Pose2d( PICKUP_POSITION_X, PICKUP_POSITION_Y, 0), 0)
                // Pause to grab specimen
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d( SCORING_POSITION_X, SCORING_POSITION_Y ), Math.toRadians(90))
                // Pause to score specimen
                .waitSeconds(10)
                .splineToLinearHeading(new Pose2d( PARKING_POSITION_X, PARKING_POSITION_Y, Math.toRadians(180)), 0)
                .build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                arm.armToPosition(arm.ARM_SPECIMEN_READY_DEGREES, arm.SLIDE_SPECIMEN_READY_INCHES),
                                drive.actionBuilder(drive.pose).strafeTo(new Vector2d(SCORING_POSITION_X, SCORING_POSITION_Y)).build()
                        ),
//                        arm.armToPosition(arm.ARM_SPECIMEN_READY_DEGREES, arm.SLIDE_SPECIMEN_READY_INCHES),
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