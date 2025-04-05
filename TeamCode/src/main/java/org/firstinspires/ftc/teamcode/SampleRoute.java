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
public class SampleRoute extends LinearOpMode {

    // Set up constants for the size of the robot
    int botWidth = 18;

    // Set up constants for "preset" field locations
    int STARTING_POSITION_Y = -70 + (botWidth/2);
    int STARTING_POSITION_X = -48 + (botWidth/2);

    int SCORING_POSITION_X = -70 + 24;
    int SCORING_POSITION_Y = -70 + 24;

    int PARKING_POSITION_X = -20;
    int PARKING_POSITION_Y = -12;

    int SCORING_ANGLE_DEGREES = 66;
    int PRE_SCORING_LENGTH_INCHES = 10;

    PinpointDrive drive = null;
    PenguinsArm arm = null;


    public void runOpMode() {

        drive = new PinpointDrive(hardwareMap, new Pose2d(STARTING_POSITION_X, STARTING_POSITION_Y, Math.toRadians(90)));
        arm = new PenguinsArm(hardwareMap, telemetry);

        // Close the claw on initialization
        arm.setClawPosition(arm.CLAW_CLOSED);


        Action driveToScoring1 = drive.actionBuilder(drive.pose)
                // Move into Scoring Position
                .splineToLinearHeading(new Pose2d( SCORING_POSITION_X, SCORING_POSITION_Y, Math.toRadians(225) ), Math.toRadians(180))
                .build();

        Action driveToSample1 = drive.actionBuilder(drive.pose)
                // Pick up 2nd Sample
                .strafeToLinearHeading(new Vector2d( SCORING_POSITION_X-2, SCORING_POSITION_Y+12), Math.toRadians(90))
                .build();

        Action driveToScoring2 = drive.actionBuilder(drive.pose)
                // Move into Scoring Position
                .strafeToLinearHeading(new Vector2d( SCORING_POSITION_X, SCORING_POSITION_Y), Math.toRadians(225))
                .build();

        Action driveToSample2 = drive.actionBuilder(drive.pose)
                // Pick up 2nd Sample
                .strafeToLinearHeading(new Vector2d( SCORING_POSITION_X-12, SCORING_POSITION_Y+12), Math.toRadians(90))
                .build();

        Action driveToScoring3 = drive.actionBuilder(drive.pose)
                // Move into Scoring Position
                .strafeToLinearHeading(new Vector2d( SCORING_POSITION_X, SCORING_POSITION_Y), Math.toRadians(225))
                .build();

        Action driveToPark = drive.actionBuilder(drive.pose)
                // Low level ascent
                .strafeToConstantHeading(new Vector2d(-36, SCORING_POSITION_Y))
                .strafeToLinearHeading(new Vector2d( -36, -24), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d( PARKING_POSITION_X, PARKING_POSITION_Y, 0), 0)
                .build();

        Action scoringRoute1 = getNewScoringAction(6);
        Action scoringRoute2 = getNewScoringAction(6);
        Action scoringRoute3 = getNewScoringAction(10);


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        arm.wristToPosition(arm.WRIST_DEPLOYED),
                        new ParallelAction(
                                driveToScoring1,
                                arm.armToPosition(SCORING_ANGLE_DEGREES, PRE_SCORING_LENGTH_INCHES, 50)
                        ),
                        scoringRoute1,
                        new ParallelAction(
                                arm.armToPosition(20, 3, 50),
                                driveToSample1
                        ),
                        arm.armToPosition(8, 3, 50),
                        arm.clawToPosition(arm.CLAW_CLOSED),
                        new SleepAction(0.75),
                        new ParallelAction(
                                arm.armToPosition(SCORING_ANGLE_DEGREES, PRE_SCORING_LENGTH_INCHES, 50),
                                driveToScoring2
                        ),
                        scoringRoute2,
                        new ParallelAction(
                                arm.armToPosition(20, 3, 50),
                                driveToSample2
                        ),
                        arm.armToPosition(8, 3, 50),
                        arm.clawToPosition(arm.CLAW_CLOSED),
                        new SleepAction(0.75),
                        new ParallelAction(
                                arm.armToPosition(SCORING_ANGLE_DEGREES, PRE_SCORING_LENGTH_INCHES, 50),
                                driveToScoring3
                        ),
                        scoringRoute3,
                        new ParallelAction(
                                arm.armToPosition(50, 3, 50),
                                driveToPark
                        )
                )
        );
    }

    public Action getNewScoringAction(int endingSlideInches) {
        return new SequentialAction(
                arm.armToPosition(SCORING_ANGLE_DEGREES, 32, 10),
                arm.clawToPosition(arm.CLAW_OPEN),
                new SleepAction(0.75),
                arm.armToPosition(75, 32, 50),
                arm.armToPosition(75, endingSlideInches, 50)
        );
    }
}