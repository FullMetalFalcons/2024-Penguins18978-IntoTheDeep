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
public class TripleSpecimenRoute extends LinearOpMode {

    // Set up constants for the size of the robot
    int botWidth = 18;

    // Set up constants for "preset" field locations
    int STARTING_POSITION_Y = -70 + (botWidth/2);
    int STARTING_POSITION_X = -9;  // Used to be +9

    int SCORING_POSITION_X = STARTING_POSITION_X;
    int SCORING_POSITION_Y = STARTING_POSITION_Y + 11;

    int PICKUP_POSITION_X = 40;
    int PICKUP_POSITION_Y = STARTING_POSITION_Y + 3;

    int PARKING_POSITION_X = 48;
    int PARKING_POSITION_Y = PICKUP_POSITION_Y;

    PinpointDrive drive = null;
    PenguinsArm arm = null;


    public void runOpMode() {

        drive = new PinpointDrive(hardwareMap, new Pose2d(STARTING_POSITION_X, STARTING_POSITION_Y, Math.toRadians(90)));
        arm = new PenguinsArm(hardwareMap, telemetry);

        // Close the claw on initialization
        arm.setClawPosition(arm.CLAW_CLOSED);


        Action drivingRoute1 = drive.actionBuilder(drive.pose)
                // Move to push a sample into the Observation Zone
                .splineToLinearHeading(new Pose2d(PICKUP_POSITION_X-10, PICKUP_POSITION_Y+20, Math.toRadians(90)), 0)
                .splineToLinearHeading(new Pose2d(PICKUP_POSITION_X, PICKUP_POSITION_Y+50, Math.toRadians(90)), 0)
                .strafeTo(new Vector2d(PICKUP_POSITION_X+8, PICKUP_POSITION_Y+50))
                // Move straight down to push the sample
                .strafeTo(new Vector2d(PICKUP_POSITION_X+8, PICKUP_POSITION_Y+5))
                // Move out of the Zone and then back to Pickup Position
                //.splineToConstantHeading(new Vector2d( PICKUP_POSITION_X-10, PICKUP_POSITION_Y+20), Math.toRadians(180))
                //.strafeToLinearHeading(new Vector2d( PICKUP_POSITION_X-15, PICKUP_POSITION_Y+20), Math.toRadians(0))
                .strafeTo(new Vector2d(PICKUP_POSITION_X+8, PICKUP_POSITION_Y+10))
                .strafeTo(new Vector2d(PICKUP_POSITION_X-20, PICKUP_POSITION_Y+10))
                // Wait for human player to place specimen
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d( PICKUP_POSITION_X, PICKUP_POSITION_Y, 0), 0)
                // Pause to grab specimen
                .build();

        Action drivingRoute2 = drive.actionBuilder(drive.pose)
                // Move back to grab a second specimen
                .strafeToLinearHeading(new Vector2d( PICKUP_POSITION_X, PICKUP_POSITION_Y), 0)
                // Pause to grab specimen
                .build();

        Action drivingRoute3 = drive.actionBuilder(drive.pose)
                // Move to park
                //.splineToLinearHeading(new Pose2d( PARKING_POSITION_X, PARKING_POSITION_Y, Math.toRadians(180)), 0)
                .build();

        Action scoringRoute1 = getNewScoringAction(0);

        Action scoringRoute2 = getNewScoringAction(-6);

        Action scoringRoute3 = getNewScoringAction(-12);


        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        scoringRoute1,
                        drivingRoute1,
                        arm.clawToPosition(arm.CLAW_CLOSED),
                        new SleepAction(0.75),
                        scoringRoute2,
                        drivingRoute2,
                        arm.clawToPosition(arm.CLAW_CLOSED),
                        new SleepAction(0.75),
                        scoringRoute3
                )
        );
    }

    public Action getNewScoringAction(int barOffsetX) {
        return new SequentialAction(
                new ParallelAction(
                        arm.armToPosition(arm.ARM_SPECIMEN_READY_DEGREES, arm.SLIDE_RESET_INCHES),
                        drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d( SCORING_POSITION_X + barOffsetX, SCORING_POSITION_Y ), Math.toRadians(90)).build()
                ),
                arm.armToPosition(arm.ARM_SPECIMEN_READY_DEGREES, arm.SLIDE_SPECIMEN_READY_INCHES),
                arm.armToPosition(arm.ARM_SPECIMEN_SCORE_DEGREES, arm.SLIDE_SPECIMEN_READY_INCHES),
                arm.armToPosition(arm.ARM_SPECIMEN_SCORE_DEGREES, arm.SLIDE_SPECIMEN_SCORE_INCHES),
                arm.clawToPosition(arm.CLAW_OPEN),
                new SleepAction(0.2),
                arm.armToPosition(arm.ARM_RESET_DEGREES, arm.SLIDE_RESET_INCHES)
        );
    }
}