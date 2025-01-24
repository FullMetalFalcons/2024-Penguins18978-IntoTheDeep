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

// HuskyLens Specific Imports
import com.qualcomm.hardware.dfrobot.HuskyLens;

@Config
@Autonomous
public class SpecimenScoreRoute extends LinearOpMode {

    // Set up constants for the size of the robot
    int botWidth = 18;

    // Set up constants for "preset" field locations
    int STARTING_POSITION_Y = -70 + (botWidth/2);
    int STARTING_POSITION_X = 9;

    int SCORING_POSITION_X = STARTING_POSITION_X;
    int SCORING_POSITION_Y = STARTING_POSITION_Y + 12;

    int PICKUP_POSITION_X = 40;
    int PICKUP_POSITION_Y = STARTING_POSITION_Y + 3;

    int PARKING_POSITION_X = 48;
    int PARKING_POSITION_Y = PICKUP_POSITION_Y;

    PinpointDrive drive = null;
    PenguinsArm arm = null;


    private HuskyLens huskyLens;

    public enum HuskyColors {
        NONE,
        BLUE,
        RED
    };
    public HuskyColors colorInView = HuskyColors.NONE;
    HuskyLens.Block[] blocks;


    public void runOpMode() {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        drive = new PinpointDrive(hardwareMap, new Pose2d(STARTING_POSITION_X, STARTING_POSITION_Y, Math.toRadians(90)));
        arm = new PenguinsArm(hardwareMap, telemetry);

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

        Action scoringRoute1 = getNewScoringAction(0);

        Action scoringRoute2 = getNewScoringAction(-6);

        Action scoringRoute3 = getNewScoringAction(-12);


        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        // Score pre-loaded specimen
                        scoringRoute1,
                        // Drive to get another
                        drive.actionBuilder(drive.pose).splineToLinearHeading(new Pose2d( PICKUP_POSITION_X, PICKUP_POSITION_Y, 0), 0).build(),
                        arm.clawToPosition(arm.CLAW_CLOSED),
                        new SleepAction(0.75),
                        // Back up to check camera data
                        drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d(PICKUP_POSITION_X-5, PICKUP_POSITION_Y ), 0).build()
                        )
        );

        getHuskyLensData();
        while (getHuskyLensColor() == HuskyColors.NONE) {
            // The pickup process failed somehow: Pause and try again
            Actions.runBlocking(
                    new SequentialAction(
                            arm.clawToPosition(arm.CLAW_OPEN),
                            new SleepAction(1),
                            drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d( PICKUP_POSITION_X, PICKUP_POSITION_Y), 0).build(),
                            arm.clawToPosition(arm.CLAW_CLOSED)
                    )
            );
            // Check the camera again
            // If the pickup failed, try again (again)
            getHuskyLensData();
        }
        // If the statement above is false, everything succeeded: Precede as normal

        Actions.runBlocking(
                new SequentialAction(
                        // Score 2nd specimen
                        scoringRoute2,
                        // Park in the Observation Zone
                        drive.actionBuilder(drive.pose).splineToLinearHeading(new Pose2d( PARKING_POSITION_X, PARKING_POSITION_Y, Math.toRadians(180)), 0).build()
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

    private void getHuskyLensData() {
        blocks = huskyLens.blocks();
    }

    private HuskyColors getHuskyLensColor() {

        // Walk through each object seen by the HuskyLens
        for (int i = 0; i < blocks.length; i++) {
            if (blocks[i].id == 1) {
                // HuskyLens sees a blue sample
                colorInView = HuskyColors.BLUE;
            } else if (blocks[i].id == 2) {
                // HuskyLens sees a red sample
                colorInView = HuskyColors.RED;
            }
        }
        if (blocks.length < 1) {
            colorInView = HuskyColors.NONE;
        }
        return colorInView;
    }
}