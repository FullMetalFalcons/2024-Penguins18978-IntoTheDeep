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

import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class TripleSpecimenRoute extends LinearOpMode {
    Servo Light;

    // Set up constants for the size of the robot
    int botWidth = 18;

    // Set up constants for "preset" field locations
    int STARTING_POSITION_Y = -70 + (botWidth/2);
    int STARTING_POSITION_X = 9;

    // A distance that will be added to SCORING_POSITION_Y
    //   after team alliance color is determined (to account
    //   for red and blue side differences)
    int scoringPositionYAddon = 13;

    int SCORING_POSITION_X = STARTING_POSITION_X;
    int SCORING_POSITION_Y = STARTING_POSITION_Y;

    int PICKUP_POSITION_X = 41;
    int PICKUP_POSITION_Y = STARTING_POSITION_Y + 4;

    int PARKING_POSITION_X = 48;
    int PARKING_POSITION_Y = PICKUP_POSITION_Y;


    // Indicator Light constants
    public final double LED_RED = 0.279;
    public final double LED_BLUE = 0.611;

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

        Light = hardwareMap.get(Servo.class, "rightIndicatorLight");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        sleep(100);
        // Adjust robot positioning based on which color specimen is pre-loaded
        if (getHuskyLensColor() == HuskyColors.BLUE || gamepad1.a) {
            scoringPositionYAddon = 11;
            Light.setPosition(LED_BLUE);
        } else {
            scoringPositionYAddon = 13;
            Light.setPosition(LED_RED);
        }
        SCORING_POSITION_Y += scoringPositionYAddon;

        // Wait to display the indicator color, then turn off the light
        sleep(1000);
        Light.setPosition(0);



        drive = new PinpointDrive(hardwareMap, new Pose2d(STARTING_POSITION_X, STARTING_POSITION_Y, Math.toRadians(90)));
        arm = new PenguinsArm(hardwareMap, telemetry);

        // Close the claw on initialization
        arm.setClawPosition(arm.CLAW_CLOSED);


        Action drivingRoute1 =drive.actionBuilder(drive.pose)
                // Move to push a sample into the Observation Zone
                .splineToLinearHeading(new Pose2d(PICKUP_POSITION_X-10, PICKUP_POSITION_Y+20, Math.toRadians(90)), 0)
                .splineToLinearHeading(new Pose2d(PICKUP_POSITION_X, PICKUP_POSITION_Y+42, Math.toRadians(90)), 0)
                .strafeTo(new Vector2d(PICKUP_POSITION_X+6, PICKUP_POSITION_Y+42))
                // Move straight down to push the sample
                .strafeTo(new Vector2d(PICKUP_POSITION_X+6, PICKUP_POSITION_Y+5))
                // Move out of the Zone and then back to Pickup Position
                .strafeTo(new Vector2d(PICKUP_POSITION_X+6, PICKUP_POSITION_Y+10))
                .strafeToLinearHeading(new Vector2d(PICKUP_POSITION_X-15, PICKUP_POSITION_Y), Math.toRadians(0))
                // Wait for human player to place specimen
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d( PICKUP_POSITION_X, PICKUP_POSITION_Y, 0), 0)
                .build();

        Action drivingRoute2 = drive.actionBuilder(drive.pose)
                // Move back to grab a second specimen
                .strafeToLinearHeading(new Vector2d( SCORING_POSITION_X, PICKUP_POSITION_Y ), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d( PICKUP_POSITION_X, PICKUP_POSITION_Y ), 0)
                // Pause to grab specimen
                .build();

        Action drivingRoute3 = drive.actionBuilder(drive.pose)
                // Move to park
                //.splineToLinearHeading(new Pose2d( PARKING_POSITION_X, PARKING_POSITION_Y, Math.toRadians(180)), 0)
                .build();

        Action scoringRoute1 = getNewScoringAction(0, 10,2000);

        Action scoringRoute2 = getNewScoringAction(-3, 10,2000);

        Action scoringRoute3 = getNewScoringAction(-6, 0,30);


        waitForStart();
        if (isStopRequested()) return;

        Light.setPosition(0.0);


        Actions.runBlocking(
                new SequentialAction(
                        scoringRoute1,
                        drivingRoute1,
                        arm.armToPosition(arm.ARM_RESET_DEGREES, arm.SLIDE_RESET_INCHES, 10),
                        arm.clawToPosition(arm.CLAW_CLOSED),
                        new SleepAction(0.75),
                        scoringRoute2,
                        drivingRoute2,
                        arm.armToPosition(arm.ARM_RESET_DEGREES, arm.SLIDE_RESET_INCHES, 10),
                        arm.clawToPosition(arm.CLAW_CLOSED),
                        new SleepAction(0.75),
                        scoringRoute3
                )
        );
    }

    public Action getNewScoringAction(int barOffsetX, int armOffset, int endTolerance) {
        return new SequentialAction(
                new ParallelAction(
                        arm.armToPosition(arm.ARM_SPECIMEN_READY_DEGREES, arm.SLIDE_RESET_INCHES, 30),
                        drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d(SCORING_POSITION_X + barOffsetX, SCORING_POSITION_Y), Math.toRadians(90)).build()
                ),
                arm.armToPosition(arm.ARM_SPECIMEN_READY_DEGREES, arm.SLIDE_SPECIMEN_READY_INCHES, 10),
                arm.armToPosition(arm.ARM_SPECIMEN_SCORE_DEGREES, arm.SLIDE_SPECIMEN_READY_INCHES, 30),
                arm.armToPosition(arm.ARM_SPECIMEN_SCORE_DEGREES, arm.SLIDE_SPECIMEN_SCORE_INCHES, 15),
                arm.clawToPosition(arm.CLAW_OPEN),
                new SleepAction(0.2),
                arm.armToPosition(arm.ARM_RESET_DEGREES + armOffset, arm.SLIDE_RESET_INCHES, endTolerance)
        );
    }

    private HuskyColors getHuskyLensColor() {
        // Get camera data
        blocks = huskyLens.blocks();

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