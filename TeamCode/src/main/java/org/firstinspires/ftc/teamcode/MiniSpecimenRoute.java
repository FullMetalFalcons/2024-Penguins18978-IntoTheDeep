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
public class MiniSpecimenRoute extends LinearOpMode {

    // Set up constants for the size of the robot
    int botWidth = 18;

    // Set up constants for "preset" field locations
    int STARTING_POSITION_Y = -70 + (botWidth/2);
    int STARTING_POSITION_X = -9;

    int SCORING_POSITION_X = STARTING_POSITION_X;
    int SCORING_POSITION_Y = STARTING_POSITION_Y + 12;

    int PARKING_POSITION_X = -25;
    int PARKING_POSITION_Y = 0;

    PinpointDrive drive = null;
    PenguinsArm arm = null;

    public void runOpMode() {

        drive = new PinpointDrive(hardwareMap, new Pose2d(STARTING_POSITION_X, STARTING_POSITION_Y, Math.toRadians(90)));
        arm = new PenguinsArm(hardwareMap, telemetry);

        // Close the claw on initialization
        arm.setClawPosition(arm.CLAW_CLOSED);

        Action moveToAscentZone = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(PARKING_POSITION_X-11, PARKING_POSITION_Y-30, Math.toRadians(0)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d( PARKING_POSITION_X, PARKING_POSITION_Y, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Action scoringRoute = getNewScoringAction(0);


        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        // Score pre-loaded specimen
                        scoringRoute,
                        // Park in the Ascent Zone
                        moveToAscentZone
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