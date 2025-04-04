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
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

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

    PinpointDrive drive = null;
    PenguinsArm arm = null;


    public void runOpMode() {

        drive = new PinpointDrive(hardwareMap, new Pose2d(STARTING_POSITION_X, STARTING_POSITION_Y, Math.toRadians(90)));
        arm = new PenguinsArm(hardwareMap, telemetry);

        // Close the claw on initialization
        arm.setClawPosition(arm.CLAW_CLOSED);


        Action drivingRoute1 = drive.actionBuilder(drive.pose)
                // Move into Scoring Position
                .splineToLinearHeading(new Pose2d( SCORING_POSITION_X, SCORING_POSITION_Y, Math.toRadians(225) ), Math.toRadians(180))
                .build();

        Action drivingRoute2 = drive.actionBuilder(drive.pose)
                // Low level ascent
                .strafeToConstantHeading(new Vector2d(-36, SCORING_POSITION_Y))
                .strafeToLinearHeading(new Vector2d( -36, -24), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d( PARKING_POSITION_X, PARKING_POSITION_Y, 0), 0)
                .build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        arm.wristToPosition(arm.WRIST_DEPLOYED),
                        new ParallelAction(
                                drivingRoute1,
                                arm.armToPosition(65, 0, 50)
                        ),
                        arm.armToPosition(65, 32, 10),
                        arm.clawToPosition(arm.CLAW_OPEN),
                        new SleepAction(0.75),
                        arm.armToPosition(75, 32, 50),
                        arm.armToPosition(75, 3, 50),
                        new ParallelAction(
                                arm.armToPosition(50, 3, 50),
                                drivingRoute2
                        )
                )
        );
    }
}