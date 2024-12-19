package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RoadRunner Specific Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Regular FTC Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
  Wireless Code Download: Terminal --> "adb connect 192.168.43.1:5555"
 */

@Config
@Autonomous
public class SquareTestRoute extends LinearOpMode {
    public void runOpMode() {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        PenguinsArm arm = new PenguinsArm(hardwareMap);

        Action turnTrajectory;
        Action strafeTrajectory;
        Action testTrajectory;

        turnTrajectory = drive.actionBuilder(drive.pose)
                .lineToY(24)
                .turn(Math.toRadians(90))
                .lineToX(-24)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .turnTo(Math.toRadians(90))
                .build();

        strafeTrajectory = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(0, 24))
                .strafeTo(new Vector2d(-24, 24))
                .strafeTo(new Vector2d(-24, 0))
                .strafeTo(new Vector2d(0, 0))
                .build();

        testTrajectory = drive.actionBuilder(drive.pose)
                .lineToY(2)
                .lineToY(0)
                .build();


        Action trajectoryActionChosen = turnTrajectory;

        while (!isStopRequested() && !opModeIsActive()) {
            if (gamepad1.dpad_up) {
                trajectoryActionChosen = turnTrajectory;
            } else if (gamepad1.dpad_down) {
                trajectoryActionChosen = strafeTrajectory;
            } else if (gamepad1.dpad_left) {
                trajectoryActionChosen = testTrajectory;
            }
        }

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        arm.ArmToPosition(2000, 2000),
                        trajectoryActionChosen,
                        arm.ArmToPosition(0, 0)
                )
        );
    }
}