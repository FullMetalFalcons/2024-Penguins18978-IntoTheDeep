package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RoadRunner Specific Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
    // Get motor attributes for Auto
    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();

    public class ArmClass {
        private DcMotorEx Arm;
        public ArmClass(HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotorEx.class, DRIVE_PARAMS.armName);
            Arm.setDirection(DRIVE_PARAMS.armDirection);

            // The arm will hold its position when given 0.0 power
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Reset the arm's position, then let the code control it
            Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        public class ArmToPosition implements Action {
            // Use constructor parameter to set target position
            private double targetPosition;
            public ArmToPosition(double targetPos) {
                targetPosition = targetPos;
            }

            private double armMovementDirection = 0;
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double position = Arm.getCurrentPosition();
                packet.put("Arm Position", position);

                // If the target position is higher than the current
                //   position use positive power:  else, use negative power

                if (!initialized) {
                    armMovementDirection = (targetPosition > position ? 1 : -1);
                    Arm.setPower(0.8 * armMovementDirection);
                    initialized = true;
                }

                if ((position < targetPosition && armMovementDirection == 1) || (position > targetPosition && armMovementDirection == -1)) {
                    // Returning true will run the action again
                    return true;
                } else {
                    // Returning false will end the action
                    Arm.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmToPosition(double targetPosition) {
            return new ArmToPosition(targetPosition);
        }


        /*
        public class ArmOffFloor implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Arm.setPower(0.8);
                    initialized = true;
                }

                double position = Arm.getCurrentPosition();
                packet.put("Arm Position", position);
                if (position < 500.0) {
                    return true;
                } else {
                    Arm.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmOffFloor() {
            return new ArmOffFloor();
        }


        public class ArmToFloor implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Arm.setPower(-0.8);
                    initialized = true;
                }

                double position = Arm.getCurrentPosition();
                packet.put("Arm Position", position);
                if (position > 0.0) {
                    return true;
                } else {
                    Arm.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmToFloor(){
            return new ArmToFloor();
        }
    */

    }



    public void runOpMode() {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        ArmClass arm = new ArmClass(hardwareMap);

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
                        arm.ArmToPosition(2000),
                        trajectoryActionChosen,
                        arm.ArmToPosition(0)
                )
        );
    }
}