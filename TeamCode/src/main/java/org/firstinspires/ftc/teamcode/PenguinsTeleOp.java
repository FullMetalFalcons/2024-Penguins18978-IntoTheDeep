package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;

import java.util.Locale;

@TeleOp
public class PenguinsTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx m1, m2, m3, m4, Arm, Slide, Hanger;
    Servo Claw;
    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();
    public static PinpointDrive.Params PINPOINT_PARAMS = new PinpointDrive.Params();


    // Encoder storage variables
    int slideLength = 0;
    final double inPerSlideTick = 0.0;

    int armAngle = 0;
    final double degreePerArmTick = 0.0;

    int currentRobotLength = 0;
    final int MAX_ROBOT_LENGTH = 42;


    // Declare OpMode member for the Odometry Computer
    GoBildaPinpointDriverRR odo;

    public void runOpMode() {

        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        m1 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftFrontDriveName);
        m2 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightFrontDriveName);
        m3 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftBackDriveName);
        m4 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightBackDriveName);
        Arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        Slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");
        Hanger = (DcMotorEx) hardwareMap.dcMotor.get("linearActuator");

        Claw = (Servo) hardwareMap.servo.get("claw");

        //Set them to the correct modes
        //This reverses the motor direction
        m1.setDirection(DRIVE_PARAMS.leftFrontDriveDirection);
        m2.setDirection(DRIVE_PARAMS.rightFrontDriveDirection);
        m3.setDirection(DRIVE_PARAMS.leftBackDriveDirection);
        m4.setDirection(DRIVE_PARAMS.rightBackDriveDirection);

        Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        Hanger.setDirection(DcMotorSimple.Direction.REVERSE);


        //This resets the encoder values when the code is initialized
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Pinpoint computer setup
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, PINPOINT_PARAMS.pinpointDeviceName);
        odo.setOffsets(DistanceUnit.MM.fromInches(PINPOINT_PARAMS.xOffset), DistanceUnit.MM.fromInches(PINPOINT_PARAMS.yOffset));
        odo.setEncoderResolution(PINPOINT_PARAMS.encoderResolution);
        odo.setEncoderDirections(PINPOINT_PARAMS.xDirection, PINPOINT_PARAMS.yDirection);
        odo.resetPosAndIMU();

        waitForStart();

        while(opModeIsActive()) {
            // Mecanum drive code
            double px = 0.0;
            double py = 0.0;
            double pa = 0.0;
            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
                // This allows for driving via dpad as well
                // Uses ternaries to make the code more compact (condition ? if true : if false)
                px = gamepad1.dpad_left ? -0.8 : 0.0;
                px = gamepad1.dpad_right ? 0.8 : px;
                py = gamepad1.dpad_down ? -0.8 : 0.0;
                py = gamepad1.dpad_up ? 0.8 : py;
            } else {
                // If the dpad is not in use, drive via sticks
                px = gamepad1.left_stick_x;
                py = -gamepad1.left_stick_y;
                pa = -gamepad1.right_stick_x;
            }

            double p1 = px + py - pa;
            double p2 = -px + py + pa;
            double p3 = -px + py - pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);


            // Arm Code
            if (gamepad1.right_bumper) {
                // Arm Up
                Arm.setPower(1);
            } else if (gamepad1.right_trigger > 0) {
                // Arm Down
                Arm.setPower(-1);
            } else {
                // At Rest
                Arm.setPower(0);
            }

            // Slide Code
            if (gamepad1.left_bumper) {
                // Slide Out
                Slide.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                // Slide In
                Slide.setPower(-1);
            } else {
                // At Rest
                Slide.setPower(0);
            }

            // Hanging Arm Code
            if (gamepad2.left_bumper) {
                // Actuator Out
                Hanger.setPower(1);
            } else {
                /* Add:
                   Hanger.getCurrentPosition() < -500 ||
                   to make the arm auto retract
                 */
                if (gamepad2.left_trigger > 0) {
                    // Retract if out
                    Hanger.setPower(-1);
                } else {
                    // Stop if fully back
                    Hanger.setPower(0);
                }
            }

            // Claw Code
            if (gamepad1.y) {
                // Open Position
                Claw.setPosition(0.3);
            } else {
                // Closed Position
                Claw.setPosition(0.6);
            }



            odo.update();

            /*
            gets the current Position (x & y in inches, and heading in degrees) of the robot, and prints it.
             */
            Pose2d pos = odo.getPositionRR();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, Math.toDegrees(pos.heading.toDouble()));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in inches/sec and heading in degrees/sec) and prints it.
             */
            PoseVelocity2d vel = odo.getVelocityRR();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.linearVel.x, vel.linearVel.y, Math.toDegrees(vel.angVel));
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
            telemetry.addData("Status", odo.getDeviceStatus());

            if (gamepad1.a) {
                // Encoder telemetry
                telemetry.addData("Arm Pos", Arm.getCurrentPosition());
                telemetry.addData("Slide Pos", Slide.getCurrentPosition());
            }

            telemetry.update();

        } // opModeActive loop ends
    }

    // Method to check whether the robot will still be within size constraints after the desired movements
    public boolean getNewRobotWidth(int deltaLength, int deltaAngle) {
        slideLength = (int) (Slide.getCurrentPosition() * inPerSlideTick);
        slideLength += deltaLength;

        armAngle = (int) (Arm.getCurrentPosition() * degreePerArmTick);
        armAngle += deltaAngle;

        currentRobotLength = (int) (Math.cos(armAngle) * slideLength);
        return currentRobotLength < MAX_ROBOT_LENGTH;
    }


} // end class