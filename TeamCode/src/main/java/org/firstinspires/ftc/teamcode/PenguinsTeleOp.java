package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;

import java.util.Locale;

@TeleOp
@Config
public class PenguinsTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx m1, m2, m3, m4, Arm, Slide, Hanger;
    Servo Claw;
    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();
    public static PinpointDrive.Params PINPOINT_PARAMS = new PinpointDrive.Params();

    public PenguinsArm penguinsArm = null;

    public static double arm_p = 5;
    public static double arm_i = 0.05;
    public static double arm_d = 0;

    public static double slide_p = 5;
    public static double slide_i = 0.05;
    public static double slide_d = 0;

    // Built-in PID loops for RUN_TO_POSITION
    public PIDFCoefficients armPID = new PIDFCoefficients(arm_p, arm_i, arm_d, 0);
    public PIDFCoefficients slidePID = new PIDFCoefficients(slide_p, slide_i, slide_d, 0);

    public PenguinsArm.ArmSlideToPosition autoArmSlider = null;



    // Custom Controls variables
    double virtualRightStickX = 0.0;


    // Declare OpMode member for the Odometry Computer
    GoBildaPinpointDriverRR odo;

    public void runOpMode() {
        penguinsArm = new PenguinsArm(hardwareMap, telemetry);

        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        m1 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftFrontDriveName);
        m2 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightFrontDriveName);
        m3 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftBackDriveName);
        m4 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightBackDriveName);
        Arm = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.armName);
        Slide = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.slideName);
        Hanger = (DcMotorEx) hardwareMap.dcMotor.get("linearActuator");

        Claw = (Servo) hardwareMap.servo.get("claw");

        //Set them to the correct modes
        //This reverses the motor direction
        m1.setDirection(DRIVE_PARAMS.leftFrontDriveDirection);
        m2.setDirection(DRIVE_PARAMS.rightFrontDriveDirection);
        m3.setDirection(DRIVE_PARAMS.leftBackDriveDirection);
        m4.setDirection(DRIVE_PARAMS.rightBackDriveDirection);

        Slide.setDirection(DRIVE_PARAMS.slideDirection);
        Hanger.setDirection(DRIVE_PARAMS.slideDirection);


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
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            //  This code allows the robot to turn via gamepad 1 or 2's joystick
            if (Math.abs(gamepad1.right_stick_x) > 0.0) {
                virtualRightStickX = gamepad1.right_stick_x;
            } else if (Math.abs(gamepad2.right_stick_x) > 0.3) {
                virtualRightStickX = gamepad2.right_stick_x;
            } else {
                virtualRightStickX = 0.0;
            }
            double px = 0.0;
            double py = 0.0;
            double pa = 0.0;
            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
                // This allows for driving via dpad as well
                // Uses ternaries to make the code more compact (condition ? if true : if false)
                px = gamepad1.dpad_left ? -0.2 : 0.0;
                px = gamepad1.dpad_right ? 0.2 : px;
                py = gamepad1.dpad_down ? -0.2 : 0.0;
                py = gamepad1.dpad_up ? 0.2 : py;
            } else {
                // If the dpad is not in use, drive via sticks
                px = gamepad1.left_stick_x;
                py = -gamepad1.left_stick_y;
                pa = -virtualRightStickX;
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






            // Arm Input Code
            double desiredArmPower = 0.0;
            if (gamepad1.right_bumper) {
                // Arm Up, if the limit will not be passed
                desiredArmPower = 1;
            } else if (gamepad1.right_trigger > 0) {
                // Arm Down, if the limit will not be passed
                desiredArmPower = -1;
            } else {
                // Go by gamepad2 joystick
                desiredArmPower = -gamepad2.left_stick_y;
            }

            // Slide Input Code
            double desiredSlidePower = 0.0;
            if (gamepad1.left_bumper) {
                // Slide Out, if the limit will not be passed
                desiredSlidePower = 1;
            } else if (gamepad1.left_trigger > 0) {
                // Slide In
                // I don't think a limit check is ever necessary here, but just in case...
                desiredSlidePower = -1;
            } else {
                // Go by gamepad2 joystick
                desiredSlidePower = -gamepad2.right_stick_y;
            }

            //TODO: Figure out how to switch directions in auto
            //See if we need to start some auto arm movements
            if (gamepad2.dpad_down && autoArmSlider == null) {
                autoArmSlider = penguinsArm.armToPosition(penguinsArm.ARM_RESET_DEGREES, penguinsArm.SLIDE_RESET_INCHES);
            } else if (gamepad2.dpad_up && autoArmSlider == null) {
                autoArmSlider = penguinsArm.armToPosition(penguinsArm.ARM_SPECIMEN_SCORE_DEGREES, penguinsArm.SLIDE_SPECIMEN_SCORE_INCHES);
            }

            if (desiredArmPower != 0 || desiredSlidePower != 0) {
                autoArmSlider = null;  // Cancel any auto actions
            }

            if (autoArmSlider == null) {
                // Set power based on manual inputs
                penguinsArm.setArmPower(desiredArmPower);
                penguinsArm.setSlidePower(desiredSlidePower);
            } else {
                // Run the auto action until it finishes
                if(autoArmSlider.run(null) == false){  //TODO Need to fix this Telemetry Packet
                    //We've finished the auto movement, return to manual mode
                    autoArmSlider = null;
                }
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
            if (gamepad1.y || gamepad2.right_bumper) {
                // Open Position
                Claw.setPosition(0.3);
            } else {
                // Closed Position
                Claw.setPosition(0.6);
            }


            // EMERGENCY encoder reset sequence
            if ((gamepad1.start && gamepad1.back) || (gamepad2.start && gamepad2.back)) {
                // In case of "emergency," reset all encoders
                Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                telemetry.addLine("Reset encoders");
            }



            odo.update();

            // gets the current Position (x & y in inches, and heading in degrees) of the robot, and prints it.
            Pose2d pos = odo.getPositionRR();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, Math.toDegrees(pos.heading.toDouble()));
            telemetry.addData("Position", data);

            // gets the current Velocity (x & y in inches/sec and heading in degrees/sec) and prints it.
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

            penguinsArm.addDebugData();
            telemetry.update();

        } // opModeActive loop ends
    }



} // end class