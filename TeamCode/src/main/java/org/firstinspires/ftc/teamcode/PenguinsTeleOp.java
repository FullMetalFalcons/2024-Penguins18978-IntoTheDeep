package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.concurrent.TimeUnit;


@TeleOp
@Config
public class PenguinsTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx m1, m2, m3, m4, Arm, Slide, Hanger;
    Servo Claw, Wrist, Light;

    // 'Public static' vars can be viewed and changed on the web dashboard
    public static double REGULAR_ARM_POWER_UP = 1.0;
    public static double REGULAR_ARM_POWER_DOWN = -1.0;

    // Set up constants for the size of the robot
    public final double BOT_WIDTH = 18.0;
    // Set up constants for "preset" field locations
    public final double STARTING_POSITION_Y = -70.0 + (BOT_WIDTH /2);
    public final double STARTING_POSITION_X = 9.0;
    public final double STARTING_ANGLE_DEG = 90.0;

    // Indicator Light constants
    public final double LED_RED = 0.279;
    public final double LED_BLUE = 0.611;
    public final double LED_YELLOW = 0.38;
    public final double LED_GREEN = 0.5;

    // Match time variables
    public double startTimeSeconds;
    public double matchDurationSeconds = 120;
    public double elapsedTimeSeconds;
    public double timeLeftSeconds;

    // Wrist variables
    boolean wristDeployed = true;
    double lastRightTrigger;


    // HuskyLens variables
    private HuskyLens huskyLens;
    public enum HuskyColors {
        NONE,
        BLUE,
        RED
    };
    public HuskyColors colorInView = HuskyColors.NONE;

    public void runOpMode() {
        //This will send telemetry data to the web dashboard (192.168.43.1:8080/dash)
        //  in addition to the driver station.
        //  It also allows any 'public static' class attributes to be viewed and changed
        //  on the dashboard as well
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointDrive drive = new PinpointDrive(hardwareMap, STARTING_POSITION_X, STARTING_POSITION_Y, STARTING_ANGLE_DEG);
        PenguinsArm penguinsArm = new PenguinsArm(hardwareMap, telemetry);
        TelemetryPacketOpMode telemetryPacket = new TelemetryPacketOpMode(telemetry);

        // Set up HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        m1 = drive.leftFront;
        m2 = drive.rightFront;
        m3 = drive.leftBack;
        m4 = drive.rightBack;

        Arm = penguinsArm.Arm;
        Slide = penguinsArm.Slide;
        Claw = penguinsArm.Claw;
        Wrist = penguinsArm.Wrist;
        Hanger = penguinsArm.Hanger;

        Light = hardwareMap.get(Servo.class, "rightIndicatorLight");

        // TODO: Figure out if this can be removed or should be added to MecanumDrive
        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        //m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //This is to keep track of the current (if any) auto arm slide Action in progress
        PenguinsArm.ArmSlideToPosition autoArmSlider = null;


        waitForStart();

        // Set start time now
        startTimeSeconds = TimeUnit.NANOSECONDS.toSeconds(System.nanoTime());

        while(opModeIsActive()) {
            // Mecanum drive code
            double px = 0.0;
            double py = 0.0;
            double pa = 0.0;
            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
                // This allows for driving via dpad as well
                // Uses ternaries to make the code more compact (condition ? if true : if false)
                px = gamepad1.dpad_left ? -0.3 : 0.0;
                px = gamepad1.dpad_right ? 0.3 : px;
                py = gamepad1.dpad_down ? -0.3 : 0.0;
                py = gamepad1.dpad_up ? 0.3 : py;
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

            // Arm Input Code
            double desiredArmPower = 0.0;
            if (gamepad1.right_bumper) {
                // Arm Up, if the limit will not be passed
                desiredArmPower = REGULAR_ARM_POWER_UP;
            } else if (gamepad1.right_trigger > 0) {
                // Arm Down, if the limit will not be passed
                desiredArmPower = REGULAR_ARM_POWER_DOWN;
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
                autoArmSlider = penguinsArm.armToPosition(penguinsArm.ARM_RESET_DEGREES, penguinsArm.SLIDE_RESET_INCHES, 10);
            } else if (gamepad2.dpad_up && autoArmSlider == null) {
                autoArmSlider = penguinsArm.armToPosition(penguinsArm.ARM_SPECIMEN_READY_DEGREES, penguinsArm.SLIDE_SPECIMEN_READY_INCHES, 10);
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
                if(autoArmSlider.run(telemetryPacket) == false){
                    //We've finished the auto movement, return to manual mode
                    autoArmSlider = null;
                }
            }


            // Claw Code
            if (gamepad1.y || gamepad2.right_bumper) {
                // Open Position
                penguinsArm.setClawPosition(penguinsArm.CLAW_OPEN);
            } else {
                // Closed Position
                penguinsArm.setClawPosition(penguinsArm.CLAW_CLOSED);
            }

            // Wrist Toggle Code
            if (gamepad2.right_trigger > 0 && lastRightTrigger == 0) {
                // Flip to the alternate state
                wristDeployed = !wristDeployed;
            }
                // Set servo position based on state
            penguinsArm.setWristPosition(wristDeployed ? penguinsArm.WRIST_DEPLOYED : penguinsArm.WRIST_FOLDED);
            lastRightTrigger = gamepad2.right_trigger;


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


            // EMERGENCY encoder reset sequence
            if ((gamepad1.start && gamepad1.back) || (gamepad2.start && gamepad2.back)) {
                // In case of "emergency," reset all encoders
                penguinsArm.resetArmEncoder();
                penguinsArm.resetSlideEncoder();
                penguinsArm.resetHangerEncoder();

                telemetry.addLine("Reset encoders");
            }


            // Get HuskyLens data
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            colorInView = HuskyColors.NONE;

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
            telemetry.addData("Color detected:", colorInView);


            // Update match time variables
            elapsedTimeSeconds = TimeUnit.NANOSECONDS.toSeconds(System.nanoTime()) - startTimeSeconds;
            timeLeftSeconds = matchDurationSeconds - elapsedTimeSeconds;
            handleColors();


            // Have each module add debug data to the telemetry object so it can be sent to the
            // driver's station
            drive.addDebugData(telemetry);
            penguinsArm.addDebugData();

            telemetry.update();

        } // opModeActive loop ends
    }

    public void handleColors() {
        // Choose color based on time frame
        if (timeLeftSeconds < 10) {
            // Red for "HANG THIS INSTANT!!!"
            Light.setPosition(LED_RED);
        } else if (timeLeftSeconds < 20) {
            // Yellow for "Go, like now"
            Light.setPosition(LED_YELLOW);
        } else if (timeLeftSeconds < 30) {
            // Green for "Endgame! Watch your cycling"
            Light.setPosition(LED_GREEN);
        } else {
            Light.setPosition(0);
        }
    }

} // end class