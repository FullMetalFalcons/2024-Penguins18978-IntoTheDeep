package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class PenguinsTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx m1, m2, m3, m4, Arm, Slide;

    public void runOpMode() {

        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");

        Arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        Slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");

        //Set them to the correct modes
        //This reverses the motor direction
        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        //This resets the encoder values when the code is initialized
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while(opModeIsActive()) {
            // Mecanum drive code
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
                Arm.setPower(-1);
            } else if (gamepad1.right_trigger > 0) {
                // Arm Down
                Arm.setPower(1);
            } else {
                // At Rest
                Arm.setPower(0);
            }

            // Slide Code
            if (gamepad1.left_bumper) {
                // Arm Up
                Slide.setPower(-1);
            } else if (gamepad1.left_trigger > 0) {
                // Arm Down
                Slide.setPower(1);
            } else {
                // At Rest
                Slide.setPower(0);
            }

        } // opModeActive loop ends
    }
} // end class

