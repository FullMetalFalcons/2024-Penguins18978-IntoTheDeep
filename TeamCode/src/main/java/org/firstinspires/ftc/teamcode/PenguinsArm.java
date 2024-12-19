package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PenguinsArm {
    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();

    private DcMotorEx Arm;
    private DcMotorEx Slide;
    public PenguinsArm(HardwareMap hardwareMap) {
        Arm = hardwareMap.get(DcMotorEx.class, DRIVE_PARAMS.armName);
        Arm.setDirection(DRIVE_PARAMS.armDirection);

        Slide = hardwareMap.get(DcMotorEx.class, DRIVE_PARAMS.slideName);
        Slide.setDirection(DRIVE_PARAMS.slideDirection);

        // The arm will hold its position when given 0.0 power
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset the arm's encoder position
        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public class ArmSlideToPosition implements Action {
        // Use constructor parameter to set target position
        private int targetArmPosition;
        private int targetSlidePosition;
        public ArmSlideToPosition(int armPos, int slidePos) {
            super();
            targetArmPosition = armPos;
            targetSlidePosition = slidePos;
        }

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //double position = Arm.getCurrentPosition();
            //packet.put("Arm Position", position);

            if (!initialized) {
                Arm.setTargetPosition(targetArmPosition);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slide.setTargetPosition(targetSlidePosition);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            packet.put("Target Position", Arm.getTargetPosition());
            packet.put("Arm isBusy", Arm.isBusy());


            if (Arm.isBusy() || Slide.isBusy()) {
                // Returning true will run the action again
                Arm.setPower(0.8);
                Slide.setPower(0.8);
                return true;
            } else {
                // Returning false will end the action
                Arm.setPower(0);
                Slide.setPower(0);
                return false;
            }
        }
    }
    public Action ArmToPosition(int targetArmPosition, int targetSlidePosition) {
        return new ArmSlideToPosition(targetArmPosition, targetSlidePosition);
    }
}