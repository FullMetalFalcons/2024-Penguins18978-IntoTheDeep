package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PenguinsArm {
    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();
    private Telemetry telemetry;

    // Known Slide/Arm/Claw positions
    public final double HOLD_POSITION = Double.MAX_VALUE;

    public final double ARM_RESET_DEGREES = 11;
    public final double SLIDE_RESET_INCHES = 0;

    public final double ARM_SPECIMEN_READY_DEGREES = 53;
    public final double SLIDE_SPECIMEN_READY_INCHES = 23;

    public final double ARM_SPECIMEN_SCORE_DEGREES = 43;  //49
    public final double SLIDE_SPECIMEN_SCORE_INCHES = 17;

    public final double CLAW_OPEN = 0.3;
    public final double CLAW_CLOSED = 0.6;

    // Encoder storage variables for arm limits
    double slideLengthInches = 0;
    double INITIAL_SLIDE_LENGTH_INCHES = 16.0;
    final double INCHES_PER_SLIDE_TICK = 0.00830154812;

    double armAngleDeg = 0;
    final double INITIAL_ARM_ENCODER = 600;
    final double DEGREES_PER_ARM_TICK = 0.018326206475;
    final double MAX_ARM_ANGLE_DEGREES = 90.0;

    // The amount that the claw adds onto the robot's length
    double clawLengthAdditionalInches = 0.0;
    // The physical length of the claw itself (from the bottom of the viper slide)
    final double CLAW_LENGTH_INCHES = 9.0;

    double currentRobotLengthInches = 0.0;
    final double INITIAL_ROBOT_LENGTH_INCHES = 18.0;

    // The amount of added length due to the offset viper slide
    double parallelSlideOffsetInches = 0.0;
    // The distance between the center of the linear actuator and the bottom of the viper slide
    final double PARALLEL_SLIDE_DIFFERENCE_INCHES = 3.0;

    final double MAX_ROBOT_LENGTH_INCHES = 42.0;

    // Constants for how much anticipatory length/angle should be added when attempting to move a motor
    final double ABSOLUTE_DELTA_LENGTH_INCHES = 3.0;
    final double ABSOLUTE_DELTA_ANGLE_DEGREES = 7.0;


    private DcMotorEx Arm;
    private DcMotorEx Slide;
    private Servo Claw;
    public PenguinsArm(HardwareMap hardwareMap, Telemetry telemetry1) {
        // Set up motors using MecanumDrive constants
        Arm = hardwareMap.get(DcMotorEx.class, DRIVE_PARAMS.armName);
        Arm.setDirection(DRIVE_PARAMS.armDirection);

        Slide = hardwareMap.get(DcMotorEx.class, DRIVE_PARAMS.slideName);
        Slide.setDirection(DRIVE_PARAMS.slideDirection);

        Claw = hardwareMap.get(Servo.class, DRIVE_PARAMS.clawName);

        // The arm will hold its position when given 0.0 power
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset the arm's encoder position
        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Allow the class to send data to telemetry
        telemetry = telemetry1;
    }


    // Method to check whether the robot will still be within size constraints after the desired movements
    public boolean getNewRobotLength(double deltaLengthInInches, double deltaAngleDeg) {
        slideLengthInches = INITIAL_SLIDE_LENGTH_INCHES + (Slide.getCurrentPosition() * INCHES_PER_SLIDE_TICK);
        slideLengthInches += deltaLengthInInches;

        armAngleDeg = (Arm.getCurrentPosition() + INITIAL_ARM_ENCODER) * DEGREES_PER_ARM_TICK;
        armAngleDeg += deltaAngleDeg;

        parallelSlideOffsetInches = Math.cos(Math.toRadians(90 - armAngleDeg)) * PARALLEL_SLIDE_DIFFERENCE_INCHES;

        currentRobotLengthInches = parallelSlideOffsetInches
                + (INITIAL_ROBOT_LENGTH_INCHES - INITIAL_SLIDE_LENGTH_INCHES)
                + (Math.cos(Math.toRadians(armAngleDeg)) * slideLengthInches);
        clawLengthAdditionalInches = Math.sin(Math.toRadians(armAngleDeg)) * CLAW_LENGTH_INCHES;
        currentRobotLengthInches += clawLengthAdditionalInches;

        // Don't let the arm go too far back
        if (armAngleDeg > MAX_ARM_ANGLE_DEGREES) {
            return false;
        } else {
            // Determine if the robot is within its size limit
            return (currentRobotLengthInches < MAX_ROBOT_LENGTH_INCHES);
        }
    }


    public void setArmPower(double desiredPower) {
        Arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        if (getNewRobotLength(0.0, ABSOLUTE_DELTA_ANGLE_DEGREES * desiredPower)) {
            Arm.setPower(desiredPower);
        } else {
            Arm.setPower(0.0);
        }
    }

    public void setSlidePower(double desiredPower) {
        Slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        if (getNewRobotLength(ABSOLUTE_DELTA_LENGTH_INCHES * desiredPower, 0.0)) {
            Slide.setPower(desiredPower);
        } else {
            Slide.setPower(0.0);
        }
    }

    public void setClawPosition(double desiredPosition) {
        Claw.setPosition(desiredPosition);
    }


    public void addDebugData() {
        // Encoder telemetry
        telemetry.addData("Arm Pos", Arm.getCurrentPosition());
        telemetry.addData("Slide Pos", Slide.getCurrentPosition());

        getNewRobotLength(0,0);
        telemetry.addData("Arm Angle", armAngleDeg);
        telemetry.addData("Slide Length", slideLengthInches - INITIAL_SLIDE_LENGTH_INCHES);
        telemetry.addData("Claw Length", clawLengthAdditionalInches);
        telemetry.addData("Robot Length", currentRobotLengthInches);

        PIDFCoefficients armPID = Arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("P", armPID.p);
        telemetry.addData("I", armPID.i);
        telemetry.addData("D", armPID.d);
        telemetry.addData("F", armPID.f);
    }


    public class ClawToPosition implements Action {
        // Use constructor parameter to set target position
        private double targetClawPosition;
        public ClawToPosition(double clawPos) {
            super();
            targetClawPosition = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(targetClawPosition);
            return false;
        }
    }
    public ClawToPosition clawToPosition(double clawPos) {
        return new ClawToPosition(clawPos);
    }


    public class ArmSlideToPosition implements Action {
        // Use constructor parameter to set target position
        private int targetArmPositionTicks;
        private int targetSlidePositionTicks;
        public ArmSlideToPosition(double armPosDegrees, double slidePosInches) {
            super();
            if (armPosDegrees == HOLD_POSITION) {
                // The arm should not move
                targetArmPositionTicks = Arm.getCurrentPosition();
            } else {
                // Convert target degrees to target ticks
                targetArmPositionTicks = (int) (armPosDegrees/DEGREES_PER_ARM_TICK - INITIAL_ARM_ENCODER);
            }

            if (slidePosInches == HOLD_POSITION) {
                // The slide should not move
                targetSlidePositionTicks = Slide.getCurrentPosition();
            } else {
                // Convert target inches to target ticks
                targetSlidePositionTicks = (int) (slidePosInches/INCHES_PER_SLIDE_TICK);
            }
        }

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //double position = Arm.getCurrentPosition();
            //packet.put("Arm Position", position);

            /*
                armPID = Arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                slidePID = Slide.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                armPID.p = arm_p;
                armPID.i = arm_i;
                armPID.d = arm_d;

                slidePID.p = slide_p;
                slidePID.i = slide_i;
                slidePID.d = slide_d;
                Arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, armPID);
                Slide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, slidePID);
             */

            if (!initialized) {
                Arm.setTargetPosition(targetArmPositionTicks);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slide.setTargetPosition(targetSlidePositionTicks);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            packet.put("Target Arm Position", targetArmPositionTicks);
            packet.put("Actual Arm Position", Arm.getTargetPosition());
            packet.put("Arm isBusy", Arm.isBusy());
            packet.put("Target Slide Position", targetSlidePositionTicks);
            packet.put("Actual Slide Position", Slide.getTargetPosition());
            packet.put("Slide isBusy", Slide.isBusy());

            //TODO Add limit checks
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
    public ArmSlideToPosition armToPosition(double targetArmDegrees, double targetSlideInches) {
        return new ArmSlideToPosition(targetArmDegrees, targetSlideInches);
    }
}