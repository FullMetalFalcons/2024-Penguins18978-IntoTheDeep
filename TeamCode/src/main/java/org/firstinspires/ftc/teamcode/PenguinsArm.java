package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PenguinsArm {

    public static class FeedforwardGains{
        double ks = 0;
        double kgravity = 0;
        double kv = 0;
        double ka = 0;
        double timeToAccelerate = 0;

        /**
          * @param ks   Min power needed to move the arm without gravity
         * @param kgravity   Min power needed to move the arm with full gravity
         * @param kv   Power per tick/sec (slope)
         * @param ka   Power per tick/sec per sec
         * @param timeToAccelerate   Time to get to the desired velocity (for acceleration)
         */
        public FeedforwardGains(double ks, double kgravity, double kv, double ka, double timeToAccelerate){
            this.ks = ks;
            this.kgravity = kgravity;
            this.kv = kv;
            this.ka = ka;
            this.timeToAccelerate = timeToAccelerate;
        }
    }
    public static class Params {
        // accesory motors setup
        public String armName = "arm";
        public DcMotorSimple.Direction armDirection = DcMotorSimple.Direction.FORWARD;

        public String slideName = "slide";
        public  DcMotorSimple.Direction slideDirection = DcMotorSimple.Direction.REVERSE;

        public String hangerName = "linearActuator";
        public  DcMotorSimple.Direction hangerDirection = DcMotorSimple.Direction.REVERSE;

        public String clawName = "claw";

        public FeedforwardGains armGains = new FeedforwardGains(0.02, 0.0, 0.0004, 0.00007, 0.25);
        public FeedforwardGains slideGains = new FeedforwardGains(0.02, 0.0, 0.0004, 0.00007, 0.25);

        //This is used when deciding if the arm could exceed the robot length limit.  It's the max
        //  amount of time the arm could run before checking the length again
        public double TIME_MOTOR_CAN_RUN_BETWEEN_LOOPS_SEC = 0.1;
    }


    public static Params ARM_PARAMS = new Params();
    protected Telemetry telemetry;

    // Known Slide/Arm/Claw positions
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

    protected FalconsArmFeedforward armFeedforward = null;
    protected FalconsArmFeedforward slideFeedforward = null;


    public DcMotorEx Arm;
    public DcMotorEx Slide;
    public Servo Claw;
    public DcMotorEx Hanger;

    public PenguinsArm(HardwareMap hardwareMap, Telemetry telemetry1) {
        // Set up motors using MecanumDrive constants
        Arm = hardwareMap.get(DcMotorEx.class, ARM_PARAMS.armName);
        Arm.setDirection(ARM_PARAMS.armDirection);

        Slide = hardwareMap.get(DcMotorEx.class, ARM_PARAMS.slideName);
        Slide.setDirection(ARM_PARAMS.slideDirection);

        Claw = hardwareMap.get(Servo.class, ARM_PARAMS.clawName);

        Hanger = hardwareMap.get(DcMotorEx.class, ARM_PARAMS.hangerName);
        Hanger.setDirection(ARM_PARAMS.hangerDirection);

        // The arm will hold its position when given 0.0 power
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoder positions
        resetArmEncoder();
        resetSlideEncoder();
        resetHangerEncoder();

        // Allow the class to send data to telemetry
        telemetry = telemetry1;

        // Class to calculate the power needed to run the arm at a certain speed
        // taking into account things like gravity based on the angle
        armFeedforward = new FalconsArmFeedforward(ARM_PARAMS.armGains.ks, ARM_PARAMS.armGains.kgravity,
                                                   ARM_PARAMS.armGains.kv, ARM_PARAMS.armGains.ka,
                                                    FalconsArmFeedforward.MechanismType.ARM, telemetry, "Arm");

        slideFeedforward = new FalconsArmFeedforward(ARM_PARAMS.slideGains.ks, ARM_PARAMS.slideGains.kgravity,
                                                    ARM_PARAMS.slideGains.kv, ARM_PARAMS.slideGains.ka,
                                                    FalconsArmFeedforward.MechanismType.SLIDE, telemetry, "Slide");
    }

    public void resetArmEncoder(){
        // Reset the arm's encoder position
        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetSlideEncoder() {
        // Reset the slider's encoder position
        Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetHangerEncoder() {
        // Reset the hanger's encoder position
        Hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void setArmVelocity(double desiredVelocityTicksPerSec) {
        armAngleDeg = (Arm.getCurrentPosition() + INITIAL_ARM_ENCODER) * DEGREES_PER_ARM_TICK;

        // Bound the desired velocity to the arm's maximum possible velocity
        double maxVelocity = armFeedforward.maxAchievableVelocity(1.0, armAngleDeg, 0.0);
        desiredVelocityTicksPerSec = Math.max(Math.min(desiredVelocityTicksPerSec, maxVelocity), -maxVelocity);  //Limit result to a valid motor power

        // The max angle the arm could move in any loop is the max of the current speed
        //  and the desired speed and the max assumed loop time
        // TODO: Is this the right logic?
        // TODO: Can we easily figure out the max >0 and <0 of desired and actual speed?
        boolean canArmMove = getNewRobotLength(0.0,
                DEGREES_PER_ARM_TICK *
                        (desiredVelocityTicksPerSec * ARM_PARAMS.TIME_MOTOR_CAN_RUN_BETWEEN_LOOPS_SEC));

        if (desiredVelocityTicksPerSec != 0 && canArmMove) {
            // Set the motor power based on the power calculated from feedForward
            Arm.setPower(getMotorPower(desiredVelocityTicksPerSec, armFeedforward, ARM_PARAMS.armGains));
        } else {
            Arm.setPower(0.0);
        }

        telemetry.addData("ArmCanMove", canArmMove);
    }

    public void setSlideVelocity(double desiredVelocityTicksPerSec) {
        armAngleDeg = (Arm.getCurrentPosition() + INITIAL_ARM_ENCODER) * DEGREES_PER_ARM_TICK;

        // Bound the desired velocity to the arm's maximum possible velocity
        double maxVelocity = slideFeedforward.maxAchievableVelocity(1.0, armAngleDeg, 0.0);
        desiredVelocityTicksPerSec = Math.max(Math.min(desiredVelocityTicksPerSec, maxVelocity), -maxVelocity);  //Limit result to a valid motor power

        // The max angle the arm could move in any loop is the max of the current speed
        //  and the desired speed and the max assumed loop time
        // TODO: Is this the right logic?
        // TODO: Can we easily figure out the max >0 and <0 of desired and actual speed?
        boolean canSlideMove = getNewRobotLength(INCHES_PER_SLIDE_TICK *
                                                    (desiredVelocityTicksPerSec * ARM_PARAMS.TIME_MOTOR_CAN_RUN_BETWEEN_LOOPS_SEC),
                                                    0.0);

        if (desiredVelocityTicksPerSec != 0 && canSlideMove) {
            // Set the motor power based on the power calculated from feedForward
            Slide.setPower(getMotorPower(desiredVelocityTicksPerSec, slideFeedforward, ARM_PARAMS.slideGains));
        } else {
            Slide.setPower(0.0);
        }

        telemetry.addData("SlideCanMove", canSlideMove);
    }

    public double getMotorPower(double desiredVelocityTicksPerSec, FalconsArmFeedforward feedforward, FeedforwardGains gains) {
        // Update the Feedforward gains from the latest values which can be changed on the Dash
        feedforward.ks_motorPower = gains.ks;
        feedforward.kgravity_motorPower = gains.kgravity;
        feedforward.kv_motorPower_PerTickPerSec = gains.kv;
        feedforward.ka_motorPower_PerTickPerSec_PerSec = gains.ka;

        //Get the desired power based on the desired velocity and arm position
        double desiredPower = feedforward.calculateArmPower(armAngleDeg, desiredVelocityTicksPerSec);
        telemetry.addData(feedforward.instanceName+" FF Power", desiredPower);
        telemetry.addData(feedforward.instanceName+" FF Scaled to 1000", desiredPower*1000);

        return desiredPower;
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
        telemetry.addData("Arm Velocity TickPerSec", Arm.getVelocity());

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
        protected double targetClawPosition;
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
        protected int targetArmPositionTicks;
        protected int targetSlidePositionTicks;
        public ArmSlideToPosition(double armPosDegrees, double slidePosInches) {
            super();
            // Convert target degrees to target ticks
            targetArmPositionTicks = (int) (armPosDegrees/DEGREES_PER_ARM_TICK - INITIAL_ARM_ENCODER);

            // Convert target inches to target ticks
            targetSlidePositionTicks = (int) (slidePosInches/INCHES_PER_SLIDE_TICK);
        }

        protected boolean initialized = false;

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
                Arm.setPower(1);
                Slide.setPower(1);
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