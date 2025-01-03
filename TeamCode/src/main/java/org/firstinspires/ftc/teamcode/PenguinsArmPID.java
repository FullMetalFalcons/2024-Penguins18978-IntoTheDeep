package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PenguinsArmPID extends PenguinsArm{
    protected FalconsPIDFController armPidController = new FalconsPIDFController(0,0,0,0, telemetry, "arm");
    protected FalconsPIDFController slidePidController = new FalconsPIDFController(0,0,0,0, telemetry, "slide");

    /** When setting the motor's velocity, which method should be used:
     */
    public enum ArmPidMode{
        /** Use the built in DcMotorEx.runToPosition function in the base class
         */
        RUN_TO_POSITION_EC_MOTOR,
        /** Use the custom FalconsArmPID class
         */
        FALCONS_PID
    }

    public static class PIDFParams {
        //Arm max speed is about 2000 ticks/sec, it doesn't do too well under 200
        //  so a P of 10 will
        //     run at full speed till within 200 (~3.5dec)
        //     run at speed of 50 at the tolerance (too low)
        public double arm_p = 7;
        public double arm_i = 0.01;
        public double arm_d = 0;
        public double arm_f = 0;
        public double arm_tolerance = 5;

        public double slide_p = 10;
        public double slide_i = 0.01;
        public double slide_d = 0;
        public double slide_f = 0;
        public double slide_tolerance = 5;

        public ArmPidMode armPidMode = ArmPidMode.FALCONS_PID;
    }
    public static PIDFParams PIDF_PARAMS = new PIDFParams();

    public PenguinsArmPID(HardwareMap hardwareMap, Telemetry telemetry1) {
        super(hardwareMap, telemetry1);
    }

    public class ArmSlideToPositionPID extends ArmSlideToPosition {
        // Use constructor parameter to set target position
        public ArmSlideToPositionPID(double armPosDegrees, double slidePosInches) {
            super(armPosDegrees, slidePosInches);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armPidController.reset();  //This clears any accumulated errors in I
                armPidController.setSetPoint(targetArmPositionTicks);

                slidePidController.reset();
                slidePidController.setSetPoint(targetSlidePositionTicks);

                initialized = true;
            }
            armPidController.setPIDF(PIDF_PARAMS.arm_p, PIDF_PARAMS.arm_i, PIDF_PARAMS.arm_d, PIDF_PARAMS.arm_f);
            armPidController.setTolerance(PIDF_PARAMS.arm_tolerance);

            slidePidController.setPIDF(PIDF_PARAMS.slide_p, PIDF_PARAMS.slide_i, PIDF_PARAMS.slide_d, PIDF_PARAMS.slide_f);
            slidePidController.setTolerance(PIDF_PARAMS.slide_tolerance);

            //Technically being at the set point can change as the arm bounces
            //  so read it once per loop to be consistent
            boolean armAtSetPoint = armPidController.atSetPoint();
            boolean slideAtSetPoint = slidePidController.atSetPoint();

            /*
             * The loop checks to see if the controller has reached
             * the desired setpoint within a specified tolerance
             * range
             */
            if (armAtSetPoint) {
                //We are at the target, return isBusy = false (i.e. we are done)
                setArmVelocity(0);
            } else {
                double armOutputV = armPidController.calculate(
                        Arm.getCurrentPosition());  // the measured value
                setArmVelocity(armOutputV);
            }

            if (slideAtSetPoint) {
                setSlideVelocity(0);
            } else {
                double slideOutputV = slidePidController.calculate(
                        Slide.getCurrentPosition());  // the measured value
                setSlideVelocity(slideOutputV);
            }

            packet.put("Target Arm Position", targetArmPositionTicks);
            packet.put("Actual Arm Position", Arm.getCurrentPosition());
            packet.put("Arm atSetPoint", armAtSetPoint);
            packet.put("Target Slide Position", targetSlidePositionTicks);
            packet.put("Actual Slide Position", Slide.getCurrentPosition());
            packet.put("Slide atSetPoint", slideAtSetPoint);

            //Return true if we are still busy (which is if either PID is not at the set point)
            return (!armAtSetPoint || !slideAtSetPoint);
        }
    }
    public ArmSlideToPosition armToPosition(double targetArmDegrees, double targetSlideInches) {
        if (PIDF_PARAMS.armPidMode == ArmPidMode.FALCONS_PID){
            return new ArmSlideToPositionPID(targetArmDegrees, targetSlideInches);
        }else{
            return new ArmSlideToPosition(targetArmDegrees, targetSlideInches);
        }
    }
}