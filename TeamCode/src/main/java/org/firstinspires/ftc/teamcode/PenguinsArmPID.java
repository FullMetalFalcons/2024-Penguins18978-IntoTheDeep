package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PenguinsArmPID extends PenguinsArm{
    protected PIDController armPidController = new PIDController(0,0,0);
    protected PIDController slidePidController = new PIDController(0,0,0);

    public PenguinsArmPID(HardwareMap hardwareMap, Telemetry telemetry1) {
        super(hardwareMap, telemetry1);
    }

    public class ArmSlideToPositionPID extends ArmSlideToPosition {
        // Use constructor parameter to set target position
        private int targetArmPositionTicks;
        private int targetSlidePositionTicks;
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
            packet.put("Target Arm Position", targetArmPositionTicks);
            packet.put("Actual Arm Position", Arm.getTargetPosition());
            packet.put("Arm isBusy", Arm.isBusy());
            packet.put("Target Slide Position", targetSlidePositionTicks);
            packet.put("Actual Slide Position", Slide.getTargetPosition());
            packet.put("Slide isBusy", Slide.isBusy());

            /*
             * The loop checks to see if the controller has reached
             * the desired setpoint within a specified tolerance
             * range
             */
            double armOutputV = armPidController.calculate(
                    Arm.getCurrentPosition());  // the measured value
            Arm.setVelocity(armOutputV);

            double slideOutpuV = armPidController.calculate(
                    Slide.getCurrentPosition());  // the measured value
            Slide.setVelocity(slideOutpuV);

            if (armPidController.atSetPoint()) {
                //We are at the target, return isBusy = false (i.e. we are done)
                Arm.setVelocity(0); // TODO: Is this needed or will it be 0 from above?
            }

            if (slidePidController.atSetPoint()) {
                Slide.setVelocity(0); // TODO: Is this needed or will it be 0 from above?
            }

            //Return true if we are still busy (which is if either PID is not at the set point)
            return (!armPidController.atSetPoint() || !slidePidController.atSetPoint());
        }
    }
    public ArmSlideToPosition armToPosition(double targetArmDegrees, double targetSlideInches) {
        return new ArmSlideToPositionPID(targetArmDegrees, targetSlideInches);
    }
}