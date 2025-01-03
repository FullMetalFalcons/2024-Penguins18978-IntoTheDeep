package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Recreates the FTCLib ArmFeedforward so that we can:
 *      - Add some debugability to better understand what's happening
 *      - Allow the params to be changed so it can be tuned in real-time
 *
 * It also allows us to specify the units of the different params since the base class
 * just assumes the caller is consistent with units, but doesn't specify them.
 */
public class FalconsArmFeedforward {
    public double ks_motorPower;
    public double kgravity_motorPower;
    public double kv_motorPower_PerTickPerSec;
    public double ka_motorPower_PerTickPerSec_PerSec;
    public Telemetry telemetry;

    public String instanceName;
    public enum MechanismType {
        ARM,
        SLIDE
    }
    public MechanismType mechanismCategory;

    /**
     * Creates a new ArmFeedforward with the specified gains.
     *
     * @param ks_motorPower   The static motorPower reqd to get the arm to move with no gravity
     * @param kgravity_motorPower The additional motor power reqd to get the arm to move with full gravity
     * @param kv_motorPowerPerTickPerSec  The motor power
     * @param ka   The acceleration gain.
     * @param armCategory   The type of mechanism; arms are fully affected by gravity at 0 deg, slides at 90 deg
     * @param instanceName   The identifier used in telemetry statements to differentiate between instances
     */
    public FalconsArmFeedforward(double ks_motorPower, double kgravity_motorPower, double kv_motorPowerPerTickPerSec,
                                 double ka, MechanismType armCategory, Telemetry telemetry, String instanceName) {
        this.ks_motorPower = ks_motorPower;
        this.kgravity_motorPower = kgravity_motorPower;
        this.kv_motorPower_PerTickPerSec = kv_motorPowerPerTickPerSec;
        this.ka_motorPower_PerTickPerSec_PerSec = ka;
        this.mechanismCategory = armCategory;
        this.telemetry = telemetry;
        this.instanceName = instanceName;
    }

    /**
     * Creates a new ArmFeedforward with the specified gains.
     *
     * @param ks_motorPower   The static motorPower reqd to get the arm to move with no gravity
     * @param kgravity_motorPower The additional motor power reqd to get the arm to move with full gravity
     * @param kv_motorPowerPerTickPerSec  The motor power to increase the velocity (i.e. the slope)
     * @param armCategory   The type of mechanism; arms are fully affected by gravity at 0 deg, slides at 90 deg
     * @param instanceName   The identifier used in telemetry statements to differentiate between instances
     */
    public FalconsArmFeedforward(double ks_motorPower, double kgravity_motorPower, double kv_motorPowerPerTickPerSec,
                                 MechanismType armCategory, Telemetry telemetry, String instanceName) {
        this(ks_motorPower, kgravity_motorPower, kv_motorPowerPerTickPerSec, 0, armCategory, telemetry, instanceName);
    }

    /**
     * Calculates the feedforward motoPower required to achieve the desired arm velocity at the
     * current arm position.
     *
     * @param velocityTicksPerSec     The velocity setpoint.
     * @param positionDegrees The acceleration setpoint.
     * @return The computed feedforward motor power.
     */
    public double calculateArmPower(double positionDegrees, double velocityTicksPerSec,
                                    double accelTicksPerSecSquared) {
        double motorPowerFromKs = ks_motorPower * Math.signum(velocityTicksPerSec);
        double motorPowerFromKgravity = getKgravityPower(positionDegrees);
        double motorPowerFromKv = kv_motorPower_PerTickPerSec * velocityTicksPerSec;
        double motorPowerFromKa = ka_motorPower_PerTickPerSec_PerSec * accelTicksPerSecSquared;

        double motorPower = motorPowerFromKs + motorPowerFromKgravity + motorPowerFromKv + motorPowerFromKa;

        if (telemetry != null){
            telemetry.addData(instanceName+"MotorPowerKs", motorPowerFromKs);
            telemetry.addData(instanceName+"MotorPowerKgravity", motorPowerFromKgravity);
            telemetry.addData(instanceName+"MotorPowerKv", motorPowerFromKv);
            telemetry.addData(instanceName+"MotorPowerKa", motorPowerFromKa);
            telemetry.addData(instanceName+"MotorPowerTotal", motorPower);
        }

        return motorPower;
    }

    /**
     * Calculates the feedforward motoPower required to achieve the desired arm velocity at the
     * current arm position.
     *
     * @param velocityTicksPerSec     The velocity setpoint.
     * @param positionDegrees The acceleration setpoint.
     * @return The computed feedforward motor power.
     */
    public double calculateArmPower(double positionDegrees, double velocityTicksPerSec) {
        return calculateArmPower(positionDegrees, velocityTicksPerSec, 0);
    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the maximum achievable velocity given a maximum voltage supply,
     * a position, and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angleDegrees The angle of the arm in degrees.
     * @param acceleration The acceleration of the arm.
     * @return The maximum possible velocity at the given acceleration and angle.
     */
    public double maxAchievableVelocity(double maxVoltage, double angleDegrees, double acceleration) {
        // Assume max velocity is positive
        return (maxVoltage - ks_motorPower - getKgravityPower(angleDegrees) - acceleration * ka_motorPower_PerTickPerSec_PerSec) / kv_motorPower_PerTickPerSec;
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage supply,
     * a position, and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angleDegrees The angle of the arm in degrees.
     * @param acceleration The acceleration of the arm.
     * @return The minimum possible velocity at the given acceleration and angle.
     */
    public double minAchievableVelocity(double maxVoltage, double angleDegrees, double acceleration) {
        // Assume min velocity is negative, ks flips sign
        return (-maxVoltage + ks_motorPower - getKgravityPower(angleDegrees) - acceleration * ka_motorPower_PerTickPerSec_PerSec) / kv_motorPower_PerTickPerSec;
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage
     * supply, a position, and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angleDegrees The angle of the arm in degrees.
     * @param velocity     The velocity of the arm.
     * @return The maximum possible acceleration at the given velocity.
     */
    public double maxAchievableAcceleration(double maxVoltage, double angleDegrees, double velocity) {
        return (maxVoltage - ks_motorPower * Math.signum(velocity) - getKgravityPower(angleDegrees) - velocity * kv_motorPower_PerTickPerSec) / ka_motorPower_PerTickPerSec_PerSec;
    }

    /**
     * Calculates the minimum achievable acceleration given a maximum voltage
     * supply, a position, and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angleDegrees The angle of the arm.
     * @param velocity     The velocity of the arm.
     * @return The minimum possible acceleration at the given velocity.
     */
    public double minAchievableAcceleration(double maxVoltage, double angleDegrees, double velocity) {
        return maxAchievableAcceleration(-maxVoltage, angleDegrees, velocity);
    }

    private double getKgravityPower(double positionDegrees) {
        double motorPowerFromKgravity;
        // If the feedfoward belongs to an arm, gravity has full effect in the 0 deg (down) position
        //   If it belongs to a slide, gravity has full effect in the 90 deg (straight up) position
        if (mechanismCategory == MechanismType.ARM) {
            motorPowerFromKgravity = kgravity_motorPower * Math.cos(Math.toRadians(positionDegrees));
        } else {
            motorPowerFromKgravity = kgravity_motorPower * Math.sin(Math.toRadians(positionDegrees));
        }
        return motorPowerFromKgravity;
    }

}
