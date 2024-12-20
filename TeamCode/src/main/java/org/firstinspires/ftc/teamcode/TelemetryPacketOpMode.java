package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * A TelemetryPacket class is provided to RoadRunner objects (like Actions) so they
 * can provide telemetry data during autonomous.
 * This class overrides that class to allow a TelemetryPacket to be provided to
 * objects outside of autonomous (like during OpMode).  When this class is used
 * it will just add the data to the regular base telemetry object.
 */
public class TelemetryPacketOpMode extends TelemetryPacket {
    Telemetry telemetry;
    TelemetryPacketOpMode(Telemetry baseTelemetry){
        super();
        telemetry = baseTelemetry;
    }

    @Override
    public void put(String key, Object value) {
        telemetry.addData(key, value);
    }
}
