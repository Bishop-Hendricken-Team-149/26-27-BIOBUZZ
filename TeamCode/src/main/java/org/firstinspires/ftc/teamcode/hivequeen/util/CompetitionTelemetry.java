package org.firstinspires.ftc.teamcode.hivequeen.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hivequeen.RobotConstants;

/**
 * R704.D compliance helper.
 *
 * Manual TU07 R704 "Use networks and bandwidth as directed." ...
 *  A. "Teams may not use any other form of wireless communication, except those offered by the
 *     provided official tools, to communicate to, from, or within the ROBOT."
 *  D. "Software with access to the ROBOT CONTROLLER Wi-Fi network must limit the amount of data
 *     being streamed (i.e., continuous transmission of data) over Wi-Fi. Software may only stream
 *     robot control data, debugging data, and telemetry to and from the ROBOT using the FTC Driver
 *     Station Application. Additional logging/streaming services, such as those hosted by third
 *     party plugins and tools such as FTC Dashboard, FTControl Panels, and others are prohibited.
 *     No continuous video stream is allowed."
 *
 * How this codebase complies:
 *  - The ONLY telemetry path is the SDK {@link Telemetry} object (FTC Driver Station app).
 *    No FTC Dashboard / Panels / sockets / HTTP / Bluetooth anywhere in TeamCode, and no extra
 *    dependencies were added to TeamCode/build.gradle.
 *  - The DS transmission interval is pinned to {@link RobotConstants#TELEMETRY_INTERVAL_MS}
 *    (250 ms = 4 Hz) and messages are kept to a few short lines.
 *  - The webcam VisionPortal is built with live view disabled and is never streamed; HuskyLens
 *    and Pinpoint are wired I2C devices (HuskyLens is explicitly allowed as a configurable,
 *    non-programmable vision coprocessor, R702 Example 5).
 */
public final class CompetitionTelemetry {
    private CompetitionTelemetry() { }

    /** Call once in every OpMode's init. */
    public static void configure(Telemetry telemetry) {
        telemetry.setMsTransmissionInterval(RobotConstants.TELEMETRY_INTERVAL_MS);
        telemetry.setAutoClear(true);
    }
}
