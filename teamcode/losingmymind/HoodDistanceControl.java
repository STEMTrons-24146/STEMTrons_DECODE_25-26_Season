package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Hood Distance Control 0.25m Steps")
public class HoodDistanceControl extends LinearOpMode {

    // =============================
    // HOOD LOOKUP TABLE (distance → servo)
    // LOW servo = HIGH hood angle
    // servo = 1.0 fully retracted
    // =============================
    private static final double[][] HOOD_TABLE = {
        {0.75, 0.82},
        {1.00, 0.78},
        {1.25, 0.73},
        {1.50, 0.69},
        {1.75, 0.65},
        {2.00, 0.60},
        {2.25, 0.56},
        {2.50, 0.52},
        {2.75, 0.47},
        {3.00, 0.42}
    };

    // Safety limits
    private static final double HOOD_MIN = 0.35;
    private static final double HOOD_MAX = 1.0;

    // Distance smoothing
    private static final double DIST_ALPHA = 0.2;

    // Scaling factor to correct Limelight measurement
    private static final double DIST_SCALE = 1.6;

    // Target AprilTag
    private static final int TARGET_TAG_ID = 20;

    // =============================

    private Limelight3A limelight;
    private Servo hoodServo;

    private double rawDistance = 0.0;
    private double filteredDistance = 0.0;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hoodServo = hardwareMap.get(Servo.class, "servo0");

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Hood Distance Control (Debug)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean tagVisible = false;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {

                    if (fid.getFiducialId() == TARGET_TAG_ID) {

                        tagVisible = true;

                        Pose3D pose = fid.getTargetPoseCameraSpace();
                        double x = pose.getPosition().x;
                        double y = pose.getPosition().y;
                        double z = pose.getPosition().z;

                        // Compute raw distance
                        rawDistance = Math.sqrt(x*x + y*y + z*z);

                        // Apply scale factor to correct camera geometry
                        double correctedDistance = rawDistance * DIST_SCALE;

                        // Low-pass filter for smooth movement
                        filteredDistance = (1 - DIST_ALPHA) * filteredDistance + DIST_ALPHA * correctedDistance;

                        // Update hood servo continuously
                        double hoodPos = getHoodPositionFromDistance(filteredDistance);
                        hoodServo.setPosition(hoodPos);

                        break;
                    }
                }
            }

            // ================= DEBUG TELEMETRY =================
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addData("Raw Distance (m)", rawDistance);
            telemetry.addData("Filtered Distance (m)", filteredDistance);
            telemetry.addData("Hood Servo Pos", hoodServo.getPosition());
            telemetry.update();

            idle();
        }
    }

    // =============================
    // Map filtered distance to hood servo using lookup table
    // =============================
    private double getHoodPositionFromDistance(double distanceMeters) {

        // Below table range
        if (distanceMeters <= HOOD_TABLE[0][0]) {
            return HOOD_TABLE[0][1];
        }

        // Above table range
        if (distanceMeters >= HOOD_TABLE[HOOD_TABLE.length - 1][0]) {
            return HOOD_TABLE[HOOD_TABLE.length - 1][1];
        }

        // Find surrounding points for interpolation
        for (int i = 0; i < HOOD_TABLE.length - 1; i++) {
            double d0 = HOOD_TABLE[i][0];
            double d1 = HOOD_TABLE[i + 1][0];

            if (distanceMeters >= d0 && distanceMeters <= d1) {
                double p0 = HOOD_TABLE[i][1];
                double p1 = HOOD_TABLE[i + 1][1];
                double t = (distanceMeters - d0) / (d1 - d0);
                return Range.clip(lerp(p0, p1, t), HOOD_MIN, HOOD_MAX);
            }
        }

        // Fallback
        return HOOD_TABLE[0][1];
    }

    // Linear interpolation helper
    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
