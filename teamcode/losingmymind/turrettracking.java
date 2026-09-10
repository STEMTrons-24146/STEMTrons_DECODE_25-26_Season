//this works, do not touch it.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret Scan Lock Follow (FIXED)")
public class turrettracking extends LinearOpMode {

    private Limelight3A limelight;
    private CRServo turretLeft, turretRight;

    // ===== CONFIG =====
    private static final int TARGET_TAG_ID = 20;

    private static final double SCAN_POWER = 0.2;
    private static final double kP = 0.035;
    private static final double MAX_POWER = 0.2;
    private static final double DEADZONE = 0.0;

    private boolean trackingEnabled = false;
    private boolean lastAState = false;

    private boolean scanLeft = true;
    private long lastScanSwitchTime = 0;
    private static final long SCAN_SWITCH_MS = 2500;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretLeft = hardwareMap.get(CRServo.class, "servo1");
        turretRight = hardwareMap.get(CRServo.class, "servo2");

        limelight.pipelineSwitch(0);
        limelight.start(); // 🔴 REQUIRED

        telemetry.addLine("Press A to toggle tracking");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== BUTTON EDGE DETECT =====
            boolean aPressed = gamepad1.a;
            if (aPressed && !lastAState) {
                trackingEnabled = !trackingEnabled;
            }
            lastAState = aPressed;

            boolean tagVisible = false;
            double tx = 0.0;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        tagVisible = true;
                        tx = fid.getTargetXDegrees();
                        break;
                    }
                }
            }

            double turretPower = 0.0;

            if (trackingEnabled) {

                if (tagVisible) {
                    // ===== FOLLOW MODE =====
                    if (Math.abs(tx) > DEADZONE) {
                        turretPower = Range.clip(kP * tx, -MAX_POWER, MAX_POWER);
                    } else {
                        turretPower = 0.0;
                    }

                } else {
                    // ===== SCAN MODE =====
                    long now = System.currentTimeMillis();
                    if (now - lastScanSwitchTime > SCAN_SWITCH_MS) {
                        scanLeft = !scanLeft;
                        lastScanSwitchTime = now;
                    }
                    turretPower = scanLeft ? -SCAN_POWER : SCAN_POWER;
                }
            }

            turretLeft.setPower(turretPower);
            turretRight.setPower(turretPower);

            telemetry.addData("Tracking", trackingEnabled);
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addData("tx", tx);
            telemetry.addData("Turret Power", turretPower);
            telemetry.update();

            idle();
        }
    }
}
