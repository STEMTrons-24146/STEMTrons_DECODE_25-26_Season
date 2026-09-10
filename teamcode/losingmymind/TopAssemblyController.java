package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Top Assembly Controller")
public class TopAssemblyController extends LinearOpMode {

    // ===== CONFIG =====
    private static final int TARGET_TAG_ID = 20;

    // Turret
    private static final double SCAN_POWER = 0.3;
    private static final double kP = 0.035;
    private static final double MAX_POWER = 0.4;
    private static final double DEADZONE = 0.0;
    private boolean trackingEnabled = true;
    private boolean lastAState = false;
    private boolean scanLeft = true;
    private long lastScanSwitchTime = 0;
    private static final long SCAN_SWITCH_MS = 2000;

    // Hood
    private static final double[][] HOOD_TABLE = {
            {0.75, 0.82}, {1.00, 0.78}, {1.25, 0.73}, {1.50, 0.69}, {1.75, 0.65},
            {2.00, 0.60}, {2.25, 0.56}, {2.50, 0.52}, {2.75, 0.47}, {3.00, 0.42}
    };
    private static final double HOOD_MIN = 0.35;
    private static final double HOOD_MAX = 1.0;

    // Shooter
    private static final double[][] SHOOTER_TABLE = {
            {0.75, 0.60}, {1.00, 0.68}, {1.25, 0.76}, {1.50, 0.84}, {1.75, 0.92},
            {2.00, 1.00}, {2.25, 1.00}, {2.50, 1.00}, {2.75, 1.00}, {3.00, 1.00}
    };

    // Filters / scaling
    private static final double DIST_ALPHA = 0.4;
    private static final double DIST_SCALE = 1.6;

    // Hardware
    private Limelight3A limelight;
    private CRServo turretLeft, turretRight;
    private Servo hoodServo;
    private DcMotor shooterMotor;

    // Distance tracking
    private double rawDistance = 0.0;
    private double filteredDistance = 1.0;

    @Override
    public void runOpMode() {

        // Hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretLeft = hardwareMap.get(CRServo.class, "servo1");
        turretRight = hardwareMap.get(CRServo.class, "servo2");
        hoodServo = hardwareMap.get(Servo.class, "servo0");
        shooterMotor = hardwareMap.get(DcMotor.class, "motor7");
        
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Full Turret, Hood, Shooter Control Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== Button edge detection for turret tracking =====
            boolean aPressed = gamepad1.a;
            if (aPressed && !lastAState) {
                trackingEnabled = !trackingEnabled;
            }
            lastAState = aPressed;

            // ===== Limelight detection =====
            boolean tagVisible = false;
            double tx = 0.0;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        tagVisible = true;

                        // Turret
                        tx = fid.getTargetXDegrees();

                        // Distance for hood and shooter
                        Pose3D pose = fid.getTargetPoseCameraSpace();
                        double x = pose.getPosition().x;
                        double y = pose.getPosition().y;
                        double z = pose.getPosition().z;

                        rawDistance = Math.sqrt(x*x + y*y + z*z);
                        double correctedDistance = rawDistance * DIST_SCALE;
                        filteredDistance = (1 - DIST_ALPHA) * filteredDistance + DIST_ALPHA * correctedDistance;

                        // Hood servo
                        double hoodPos = getHoodPositionFromDistance(filteredDistance);
                        hoodServo.setPosition(hoodPos);

                        // Shooter power
                        double shooterPower = getShooterPowerFromDistance(filteredDistance);
                        shooterMotor.setPower(shooterPower);

                        break;
                    }
                }
            } else {
                shooterMotor.setPower(0); // Stop shooter if no tag
            }

            // ===== Turret control =====
            double turretPower = 0.0;
            if (trackingEnabled) {
                if (tagVisible) {
                    if (Math.abs(tx) > DEADZONE) {
                        turretPower = Range.clip(kP * tx, -MAX_POWER, MAX_POWER);
                    } else {
                        turretPower = 0.0;
                    }
                } else {
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

            // ===== Telemetry =====
            telemetry.addData("Tracking Enabled", trackingEnabled);
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addData("Turret tx", tx);
            telemetry.addData("Turret Power", turretPower);
            telemetry.addData("Raw Distance (m)", rawDistance);
            telemetry.addData("Filtered Distance (m)", filteredDistance);
            telemetry.addData("Hood Servo Pos", hoodServo.getPosition());
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.update();

            idle();
        }
    }

    // ===== Hood lookup interpolation =====
    private double getHoodPositionFromDistance(double distanceMeters) {
        if (distanceMeters <= HOOD_TABLE[0][0]) return HOOD_TABLE[0][1];
        if (distanceMeters >= HOOD_TABLE[HOOD_TABLE.length-1][0]) return HOOD_TABLE[HOOD_TABLE.length-1][1];

        for (int i=0; i<HOOD_TABLE.length-1; i++) {
            double d0 = HOOD_TABLE[i][0], d1 = HOOD_TABLE[i+1][0];
            if (distanceMeters >= d0 && distanceMeters <= d1) {
                double p0 = HOOD_TABLE[i][1], p1 = HOOD_TABLE[i+1][1];
                double t = (distanceMeters - d0)/(d1 - d0);
                return Range.clip(p0 + (p1 - p0)*t, HOOD_MIN, HOOD_MAX);
            }
        }
        return HOOD_TABLE[0][1];
    }

    // ===== Shooter lookup interpolation =====
    private double getShooterPowerFromDistance(double distanceMeters) {
        if (distanceMeters <= SHOOTER_TABLE[0][0]) return SHOOTER_TABLE[0][1];
        if (distanceMeters >= SHOOTER_TABLE[SHOOTER_TABLE.length-1][0]) return SHOOTER_TABLE[SHOOTER_TABLE.length-1][1];

        for (int i=0; i<SHOOTER_TABLE.length-1; i++) {
            double d0 = SHOOTER_TABLE[i][0], d1 = SHOOTER_TABLE[i+1][0];
            if (distanceMeters >= d0 && distanceMeters <= d1) {
                double p0 = SHOOTER_TABLE[i][1], p1 = SHOOTER_TABLE[i+1][1];
                double t = (distanceMeters - d0)/(d1 - d0);
                return Range.clip(p0 + (p1 - p0)*t, 0, 1.0);
            }
        }
        return SHOOTER_TABLE[0][1];
    }
}
