package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Shooter Distance Control")
public class ShooterDistControl extends LinearOpMode {

    private static final int TARGET_TAG_ID = 20;

    // Distance smoothing
    private static final double DIST_ALPHA = 0.2;

    // Scale factor for Limelight distance
    private static final double DIST_SCALE = 1.6;

    // Shooter lookup table: distance (m) -> motor power
    private static final double[][] SHOOTER_TABLE = {
            {0.75, 0.60},
            {1.00, 0.68},
            {1.25, 0.76},
            {1.50, 0.84},
            {1.75, 0.92},
            {2.00, 1.00},
            {2.25, 1.00},
            {2.50, 1.00},
            {2.75, 1.00},
            {3.00, 1.00}
    };

    private Limelight3A limelight;
    private DcMotor shooterMotor;

    private double rawDistance = 0.0;
    private double filteredDistance = 0.0;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        shooterMotor = hardwareMap.get(DcMotor.class, "motor0");

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Shooter Distance Control Ready");
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

                        // raw distance
                        rawDistance = Math.sqrt(x*x + y*y + z*z);

                        // apply scale factor
                        double correctedDistance = rawDistance * DIST_SCALE;

                        // low-pass filter
                        filteredDistance = (1 - DIST_ALPHA) * filteredDistance + DIST_ALPHA * correctedDistance;

                        // determine shooter power
                        double power = getShooterPowerFromDistance(filteredDistance);
                        shooterMotor.setPower(power);

                        break;
                    }
                }
            } else {
                // No tag visible, stop motor
                shooterMotor.setPower(0);
            }

            // telemetry
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addData("Raw Distance (m)", rawDistance);
            telemetry.addData("Filtered Distance (m)", filteredDistance);
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.update();

            idle();
        }
    }

    // interpolation from lookup table
    private double getShooterPowerFromDistance(double distanceMeters) {

        // below table
        if (distanceMeters <= SHOOTER_TABLE[0][0]) {
            return SHOOTER_TABLE[0][1];
        }

        // above table
        if (distanceMeters >= SHOOTER_TABLE[SHOOTER_TABLE.length - 1][0]) {
            return SHOOTER_TABLE[SHOOTER_TABLE.length - 1][1];
        }

        // interpolate
        for (int i = 0; i < SHOOTER_TABLE.length - 1; i++) {
            double d0 = SHOOTER_TABLE[i][0];
            double d1 = SHOOTER_TABLE[i+1][0];
            if (distanceMeters >= d0 && distanceMeters <= d1) {
                double p0 = SHOOTER_TABLE[i][1];
                double p1 = SHOOTER_TABLE[i+1][1];
                double t = (distanceMeters - d0)/(d1 - d0);
                return Range.clip(p0 + (p1-p0)*t, 0, 1.0);
            }
        }

        return SHOOTER_TABLE[0][1];
    }
}
