package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

@TeleOp(name = "Jarvis")
public class Jarvis1 extends LinearOpMode {

    // -------- HARDWARE --------
    DcMotor flDrive, frDrive, rlDrive, rrDrive;
    CRServo turretLeft, turretRight;
    Limelight3A limelight;

    // -------- STATE --------
    boolean trackingEnabled = true;
    boolean xWasPressed = false;

    private static final int TARGET_TAG_ID = 20;

    // -------- TUNING --------
    private static final double kP = 0.018;       // proportional gain
    private static final double MAX_POWER = 0.26; // max turret power
    private static final double DEADBAND = 0.4;  // ignore tiny errors
    private static final double MIN_ERROR = 0.0;  // minimum error to move (dynamic scaling)
    
    @Override
    public void runOpMode() {

        // -------- HARDWARE MAP --------
        flDrive = hardwareMap.dcMotor.get("motor4");
        frDrive = hardwareMap.dcMotor.get("motor0");
        rlDrive = hardwareMap.dcMotor.get("motor5");
        rrDrive = hardwareMap.dcMotor.get("motor1");

        turretLeft = hardwareMap.get(CRServo.class, "servo6");
        turretRight = hardwareMap.get(CRServo.class, "servo5");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // -------- LIMELIGHT SETTINGS --------
        limelight.pipelineSwitch(8);  // pipeline 8 for QR / AprilTag
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            // -------- DRIVE --------
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double fl = y + x + rx;
            double fr = y - x - rx;
            double rl = y - x + rx;
            double rr = y + x - rx;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                  Math.max(Math.abs(rl), Math.abs(rr)));

            if (max > 1.0) {
                fl /= max; fr /= max; rl /= max; rr /= max;
            }

            flDrive.setPower(fl);
            frDrive.setPower(fr);
            rlDrive.setPower(rl);
            rrDrive.setPower(rr);

            // -------- TRACK TOGGLE --------
            if (gamepad1.x && !xWasPressed)
                trackingEnabled = !trackingEnabled;
            xWasPressed = gamepad1.x;

            // -------- LIMELIGHT TRACKING --------
            LLResult result = limelight.getLatestResult();
            boolean tagVisible = false;
            double rawTx = 0;

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        rawTx = fid.getTargetXDegrees();
                        tagVisible = true;
                        break;
                    }
                }
            }

            // -------- TURRET CONTROL --------
            double turretPower = 0;

            if (trackingEnabled && tagVisible) {
                double error = rawTx;

                if (Math.abs(error) > DEADBAND) {
                    // Linear scaling
                    turretPower = kP * error;
                    turretPower = Range.clip(turretPower, -MAX_POWER, MAX_POWER);
                } else {
                    turretPower = 0; // within deadband, stop
                }

            } else {
                turretPower = 0; // tag lo2
            }

            turretLeft.setPower(turretPower);
            turretRight.setPower(turretPower);

            // -------- TELEMETRY --------
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addData("Raw Tx", rawTx);
            telemetry.addData("Turret Power", turretPower);
            telemetry.update();

            idle();
        }
    }
}
