package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "blake rev L (Simple)")
public class decodesimple extends LinearOpMode {

    // =====================
    // Drivetrain
    // =====================
    DcMotor flDrive, frDrive, rlDrive, rrDrive;

    // =====================
    // Spindex
    // =====================
    DcMotor armMotor;
    int armTargetTicks = 0;
    boolean lbLast = false;
    boolean rbLast = false;

    private static final int FULL_STEP = 128;
    private static final int HALF_STEP = 64;

    // =====================
    // Shooter Velocity
    // =====================
    DcMotorEx shooterMotor;
    double shooterSetTPS = 0;

    boolean rStickLast = false;
    boolean lStickLast = false;

    // =====================
    // Transfer
    // =====================
    DcMotor transfer;
    private static final int TRANSFER_UP_TICKS = -120;

    private static final double TRANSFER_TIME = 0.4;
    private static final double ARM_SETTLE = 0.7;

    private boolean firing = false;
    private boolean ltLast = false;
    private int fireStage = 0;
    private double stageStart = 0;

    // =====================
    // Hood
    // =====================
    Servo hoodServo;

    private static final double HOOD_MIN = 0.35;
    private static final double HOOD_MAX = 1.0;
    private static final double HOOD_STEP = 0.05;

    private double hoodTarget = 1.0;

    private boolean aLast = false;
    private boolean bLast = false;

    // =====================
    // Limelight
    // =====================
    Limelight3A limelight;
    private static final int TARGET_TAG_ID = 20;
    double rawDistance = 0.0;

    @Override
    public void runOpMode() {

        // Hardware
        flDrive = hardwareMap.dcMotor.get("motor4");
        frDrive = hardwareMap.dcMotor.get("motor0");
        rlDrive = hardwareMap.dcMotor.get("motor5");
        rrDrive = hardwareMap.dcMotor.get("motor1");

        armMotor = hardwareMap.get(DcMotor.class, "motor7");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "motor3");
        transfer = hardwareMap.get(DcMotor.class, "motor6");

        hoodServo = hardwareMap.get(Servo.class, "servo4");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Spindex setup
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(550);

        // Shooter setup
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Transfer setup
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setTargetPosition(0);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(0.6);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            // -----------------
            // Drivetrain
            // -----------------
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double fl = y + x + rx;
            double fr = y - x - rx;
            double rl = y - x + rx;
            double rr = y + x - rx;

            double max = Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(rl), Math.abs(rr))
            );
            if (max > 1.0) {
                fl /= max; fr /= max; rl /= max; rr /= max;
            }

            flDrive.setPower(fl);
            frDrive.setPower(fr);
            rlDrive.setPower(rl);
            rrDrive.setPower(rr);

            // -----------------
            // Spindex Control
            // -----------------
            if (gamepad1.left_bumper && !lbLast) {
                armTargetTicks += FULL_STEP;
            }
            if (gamepad1.right_bumper && !rbLast) {
                armTargetTicks += HALF_STEP;
            }
            lbLast = gamepad1.left_bumper;
            rbLast = gamepad1.right_bumper;

            armMotor.setTargetPosition(armTargetTicks);

            // -----------------
            // Hood Control (A/B)
            // -----------------
            boolean aPressed = gamepad1.a;
            boolean bPressed = gamepad1.b;

            if (bPressed && !bLast) hoodTarget += HOOD_STEP;
            if (aPressed && !aLast) hoodTarget -= HOOD_STEP;

            aLast = aPressed;
            bLast = bPressed;

            hoodTarget = Range.clip(hoodTarget, HOOD_MIN, HOOD_MAX);
            hoodServo.setPosition(hoodTarget);

            // -----------------
            // Shooter TPS Tuning (Stick Press)
            // -----------------
            boolean rStick = gamepad1.right_stick_button;
            boolean lStick = gamepad1.left_stick_button;

            if (rStick && !rStickLast) shooterSetTPS += 100;
            if (lStick && !lStickLast) shooterSetTPS -= 100;

            rStickLast = rStick;
            lStickLast = lStick;

            if (shooterSetTPS < 0) shooterSetTPS = 0;

            shooterMotor.setVelocity(shooterSetTPS);
            double actualTPS = shooterMotor.getVelocity();

            // -----------------
            // Limelight Raw Distance
            // -----------------
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        Pose3D pose = fid.getTargetPoseCameraSpace();
                        rawDistance = pose.getPosition().z;
                        break;
                    }
                }
            }

            // -----------------
            // Firing Sequence
            // -----------------
            boolean ltPressed = gamepad1.left_trigger > 0.1;

            if (ltPressed && !ltLast && !firing) {
                firing = true;
                fireStage = 0;
                stageStart = getRuntime();
            }
            ltLast = ltPressed;

            if (firing) {
                double elapsed = getRuntime() - stageStart;

                switch (fireStage) {

                    case 0:
                        transfer.setTargetPosition(TRANSFER_UP_TICKS);
                        if (elapsed >= TRANSFER_TIME) {
                            fireStage = 1;
                            stageStart = getRuntime();
                        }
                        break;

                    case 1:
                        transfer.setTargetPosition(0);
                        if (elapsed >= TRANSFER_TIME) {
                            armTargetTicks += FULL_STEP;
                            fireStage = 2;
                            stageStart = getRuntime();
                        }
                        break;

                    case 2:
                        if (elapsed >= ARM_SETTLE) {
                            firing = false;
                        }
                        break;
                }
            }

            // -----------------
            // Telemetry
            // -----------------
            telemetry.addData("Hood Position", hoodTarget);
            telemetry.addData("Shooter Set TPS", shooterSetTPS);
            telemetry.addData("Shooter Actual TPS", actualTPS);
            telemetry.addData("Raw Distance (m)", rawDistance);
            telemetry.update();

            idle();
        }
    }
}
