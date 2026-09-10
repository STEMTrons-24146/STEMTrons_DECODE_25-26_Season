package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Telopithicalytely")
public class Telopithicalytely extends LinearOpMode {

    // -----------------
    // Hardware
    // -----------------
    DcMotor flDrive, frDrive, rlDrive, rrDrive;
    DcMotor intake, armMotor, shooterMotor, transfer;
    CRServo turretLeft, turretRight;
    Servo hoodServo;
    Limelight3A limelight;

    // -----------------
    // Controls / State
    // -----------------
    boolean intakeOn = false, yPressedLast = false;
    int armTargetTicks = 0;
    final int ARM_STEP_TICKS = 128;
    boolean lbWasPressed = false, rbWasPressed = false;

    private static final int TARGET_TAG_ID = 20;

    // Turret Tracking
    private static final double kP = 0.018;       // proportional gain for smooth tracking
    private static final double MAX_POWER = 0.26; // max turret power
    private static final double DEADBAND = 0.4;   // ignore tiny errors
    private long lastTagSeenTime = 0;
    private double lastTx = 0.0;
    private boolean trackingEnabled = true;
    private boolean xWasPressed = false;

    // Distance
    private double filteredDistance = 1.0;

    // Firing State Machine
    private boolean firing = false, ltWasPressed = false;
    private int fireStage = 0;
    private double stageStart = 0;

    // Constants
    private static final double SPIN_UP_TIME = 1.7;
    private static final double ARM_COOLDOWN = 0.7;
    private static final double TRANSFER_MOVE_TIME = 0.4;
    private static final double HOOD_MIN = 0.35;
    private static final double HOOD_MAX = 1.0;

    @Override
    public void runOpMode() {

        // -----------------
        // Hardware Mapping
        // -----------------
        flDrive = hardwareMap.dcMotor.get("motor4");
        frDrive = hardwareMap.dcMotor.get("motor0");
        rlDrive = hardwareMap.dcMotor.get("motor5");
        rrDrive = hardwareMap.dcMotor.get("motor1");
        intake = hardwareMap.dcMotor.get("motor2");
        armMotor = hardwareMap.get(DcMotor.class, "motor7");
        shooterMotor = hardwareMap.get(DcMotor.class, "motor3");
        transfer = hardwareMap.get(DcMotor.class, "motor6");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretLeft = hardwareMap.get(CRServo.class, "servo6");
        turretRight = hardwareMap.get(CRServo.class, "servo5");
        hoodServo = hardwareMap.get(Servo.class, "servo4");

        // -----------------
        // Motor Setup
        // -----------------
        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(650);

        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setTargetPosition(0);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(0.8);

        // -----------------
        // Limelight Settings
        // -----------------
        limelight.pipelineSwitch(8); // pipeline 8 for QR / AprilTag
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

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                  Math.max(Math.abs(rl), Math.abs(rr)));

            if (max > 1.0) {
                fl /= max; fr /= max; rl /= max; rr /= max;
            }

            flDrive.setPower(fl);
            frDrive.setPower(fr);
            rlDrive.setPower(rl);
            rrDrive.setPower(rr);

            // -----------------
            // Intake
            // -----------------
            if (gamepad1.y && !yPressedLast)
                intakeOn = !intakeOn;
            yPressedLast = gamepad1.y;
            intake.setPower(intakeOn ? 1.0 : 0.0);

            // -----------------
            // Manual Spindex
            // -----------------
            if (gamepad1.left_bumper && !lbWasPressed)
                armTargetTicks += ARM_STEP_TICKS;
            lbWasPressed = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !rbWasPressed)
                armTargetTicks += ARM_STEP_TICKS / 2;
            rbWasPressed = gamepad1.right_bumper;

            armMotor.setTargetPosition(armTargetTicks);

            // -----------------
            // Limelight Tracking + Distance
            // -----------------
            if (gamepad1.x && !xWasPressed)
                trackingEnabled = !trackingEnabled;
            xWasPressed = gamepad1.x;

            LLResult result = limelight.getLatestResult();
            boolean tagVisible = false;
            double rawTx = 0;

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        rawTx = fid.getTargetXDegrees();
                        lastTagSeenTime = System.currentTimeMillis();
                        tagVisible = true;

                        Pose3D pose = fid.getTargetPoseCameraSpace();
                        filteredDistance = pose.getPosition().z;
                        break;
                    }
                }
            }

            // -----------------
            // Turret Control (smooth version)
            // -----------------
            double turretPower = 0;

            if (trackingEnabled && tagVisible) {
                double error = rawTx;

                if (Math.abs(error) > DEADBAND) {
                    turretPower = kP * error;
                    turretPower = Range.clip(turretPower, -MAX_POWER, MAX_POWER);
                } else {
                    turretPower = 0;
                }
            } else {
                turretPower = 0;
            }

            turretLeft.setPower(turretPower);
            turretRight.setPower(turretPower);

            // -----------------
            // Fire State Machine
            // -----------------
            boolean ltPressed = gamepad1.left_trigger > 0.1;

            if (ltPressed && !ltWasPressed && !firing) {
                firing = true;
                fireStage = 0;
                stageStart = getRuntime();
            }

            ltWasPressed = ltPressed;

            if (firing) {
                ShooterCalculator.ShooterState state =
                        ShooterCalculator.calculate(filteredDistance);

                double hoodPos = Range.clip(state.hoodPosition, HOOD_MIN, HOOD_MAX);
                double shooterTPS = Math.max(state.shooterSpeed, 0);

                hoodServo.setPosition(hoodPos);
                ((DcMotorEx) shooterMotor).setVelocity(shooterTPS);

                double elapsed = getRuntime() - stageStart;

                switch (fireStage) {
                    case 0: if (elapsed >= SPIN_UP_TIME) { fireStage=1; stageStart=getRuntime(); } break;
                    case 1: if (elapsed >= ARM_COOLDOWN) { transfer.setTargetPosition(-116); fireStage=2; stageStart=getRuntime(); } break;
                    case 2: if (elapsed >= TRANSFER_MOVE_TIME) { transfer.setTargetPosition(0); fireStage=3; stageStart=getRuntime(); } break;
                    case 3: if (elapsed >= TRANSFER_MOVE_TIME) { armTargetTicks += 128; fireStage=4; stageStart=getRuntime(); } break;
                    case 4: if (elapsed >= ARM_COOLDOWN) { transfer.setTargetPosition(-116); fireStage=5; stageStart=getRuntime(); } break;
                    case 5: if (elapsed >= TRANSFER_MOVE_TIME) { transfer.setTargetPosition(0); fireStage=6; stageStart=getRuntime(); } break;
                    case 6: if (elapsed >= TRANSFER_MOVE_TIME) { armTargetTicks += 128; fireStage=7; stageStart=getRuntime(); } break;
                    case 7: if (elapsed >= ARM_COOLDOWN) { transfer.setTargetPosition(-116); fireStage=8; stageStart=getRuntime(); } break;
                    case 8: if (elapsed >= TRANSFER_MOVE_TIME) { transfer.setTargetPosition(0); fireStage=9; stageStart=getRuntime(); } break;
                    case 9: if (elapsed >= TRANSFER_MOVE_TIME) { armTargetTicks += 64; fireStage=10; stageStart=getRuntime(); } break;
                    case 10: if (elapsed >= ARM_COOLDOWN) { ((DcMotorEx) shooterMotor).setVelocity(0); firing=false; } break;
                }
            } else {
                ((DcMotorEx) shooterMotor).setVelocity(0);
                transfer.setTargetPosition(0);
            }

            // -----------------
            // Telemetry
            // -----------------
            telemetry.addData("Distance (m)", "%.3f", filteredDistance);
            telemetry.addData("Shooter TPS", "%.1f", ((DcMotorEx) shooterMotor).getVelocity());
            telemetry.addData("Hood Pos", "%.2f", hoodServo.getPosition());
            telemetry.addData("Turret Tracking", trackingEnabled);
            telemetry.addData("Turret Power", turretPower);
            telemetry.addData("Firing Stage", fireStage);
            telemetry.update();

            idle();
        }
    }
}
