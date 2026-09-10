package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// Decode (BLUE) - competition TeleOp. One handle___() method per subsystem,
// called in the same order every loop.
@TeleOp(name = "Jarvis")
public class Jarvis extends LinearOpMode {

    // Hardware
    private DcMotor flDrive, frDrive, rlDrive, rrDrive;
    private DcMotor intakeMotor, transferMotor;
    private DcMotorEx spindexMotor, shooterMotor;
    private CRServo turretLeft, turretRight;
    private Servo hoodServo;
    private Limelight3A limelight;

    // Intake
    private boolean intakeOn = false;
    private boolean yWasPressed = false;

    // Spindex manual step
    private int spindexTargetTicks = 0;
    private static final int SPINDEX_STEP_TICKS = 128; // ticks between pockets
    private boolean rbWasPressed = false;

    // Spindex jam detection / auto recovery
    private static final int JAM_THRESHOLD_TICKS = 50;
    private static final int JAM_RECOVERY_BACKOFF_TICKS = 64;
    private static final double JAM_RECOVERY_COOLDOWN = 0.5;
    private boolean jamDetected = false;
    private boolean wasJammedLastLoop = false;
    private double lastJamRecoveryTime = -JAM_RECOVERY_COOLDOWN;
    private int spindexPositionError = 0;

    // Unjam - dpad down jiggle
    private boolean dpadDownWasPressed = false;
    private boolean jiggleActive = false;
    private double jiggleStartTime = 0;
    private boolean jiggleDirectionUp = false;
    private int jiggleHalfCycles = 0;
    private static final double JIGGLE_HALF_PERIOD = 0.15;
    private static final int JIGGLE_STEP_TICKS = 32;
    private static final int JIGGLE_MAX_HALF_CYCLES = 10;

    // Spindex smart align - right trigger
    private boolean rtWasPressed = false;

    // Manual single-ball push - left bumper toggle
    private boolean lbWasPressed = false;
    private boolean manualPushActive = false;
    private boolean manualRetracting = false;

    // Turret tracking (Limelight)
    private static final int TARGET_TAG_ID = 20;
    private static final double TURRET_kP = 0.035;
    private static final double TURRET_MAX_POWER = 0.4;
    private static final long TAG_STALE_MS = 250; // was 25ms - likely a typo, dropped nearly all tracking
    private long lastTagSeenTimeMs = 0;
    private double lastTagTx = 0.0;
    private boolean turretTrackingEnabled = true;
    private boolean xWasPressed = false;
    private double filteredDistanceMeters = 1.0;

    // Firing sequence
    private static final double SPIN_UP_TIME = 1.7;
    private static final double ARM_SETTLE_TIME = 1.0;
    private static final double TRANSFER_MOVE_TIME = 0.8; // safety timeout only, see
                                                          // transferPushComplete/transferAtHome
    private static final int TRANSFER_PUSH_TARGET_TICKS = -130;
    private static final int TRANSFER_HOME_TOLERANCE_TICKS = 20;
    private static final double HOOD_MIN = 0.35;
    private static final double HOOD_MAX = 1.0;
    private static final double TRANSFER_PUSH_POWER = -1.0;
    private static final double TRANSFER_RETRACT_POWER = 1.0;

    // distance(m), hood position, shooter ticks/sec - calibrated, don't reorder
    // columns
    private static final double[][] SHOT_TABLE = {
            { 0.58, 1.0, 1300 },
            { 0.69, 1.0, 1300 },
            { 0.8, 0.9, 1400 },
            { 0.9, 0.85, 1400 },
            { 1.0, 0.85, 1400 },
            { 1.1, 0.75, 1480 },
            { 1.2, 0.75, 1500 },
            { 1.3, 0.75, 1500 },
            { 1.4, 0.7, 1600 },
            { 1.5, 0.65, 1600 },
            { 1.6, 0.7, 1600 },
            { 1.75, 0.65, 1700 },
    };

    private enum FireStage {
        IDLE,
        SPIN_UP,
        SETTLE_BEFORE_PUSH_1, PUSH_1, RETRACT_1,
        SETTLE_BEFORE_PUSH_2, PUSH_2, RETRACT_2,
        SETTLE_BEFORE_PUSH_3, PUSH_3, RETRACT_3,
        FINAL_COOLDOWN
    }

    private boolean firing = false;
    private boolean ltWasPressed = false;
    private boolean dpadUpWasPressed = false;
    private FireStage fireStage = FireStage.IDLE;
    private double stageStartTime = 0;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            handleDrivetrain();
            boolean unjamActive = handleUnjamControls();
            boolean aligning = handleSpindexAlign();
            handleIntake(unjamActive);
            handleManualSpindexStep();
            handleManualPush();
            spindexMotor.setTargetPosition(spindexTargetTicks);
            handleJamDetection(unjamActive || aligning);
            handleTurretTracking();
            handleFiringSequence();
            updateTelemetry();
            idle();
        }
    }

    private void initHardware() {
        flDrive = hardwareMap.dcMotor.get("motor4");
        frDrive = hardwareMap.dcMotor.get("motor0");
        rlDrive = hardwareMap.dcMotor.get("motor5");
        rrDrive = hardwareMap.dcMotor.get("motor1");
        intakeMotor = hardwareMap.dcMotor.get("motor2");
        spindexMotor = hardwareMap.get(DcMotorEx.class, "motor7");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "motor3");
        transferMotor = hardwareMap.get(DcMotor.class, "motor6");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretLeft = hardwareMap.get(CRServo.class, "servo6");
        turretRight = hardwareMap.get(CRServo.class, "servo5");
        hoodServo = hardwareMap.get(Servo.class, "servo4");

        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // flip back if backwards the other way

        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setTargetPosition(0);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(650);

        // RUN_USING_ENCODER, driven by timed power below - not RUN_TO_POSITION,
        // which tapers power down near a target and weakens the push.
        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setPower(0);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    private void handleDrivetrain() {
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
            fl /= max;
            fr /= max;
            rl /= max;
            rr /= max;
        }

        flDrive.setPower(fl);
        frDrive.setPower(fr);
        rlDrive.setPower(rl);
        rrDrive.setPower(rr);
    }

    private void handleIntake(boolean unjamActive) {
        if (unjamActive)
            return;

        if (gamepad1.y && !yWasPressed)
            intakeOn = !intakeOn;
        yWasPressed = gamepad1.y;
        intakeMotor.setPower(intakeOn ? 1.0 : 0.0);
    }

    // Right bumper: spindex +1 pocket
    private void handleManualSpindexStep() {
        if (gamepad1.right_bumper && !rbWasPressed)
            stepSpindex(SPINDEX_STEP_TICKS);
        rbWasPressed = gamepad1.right_bumper;
    }

    // Left bumper: toggle - one press pushes one ball and auto-retracts,
    // spindex untouched. Does nothing while the LT auto-sequence is running.
    private void handleManualPush() {
        if (firing) {
            manualPushActive = false;
            manualRetracting = false;
            return;
        }

        if (gamepad1.left_bumper && !lbWasPressed && !manualPushActive && !manualRetracting) {
            manualPushActive = true;
        }
        lbWasPressed = gamepad1.left_bumper;

        if (manualPushActive) {
            applyShotTable();
            transferMotor.setPower(TRANSFER_PUSH_POWER);
            if (transferPushComplete()) {
                manualPushActive = false;
                manualRetracting = true;
            }
        } else if (manualRetracting) {
            transferMotor.setPower(TRANSFER_RETRACT_POWER);
            if (transferAtHome()) {
                transferMotor.setPower(0);
                shooterMotor.setVelocity(0);
                manualRetracting = false;
            }
        }
    }

    private boolean handleUnjamControls() {
        boolean unjamActive = false;

        if (gamepad1.a) {
            stepSpindex(-10);
            unjamActive = true;
        }

        if (gamepad1.b) {
            intakeMotor.setPower(-1.0);
            unjamActive = true;
        }

        if (gamepad1.dpad_down && !dpadDownWasPressed) {
            jiggleActive = !jiggleActive;
            if (jiggleActive) {
                jiggleStartTime = getRuntime();
                jiggleDirectionUp = false;
                jiggleHalfCycles = 0;
            }
        }
        dpadDownWasPressed = gamepad1.dpad_down;

        if (jiggleActive) {
            if (getRuntime() - jiggleStartTime > JIGGLE_HALF_PERIOD) {
                jiggleDirectionUp = !jiggleDirectionUp;
                jiggleStartTime = getRuntime();
                jiggleHalfCycles++;
                stepSpindex(jiggleDirectionUp ? JIGGLE_STEP_TICKS : -JIGGLE_STEP_TICKS);
                if (jiggleHalfCycles >= JIGGLE_MAX_HALF_CYCLES)
                    jiggleActive = false;
            }
            unjamActive = true;
        }

        return unjamActive;
    }

    // Right trigger, one-shot: snap spindex to the nearest real pocket from
    // its current position. Deliberate align step, done before LT - firing
    // no longer auto-corrects mid-sequence.
    private boolean handleSpindexAlign() {
        boolean rtPressed = gamepad1.right_trigger > 0.1;
        boolean justPressed = rtPressed && !rtWasPressed;
        rtWasPressed = rtPressed;

        if (justPressed && !firing) {
            spindexTargetTicks = (int) Math.round(spindexMotor.getCurrentPosition()
                    / (double) SPINDEX_STEP_TICKS) * SPINDEX_STEP_TICKS;
            return true;
        }
        return false;
    }

    // Edge-triggered: nudges back once per newly-detected jam, then cools
    // down, instead of re-correcting every loop and running away.
    private void handleJamDetection(boolean unjamActive) {
        spindexPositionError = Math.abs(spindexMotor.getCurrentPosition() - spindexTargetTicks);
        jamDetected = spindexPositionError > JAM_THRESHOLD_TICKS;

        boolean jamJustStarted = jamDetected && !wasJammedLastLoop;
        boolean cooldownElapsed = (getRuntime() - lastJamRecoveryTime) > JAM_RECOVERY_COOLDOWN;

        if (jamJustStarted && cooldownElapsed && !firing && !unjamActive) {
            spindexTargetTicks = spindexMotor.getCurrentPosition() - JAM_RECOVERY_BACKOFF_TICKS;
            spindexMotor.setTargetPosition(spindexTargetTicks);
            lastJamRecoveryTime = getRuntime();
        }

        wasJammedLastLoop = jamDetected;
    }

    private void handleTurretTracking() {
        if (gamepad1.x && !xWasPressed)
            turretTrackingEnabled = !turretTrackingEnabled;
        xWasPressed = gamepad1.x;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                if (fid.getFiducialId() == TARGET_TAG_ID) {
                    lastTagTx = fid.getTargetXDegrees();
                    lastTagSeenTimeMs = System.currentTimeMillis();
                    Pose3D pose = fid.getTargetPoseCameraSpace();
                    filteredDistanceMeters = pose.getPosition().z;
                    break;
                }
            }
        }

        boolean tagIsFresh = (System.currentTimeMillis() - lastTagSeenTimeMs) < TAG_STALE_MS;
        double turretPower = 0;
        if (turretTrackingEnabled && tagIsFresh) {
            turretPower = Range.clip(TURRET_kP * lastTagTx, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        }

        turretLeft.setPower(turretPower);
        turretRight.setPower(turretPower);
    }

    private void handleFiringSequence() {
        boolean ltPressed = gamepad1.left_trigger > 0.1;
        if (ltPressed && !ltWasPressed && !firing) {
            firing = true;
            fireStage = FireStage.SPIN_UP;
            stageStartTime = getRuntime();
        }
        ltWasPressed = ltPressed;

        boolean dpadUpPressed = gamepad1.dpad_up;
        if (dpadUpPressed && !dpadUpWasPressed && firing) {
            firing = false;
            fireStage = FireStage.IDLE;
            shooterMotor.setVelocity(0);
            transferMotor.setPower(0);
        }
        dpadUpWasPressed = dpadUpPressed;

        if (!firing) {
            if (!manualPushActive && !manualRetracting) {
                shooterMotor.setVelocity(0);
                transferMotor.setPower(0);
            }
            return;
        }

        applyShotTable();
        double elapsed = getRuntime() - stageStartTime;

        switch (fireStage) {
            case SPIN_UP:
                if (elapsed >= SPIN_UP_TIME) {
                    advanceStage(FireStage.SETTLE_BEFORE_PUSH_1);
                }
                break;

            case SETTLE_BEFORE_PUSH_1:
                if (elapsed >= ARM_SETTLE_TIME) {
                    transferMotor.setPower(TRANSFER_PUSH_POWER);
                    advanceStage(FireStage.PUSH_1);
                }
                break;
            case PUSH_1:
                if (transferPushComplete() || elapsed >= TRANSFER_MOVE_TIME) {
                    transferMotor.setPower(TRANSFER_RETRACT_POWER);
                    advanceStage(FireStage.RETRACT_1);
                }
                break;
            case RETRACT_1:
                if (transferAtHome() || elapsed >= TRANSFER_MOVE_TIME) {
                    transferMotor.setPower(0);
                    stepSpindex(SPINDEX_STEP_TICKS);
                    advanceStage(FireStage.SETTLE_BEFORE_PUSH_2);
                }
                break;

            case SETTLE_BEFORE_PUSH_2:
                if (elapsed >= ARM_SETTLE_TIME) {
                    transferMotor.setPower(TRANSFER_PUSH_POWER);
                    advanceStage(FireStage.PUSH_2);
                }
                break;
            case PUSH_2:
                if (transferPushComplete() || elapsed >= TRANSFER_MOVE_TIME) {
                    transferMotor.setPower(TRANSFER_RETRACT_POWER);
                    advanceStage(FireStage.RETRACT_2);
                }
                break;
            case RETRACT_2:
                if (transferAtHome() || elapsed >= TRANSFER_MOVE_TIME) {
                    transferMotor.setPower(0);
                    stepSpindex(SPINDEX_STEP_TICKS);
                    advanceStage(FireStage.SETTLE_BEFORE_PUSH_3);
                }
                break;

            case SETTLE_BEFORE_PUSH_3:
                if (elapsed >= ARM_SETTLE_TIME) {
                    transferMotor.setPower(TRANSFER_PUSH_POWER);
                    advanceStage(FireStage.PUSH_3);
                }
                break;
            case PUSH_3:
                if (transferPushComplete() || elapsed >= TRANSFER_MOVE_TIME) {
                    transferMotor.setPower(TRANSFER_RETRACT_POWER);
                    advanceStage(FireStage.RETRACT_3);
                }
                break;
            case RETRACT_3:
                if (transferAtHome() || elapsed >= TRANSFER_MOVE_TIME) {
                    transferMotor.setPower(0);
                    advanceStage(FireStage.FINAL_COOLDOWN); // no advance - already on a pocket
                }
                break;

            case FINAL_COOLDOWN:
                if (elapsed >= ARM_SETTLE_TIME) {
                    shooterMotor.setVelocity(0);
                    firing = false;
                    fireStage = FireStage.IDLE;
                }
                break;

            default:
                firing = false;
                fireStage = FireStage.IDLE;
                break;
        }
    }

    private void advanceStage(FireStage next) {
        fireStage = next;
        stageStartTime = getRuntime();
    }

    private boolean transferPushComplete() {
        return transferMotor.getCurrentPosition() <= TRANSFER_PUSH_TARGET_TICKS;
    }

    private boolean transferAtHome() {
        return Math.abs(transferMotor.getCurrentPosition()) <= TRANSFER_HOME_TOLERANCE_TICKS;
    }

    // Steps relative to the ACTUAL current position, not the last target, so
    // small under/overshoot doesn't stack up over repeated steps.
    private void stepSpindex(int deltaTicks) {
        spindexTargetTicks = spindexMotor.getCurrentPosition() + deltaTicks;
    }

    private void applyShotTable() {
        double hoodPos = SHOT_TABLE[0][1];
        double shooterTPS = SHOT_TABLE[0][2];
        double minDiff = Math.abs(filteredDistanceMeters - SHOT_TABLE[0][0]);

        for (int i = 1; i < SHOT_TABLE.length; i++) {
            double diff = Math.abs(filteredDistanceMeters - SHOT_TABLE[i][0]);
            if (diff < minDiff) {
                minDiff = diff;
                hoodPos = SHOT_TABLE[i][1];
                shooterTPS = SHOT_TABLE[i][2];
            }
        }

        hoodServo.setPosition(Range.clip(hoodPos, HOOD_MIN, HOOD_MAX));
        shooterMotor.setVelocity(Math.max(shooterTPS, 0));
    }

    private void updateTelemetry() {
        telemetry.addLine("--- SHOOTING ---");
        telemetry.addData("Distance (m)", "%.3f", filteredDistanceMeters);
        telemetry.addData("Fire Stage", fireStage);
        telemetry.addData("Shooter TPS", "%.1f", shooterMotor.getVelocity());
        telemetry.addData("Hood Pos", "%.2f", hoodServo.getPosition());
        telemetry.addData("Turret Tracking", turretTrackingEnabled ? "ON" : "OFF");
        telemetry.addData("Transfer Power", "%.2f", transferMotor.getPower());
        telemetry.addData("Transfer Ticks", transferMotor.getCurrentPosition());

        telemetry.addLine("--- SPINDEX ---");
        telemetry.addData("Target Ticks", spindexTargetTicks);
        telemetry.addData("Current Ticks", spindexMotor.getCurrentPosition());
        telemetry.addData("Position Error", spindexPositionError);
        telemetry.addData("Jam Detected", jamDetected ? "YES" : "no");
        telemetry.addData("Spindex Current (A)", "%.2f", spindexMotor.getCurrent(CurrentUnit.AMPS));

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Y", "Toggle intake");
        telemetry.addData("LT", "Fire (3 balls)");
        telemetry.addData("RT", "Smart align spindex");
        telemetry.addData("RB", "Spindex +1 pocket");
        telemetry.addData("LB", "Push 1 ball (toggle)");
        telemetry.addData("A (hold)", "Reverse spindex");
        telemetry.addData("B (hold)", "Reverse intake");
        telemetry.addData("Dpad Down", jiggleActive ? "JIGGLING" : "Jiggle spindex");
        telemetry.addData("Dpad Up", "Abort firing sequence");
        telemetry.addData("X", "Toggle turret tracking");

        telemetry.update();
    }
}