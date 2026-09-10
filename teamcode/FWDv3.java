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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Decode (BLUE) - competition TeleOp.
 *
 * Layout:
 * 1. Hardware fields & tuning constants, grouped by subsystem
 * 2. runOpMode() - init, then one handle___() call per subsystem
 * per loop, always in the same order
 * 3. One method per subsystem below, in the order they're called
 */
@TeleOp(name = "FWDv3")
public class FWDv3 extends LinearOpMode {

    // =========================================================
    // Hardware
    // =========================================================
    private DcMotor flDrive, frDrive, rlDrive, rrDrive;
    private DcMotor intakeMotor;
    private DcMotorEx spindexMotor, shooterMotor, transferMotor;
    private CRServo turretLeft, turretRight;
    private Servo hoodServo;
    private Limelight3A limelight;

    // =========================================================
    // Intake
    // =========================================================
    private boolean intakeOn = false;
    private boolean yWasPressed = false;

    // =========================================================
    // Spindex - manual stepping
    // =========================================================
    private int spindexTargetTicks = 0;
    private static final int SPINDEX_STEP_TICKS = 128; // ticks between pockets
    private boolean lbWasPressed = false, rbWasPressed = false;

    // =========================================================
    // Spindex - jam detection / auto recovery
    // =========================================================
    private static final int JAM_THRESHOLD_TICKS = 50;
    private static final int JAM_RECOVERY_BACKOFF_TICKS = 64;
    private static final double JAM_RECOVERY_COOLDOWN = 0.5; // seconds between auto-recovery attempts
    private boolean jamDetected = false;
    private boolean wasJammedLastLoop = false;
    private double lastJamRecoveryTime = -JAM_RECOVERY_COOLDOWN;
    private int spindexPositionError = 0;

    // =========================================================
    // Manual unjam controls
    // =========================================================
    private boolean dpadDownWasPressed = false;
    private boolean jiggleActive = false;
    private double jiggleStartTime = 0;
    private boolean jiggleDirectionUp = false;
    private int jiggleHalfCycles = 0;
    private static final double JIGGLE_HALF_PERIOD = 0.15; // seconds
    private static final int JIGGLE_STEP_TICKS = 32;
    private static final int JIGGLE_MAX_HALF_CYCLES = 10;

    // =========================================================
    // Turret tracking (Limelight)
    // =========================================================
    private static final int TARGET_TAG_ID = 20;
    private static final double TURRET_kP = 0.035;
    private static final double TURRET_MAX_POWER = 0.4;
    // NOTE: this was 25ms in the old code, which is likely a typo - at typical
    // loop speeds that basically requires seeing the tag on *every single loop*
    // or tracking drops out. 250ms is a more normal "still fresh" window.
    // Flagging this: change it back if 25ms was actually intentional.
    private static final long TAG_STALE_MS = 250;
    // Below this error, snap to zero instead of commanding a tiny correction.
    // Without this, per-frame noise in tx keeps producing small nonzero
    // powers forever, so the turret constantly hunts/creeps in a limit cycle
    // instead of ever truly stopping. Tune by watching how far off-center it
    // sits once "stopped" - shrink this if that's more than you want.
    private static final double TURRET_DEADBAND_DEGREES = 1.5;
    private long lastTagSeenTimeMs = 0;
    private double lastTagTx = 0.0;
    private boolean turretTrackingEnabled = true;
    private boolean xWasPressed = false;
    private double filteredDistanceMeters = 1.0;

    // =========================================================
    // Firing sequence
    // =========================================================
    private static final double SPIN_UP_TIME = 1.7;
    private static final double ARM_SETTLE_TIME = 1.0; // time for spindex to finish a move before pushing
    private static final double TRANSFER_PUSH_TIME = 0.4;
    // A fixed retract duration can't tell "fully home" from "stalled halfway,
    // still fighting resistance" - it just cuts power when the clock runs out
    // either way, which is why bumping the number up alone didn't fix a
    // half-retracted arm. Instead we now watch the motor's own velocity: once
    // it's basically not turning anymore (while still under retract power),
    // that means it's hit the hard stop, i.e. actually home, and that's the
    // real signal to cut power. TRANSFER_RETRACT_TIME is kept only as a safety
    // cap in case something is genuinely jammed and stall is never detected.
    private static final double TRANSFER_RETRACT_TIME = 1.0; // hard safety cap
    private static final double TRANSFER_RETRACT_MIN_TIME = 0.15; // ignore stall check during initial ramp-up
    private static final double TRANSFER_STALL_VELOCITY_TICKS_PER_SEC = 15; // tune: velocity below this counts as "not
                                                                            // moving"
    private static final int TRANSFER_STALL_DEBOUNCE_LOOPS = 3; // consecutive low-velocity loops required before
                                                                // declaring stalled
    private static final double HOOD_MIN = 0.35;
    private static final double HOOD_MAX = 1.0;
    // Driven as timed full-power strokes, not RUN_TO_POSITION targets - see
    // initHardware() for why: RUN_TO_POSITION tapers power down near the
    // target, so the push loses force right as it's contacting the ball.
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
    private FireStage fireStage = FireStage.IDLE;
    private double stageStartTime = 0;
    private int transferLowVelocityLoopCount = 0;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            handleDrivetrain();
            boolean unjamActive = handleUnjamControls();
            handleIntake(unjamActive);
            handleManualSpindexStep();
            spindexMotor.setTargetPosition(spindexTargetTicks);
            handleJamDetection(unjamActive);
            handleTurretTracking();
            handleFiringSequence();
            updateTelemetry();
            idle();
        }
    }

    // =========================================================
    // Init
    // =========================================================
    private void initHardware() {
        flDrive = hardwareMap.dcMotor.get("motor4");
        frDrive = hardwareMap.dcMotor.get("motor0");
        rlDrive = hardwareMap.dcMotor.get("motor5");
        rrDrive = hardwareMap.dcMotor.get("motor1");
        intakeMotor = hardwareMap.dcMotor.get("motor2");
        spindexMotor = hardwareMap.get(DcMotorEx.class, "motor7");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "motor3");
        transferMotor = hardwareMap.get(DcMotorEx.class, "motor6");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretLeft = hardwareMap.get(CRServo.class, "servo6");
        turretRight = hardwareMap.get(CRServo.class, "servo5");
        hoodServo = hardwareMap.get(Servo.class, "servo4");

        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake was spinning the wrong way - reversed here. If it's now
        // backwards the OTHER direction, change REVERSE to FORWARD.
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setTargetPosition(0);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(650);

        // RUN_USING_ENCODER (not RUN_TO_POSITION): we drive this as timed full-power
        // strokes below, not a position target, so it doesn't decelerate on approach.
        // RUN_USING_ENCODER also means if the ball resists it, the controller pushes
        // back with more power to hold the commanded speed, instead of just stalling.
        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setPower(0);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // =========================================================
    // Drivetrain - mecanum, single stick + turn
    // =========================================================
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

    // =========================================================
    // Intake
    // =========================================================
    private void handleIntake(boolean unjamActive) {
        if (unjamActive)
            return; // unjam controls own the intake motor this cycle

        if (gamepad1.y && !yWasPressed)
            intakeOn = !intakeOn;
        yWasPressed = gamepad1.y;
        intakeMotor.setPower(intakeOn ? 1.0 : 0.0);
    }

    // =========================================================
    // Manual spindex stepping (bumpers)
    // =========================================================
    private void handleManualSpindexStep() {
        if (gamepad1.left_bumper && !lbWasPressed)
            stepSpindex(SPINDEX_STEP_TICKS);
        lbWasPressed = gamepad1.left_bumper;

        if (gamepad1.right_bumper && !rbWasPressed)
            stepSpindex(SPINDEX_STEP_TICKS / 2);
        rbWasPressed = gamepad1.right_bumper;
    }

    // =========================================================
    // Manual unjam controls. Returns true if any unjam input is
    // active this cycle - callers should not also drive intake
    // through its normal path when this is true.
    // =========================================================
    private boolean handleUnjamControls() {
        boolean unjamActive = false;

        // A: reverse spindex only
        if (gamepad1.a) {
            stepSpindex(-10);
            unjamActive = true;
        }

        // B: reverse intake only
        if (gamepad1.b) {
            intakeMotor.setPower(-1.0);
            unjamActive = true;
        }

        // Dpad down: toggle jiggle mode (alternating small spindex moves)
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

        // Dpad up: reverse both intake and spindex together
        if (gamepad1.dpad_up) {
            intakeMotor.setPower(-1.0);
            stepSpindex(-10);
            unjamActive = true;
        }

        // Dpad left: emergency reset spindex to 0
        if (gamepad1.dpad_left) {
            spindexTargetTicks = 0;
            unjamActive = true;
        }

        return unjamActive;
    }

    // =========================================================
    // Jam detection - edge-triggered auto-recovery.
    //
    // OLD BUG: this used to re-apply a -64 tick correction on EVERY loop
    // (50x/sec) for as long as the jam persisted, with nothing to stop it -
    // so the target ran away further and further instead of settling. That's
    // almost certainly why "unjamming doesn't work." Now it nudges back once
    // when a jam is newly detected, then waits out a cooldown before it will
    // trigger again.
    // =========================================================
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

    // =========================================================
    // Turret tracking (Limelight)
    // =========================================================
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
        if (turretTrackingEnabled && tagIsFresh && Math.abs(lastTagTx) >= TURRET_DEADBAND_DEGREES) {
            turretPower = Range.clip(TURRET_kP * lastTagTx, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        }

        turretLeft.setPower(turretPower);
        turretRight.setPower(turretPower);
    }

    // =========================================================
    // Firing sequence
    // =========================================================
    private void handleFiringSequence() {
        boolean ltPressed = gamepad1.left_trigger > 0.1;
        if (ltPressed && !ltWasPressed && !firing) {
            firing = true;
            fireStage = FireStage.SPIN_UP;
            stageStartTime = getRuntime();
        }
        ltWasPressed = ltPressed;

        if (!firing) {
            shooterMotor.setVelocity(0);
            transferMotor.setPower(0);
            return;
        }

        applyShotTable();
        double elapsed = getRuntime() - stageStartTime;

        switch (fireStage) {
            case SPIN_UP:
                if (elapsed >= SPIN_UP_TIME) {
                    correctHalfStepDrift();
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
                if (elapsed >= TRANSFER_PUSH_TIME) {
                    transferMotor.setPower(TRANSFER_RETRACT_POWER);
                    transferLowVelocityLoopCount = 0;
                    advanceStage(FireStage.RETRACT_1);
                }
                break;
            case RETRACT_1:
                if (transferRetractComplete(elapsed)) {
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
                if (elapsed >= TRANSFER_PUSH_TIME) {
                    transferMotor.setPower(TRANSFER_RETRACT_POWER);
                    transferLowVelocityLoopCount = 0;
                    advanceStage(FireStage.RETRACT_2);
                }
                break;
            case RETRACT_2:
                if (transferRetractComplete(elapsed)) {
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
                if (elapsed >= TRANSFER_PUSH_TIME) {
                    transferMotor.setPower(TRANSFER_RETRACT_POWER);
                    transferLowVelocityLoopCount = 0;
                    advanceStage(FireStage.RETRACT_3);
                }
                break;
            case RETRACT_3:
                if (transferRetractComplete(elapsed)) {
                    transferMotor.setPower(0);
                    // No advance here on purpose: after 2 advances (shot 1 -> shot 2 ->
                    // shot 3) the spindex is already sitting on a pocket. Advancing again
                    // would land on a wall instead - that was the old bug.
                    advanceStage(FireStage.FINAL_COOLDOWN);
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

    /**
     * True once the transfer's retract stroke should be considered finished:
     * either it's genuinely stopped moving (stalled against its home stop,
     * i.e. actually home) or the safety-cap time has run out regardless.
     * TRANSFER_RETRACT_MIN_TIME skips the check right at the start of the
     * stroke, since the motor is still ramping up from zero velocity there
     * and would otherwise look "stalled" immediately.
     */
    private boolean transferRetractComplete(double elapsed) {
        if (elapsed >= TRANSFER_RETRACT_TIME) {
            return true;
        }
        if (elapsed < TRANSFER_RETRACT_MIN_TIME) {
            transferLowVelocityLoopCount = 0;
            return false;
        }
        if (Math.abs(transferMotor.getVelocity()) < TRANSFER_STALL_VELOCITY_TICKS_PER_SEC) {
            transferLowVelocityLoopCount++;
        } else {
            transferLowVelocityLoopCount = 0;
        }
        return transferLowVelocityLoopCount >= TRANSFER_STALL_DEBOUNCE_LOOPS;
    }

    /**
     * Step the spindex target relative to where it ACTUALLY is right now,
     * not relative to the last commanded target. If a step under/overshoots
     * by a few ticks - normal backlash, well under the jam threshold - this
     * keeps that small error from stacking up over repeated steps.
     */
    private void stepSpindex(int deltaTicks) {
        spindexTargetTicks = spindexMotor.getCurrentPosition() + deltaTicks;
    }

    /**
     * If the spindex is sitting half a pocket off, snap it into alignment.
     * Does NOT touch it if it's already correctly aligned on a pocket -
     * that was the old bug that put a wall on the transfer instead of a
     * pocket right as it fired.
     */
    private void correctHalfStepDrift() {
        double steps = spindexTargetTicks / (double) SPINDEX_STEP_TICKS;
        double frac = steps - Math.floor(steps);
        if (Math.abs(frac - 0.5) <= 0.1) {
            spindexTargetTicks += SPINDEX_STEP_TICKS / 2;
        }
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

    // =========================================================
    // Telemetry
    // =========================================================
    private void updateTelemetry() {
        telemetry.addLine("--- SHOOTING ---");
        telemetry.addData("Distance (m)", "%.3f", filteredDistanceMeters);
        telemetry.addData("Fire Stage", fireStage);
        telemetry.addData("Shooter TPS", "%.1f", shooterMotor.getVelocity());
        telemetry.addData("Hood Pos", "%.2f", hoodServo.getPosition());
        telemetry.addData("Turret Tracking", turretTrackingEnabled ? "ON" : "OFF");
        telemetry.addData("Turret Error (deg)", "%.2f", lastTagTx);
        telemetry.addData("Transfer Power", "%.2f", transferMotor.getPower());
        telemetry.addData("Transfer Velocity (t/s)", "%.1f", transferMotor.getVelocity());
        telemetry.addData("Transfer Low-Vel Loops", transferLowVelocityLoopCount);

        telemetry.addLine("--- SPINDEX ---");
        telemetry.addData("Target Ticks", spindexTargetTicks);
        telemetry.addData("Current Ticks", spindexMotor.getCurrentPosition());
        telemetry.addData("Position Error", spindexPositionError);
        telemetry.addData("Jam Detected", jamDetected ? "YES" : "no");
        telemetry.addData("Spindex Current (A)", "%.2f", spindexMotor.getCurrent(CurrentUnit.AMPS));

        telemetry.addLine("--- UNJAM CONTROLS ---");
        telemetry.addData("A", "Reverse spindex");
        telemetry.addData("B", "Reverse intake");
        telemetry.addData("Dpad Up", "Reverse both");
        telemetry.addData("Dpad Down", jiggleActive ? "JIGGLING" : "Jiggle");
        telemetry.addData("Dpad Left", "Reset spindex to 0");

        telemetry.update();
    }
}