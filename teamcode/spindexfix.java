package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.*;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "spindexLockFinal")
public class spindexfix extends LinearOpMode {

    // -----------------
    // Hardware
    // -----------------
    DcMotor flDrive, frDrive, rlDrive, rrDrive;
    DcMotor intake, armMotor, shooterMotor, transfer;
    CRServo turretLeft, turretRight;
    Servo hoodServo;
    Limelight3A limelight;

    // -----------------
    // Spindex System
    // -----------------
    final int SLOT_SPACING = 128;
    final int OFFSET = 65; // TUNE THIS

    int currentSlot = 0;

    int getSlotPosition(int slot) {
        return slot * SLOT_SPACING;
    }

    int getCatapultPosition(int slot) {
        return (slot * SLOT_SPACING) + OFFSET;
    }

    // -----------------
    // Controls
    // -----------------
    boolean intakeOn = false, yPressedLast = false;
    boolean lbWasPressed = false, rbWasPressed = false;

    boolean lockMode = false;
    boolean aWasPressed = false;

    private static final int TARGET_TAG_ID = 20;

    // Turret
    private static final double kP = 0.035;
    private static final double MAX_POWER = 0.4;
    private long lastTagSeenTime = 0;
    private double lastTx = 0.0;
    private boolean trackingEnabled = true;
    private boolean xWasPressed = false;

    // Distance
    private double filteredDistance = 1.0;

    // Firing
    private boolean firing = false, ltWasPressed = false;
    private int fireStage = 0;
    private double stageStart = 0;

    // Timing
    private static final double SPIN_UP_TIME = 1.7;
    private static final double ARM_COOLDOWN = 0.5;
    private static final double TRANSFER_MOVE_TIME = 0.4;

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
        armMotor = hardwareMap.dcMotor.get("motor7");
        shooterMotor = hardwareMap.dcMotor.get("motor3");
        transfer = hardwareMap.dcMotor.get("motor6");

        turretLeft = hardwareMap.get(CRServo.class, "servo6");
        turretRight = hardwareMap.get(CRServo.class, "servo5");
        hoodServo = hardwareMap.get(Servo.class, "servo4");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // -----------------
        // Setup
        // -----------------
        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(800);

        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setTargetPosition(0);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(1.0);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            // -----------------
            // Drive
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
                fl /= max;
                fr /= max;
                rl /= max;
                rr /= max;
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
            // Lock Toggle (A)
            // -----------------
            if (gamepad1.a && !aWasPressed)
                lockMode = !lockMode;
            aWasPressed = gamepad1.a;

            // -----------------
            // Manual Index
            // -----------------
            if (gamepad1.left_bumper && !lbWasPressed)
                currentSlot = (currentSlot + 1) % 3;
            lbWasPressed = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !rbWasPressed)
                currentSlot = (currentSlot + 2) % 3;
            rbWasPressed = gamepad1.right_bumper;

            // -----------------
            // Arm Control
            // -----------------
            if (!firing) {
                if (lockMode) {
                    armMotor.setTargetPosition(getCatapultPosition(currentSlot));
                } else {
                    armMotor.setTargetPosition(getSlotPosition(currentSlot));
                }
            }

            // -----------------
            // Limelight Tracking
            // -----------------
            if (gamepad1.x && !xWasPressed)
                trackingEnabled = !trackingEnabled;
            xWasPressed = gamepad1.x;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        lastTx = fid.getTargetXDegrees();
                        lastTagSeenTime = System.currentTimeMillis();

                        Pose3D pose = fid.getTargetPoseCameraSpace();
                        filteredDistance = pose.getPosition().z;
                        break;
                    }
                }
            }

            double turretPower = 0;

            if (trackingEnabled &&
                    (System.currentTimeMillis() - lastTagSeenTime) < 25) {
                turretPower = Range.clip(kP * lastTx, -MAX_POWER, MAX_POWER);
            }

            turretLeft.setPower(turretPower);
            turretRight.setPower(turretPower);

            // -----------------
            // FIRE SYSTEM (LOCK + ALIGN)
            // -----------------
            boolean ltPressed = gamepad1.left_trigger > 0.1;

            if (ltPressed && !ltWasPressed && !firing) {
                firing = true;
                fireStage = 0;
                stageStart = getRuntime();
            }
            ltWasPressed = ltPressed;

            if (firing) {

                double elapsed = getRuntime() - stageStart;
                int target = getCatapultPosition(currentSlot);
                int error = Math.abs(armMotor.getCurrentPosition() - target);

                switch (fireStage) {

                    case 0: // LOCK + ALIGN FIRST
                        lockMode = true;
                        armMotor.setTargetPosition(target);

                        if (error < 10) {
                            fireStage = 1;
                            stageStart = getRuntime();
                        }
                        break;

                    case 1: // SPIN UP
                        ((DcMotorEx) shooterMotor).setVelocity(1400);

                        if (elapsed >= SPIN_UP_TIME) {
                            fireStage = 2;
                            stageStart = getRuntime();
                        }
                        break;

                    case 2: // FIRE
                        transfer.setTargetPosition(-130);

                        if (elapsed >= TRANSFER_MOVE_TIME) {
                            transfer.setTargetPosition(0);
                            fireStage = 3;
                            stageStart = getRuntime();
                        }
                        break;

                    case 3: // NEXT SLOT
                        currentSlot = (currentSlot + 1) % 3;
                        armMotor.setTargetPosition(getSlotPosition(currentSlot));

                        if (elapsed >= ARM_COOLDOWN) {
                            fireStage = 4;
                        }
                        break;

                    case 4: // DONE
                        firing = false;
                        lockMode = false;
                        ((DcMotorEx) shooterMotor).setVelocity(0);
                        break;
                }

            } else {
                transfer.setTargetPosition(0);
                ((DcMotorEx) shooterMotor).setVelocity(0);
            }

            // -----------------
            // Telemetry
            // -----------------
            telemetry.addData("Slot", currentSlot);
            telemetry.addData("Lock Mode", lockMode);
            telemetry.addData("Arm Target", armMotor.getTargetPosition());
            telemetry.addData("Arm Current", armMotor.getCurrentPosition());
            telemetry.addData("Error", Math.abs(armMotor.getCurrentPosition() - armMotor.getTargetPosition()));
            telemetry.update();

            idle();
        }
    }
}