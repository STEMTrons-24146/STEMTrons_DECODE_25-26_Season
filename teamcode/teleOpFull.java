package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "sahaa")
public class teleOpFull extends LinearOpMode {

    // =====================
    // Drivetrain
    // =====================
    DcMotor flDrive, frDrive, rlDrive, rrDrive;

    // =====================
    // Intake
    // =====================
    DcMotor intake;
    boolean intakeOn = false;
    boolean yPressedLast = false;

    // =====================
    // Spindex (Encoder Based)
    // =====================
    DcMotor armMotor;
    int armTargetTicks = 0;
    final int ARM_STEP_TICKS = 128;

    boolean lbWasPressed = false;
    boolean rbWasPressed = false;

    // =====================
    // Vision + Turret
    // =====================
    Limelight3A limelight;
    CRServo turretLeft, turretRight;

    private static final int TARGET_TAG_ID = 20;

    private static final double kP = 0.035;
    private static final double MAX_POWER = 0.4;
    private static final double DEADZONE = 0.0;
    private static final long TAG_LOST_GRACE_MS = 25;

    private static final double DIST_ALPHA = 0.4;
    private static final double DIST_SCALE = 1.6;

    private double rawDistance = 0.0;
    private double filteredDistance = 1.0;

    private long lastTagSeenTime = 0;
    private double lastTx = 0.0;

    // Tracking toggle
    private boolean trackingEnabled = true;
    private boolean xWasPressed = false;

    // =====================
    // Shooter System
    // =====================
    DcMotorEx shooterMotor;
    DcMotor transfer;

    private static final int TRANSFER_UP_TICKS = -116;
    private static final double TRANSFER_START_SEC = 2.0;
    private static final double TOTAL_FIRE_TIME_SEC = 6.0;

    private boolean firingSequenceActive = false;
    private double firingStartTime = 0.0;
    

    @Override
    public void runOpMode() {

        // =====================
        // Hardware Map
        // =====================
        flDrive = hardwareMap.dcMotor.get("motor4");
        frDrive = hardwareMap.dcMotor.get("motor0");
        rlDrive = hardwareMap.dcMotor.get("motor5");
        rrDrive = hardwareMap.dcMotor.get("motor1");

        intake = hardwareMap.dcMotor.get("motor2");
        armMotor = hardwareMap.get(DcMotor.class, "motor7");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretLeft = hardwareMap.get(CRServo.class, "servo6");
        turretRight = hardwareMap.get(CRServo.class, "servo5");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "motor3");
        transfer = hardwareMap.get(DcMotor.class, "motor6");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPower(0.0);

        // =====================
        // Drivetrain Setup
        // =====================
        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // =====================
        // Intake Setup
        // =====================
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // =====================
        // Spindex Setup
        // =====================
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(550);
        

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Decode Rev D Ready");
        telemetry.update();

        waitForStart();

        // =====================
        // Main Loop
        // =====================
        while (opModeIsActive()) {

            // -----------------
            // Drivetrain
            // -----------------
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double deadzone = 0.2;
            if (Math.abs(y) < deadzone) y = 0;
            if (Math.abs(x) < deadzone) x = 0;
            if (Math.abs(rx) < deadzone) rx = 0;

            double flPower = y + x + rx;
            double frPower = y - x - rx;
            double rlPower = y - x + rx;
            double rrPower = y + x - rx;

            double max = Math.max(
                    Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(rlPower), Math.abs(rrPower))
            );
            if (max > 1.0) {
                flPower /= max;
                frPower /= max;
                rlPower /= max;
                rrPower /= max;
            }

            flDrive.setPower(flPower);
            frDrive.setPower(frPower);
            rlDrive.setPower(rlPower);
            rrDrive.setPower(rrPower);

            // -----------------
            // Intake Toggle
            // -----------------
            if (gamepad1.y && !yPressedLast) {
                intakeOn = !intakeOn;
            }
            yPressedLast = gamepad1.y;
            intake.setPower(intakeOn ? 1.0 : 0.0);

            // -----------------
            // Spindex Control
            // -----------------
            if (gamepad1.left_bumper && !lbWasPressed) {
                armTargetTicks += ARM_STEP_TICKS;
            }
            lbWasPressed = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !rbWasPressed) {
                armTargetTicks += ARM_STEP_TICKS / 2;
            }
            rbWasPressed = gamepad1.right_bumper;

            setAndHoldArmPosition(armTargetTicks);

            // -----------------
            // Tracking Toggle (X)
            // -----------------
            if (gamepad1.x && !xWasPressed) {
                trackingEnabled = !trackingEnabled;
            }
            xWasPressed = gamepad1.x;

            // -----------------
            // Limelight Detection
            // -----------------
            boolean tagVisible = false;
            double tx = 0.0;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        tagVisible = true;
                        tx = fid.getTargetXDegrees();

                        Pose3D pose = fid.getTargetPoseCameraSpace();
                        double xPos = pose.getPosition().x;
                        double yPos = pose.getPosition().y;
                        double zPos = pose.getPosition().z;

                        rawDistance = Math.sqrt(xPos*xPos + yPos*yPos + zPos*zPos);
                        double corrected = rawDistance * DIST_SCALE;
                        filteredDistance =
                                (1 - DIST_ALPHA) * filteredDistance + DIST_ALPHA * corrected;

                        lastTagSeenTime = System.currentTimeMillis();
                        lastTx = tx;
                        break;
                    }
                }
            }

            // -----------------
            // Turret Auto Tracking
            // -----------------
            double turretPower = 0.0;

            if (trackingEnabled) {
                if (tagVisible || (System.currentTimeMillis() - lastTagSeenTime) <= TAG_LOST_GRACE_MS) {
                    if (Math.abs(lastTx) > DEADZONE) {
                        turretPower = Range.clip(kP * lastTx, -MAX_POWER, MAX_POWER);
                    }
                }
            }

            turretLeft.setPower(turretPower);
            turretRight.setPower(turretPower);

            // -----------------
            // Fire Sequence (Left Trigger)
            // -----------------
            if (gamepad1.left_trigger > 0.1 && !firingSequenceActive) {
                firingSequenceActive = true;
                firingStartTime = getRuntime();
            }

            if (gamepad1.left_trigger > 0.1) {
                // Shooter spins at velocity based on distance
                double safeDistance = Range.clip(filteredDistance, 0.75, 2.50);
                double targetVel = getShooterVelocityFromDistance(safeDistance);
                shooterMotor.setVelocity(targetVel);
            } else {
                shooterMotor.setVelocity(0);
            }
            
            boolean dpadUpPressed = gamepad1.dpad_up;

            if (dpadUpPressed) {
                transfer.setPower(1.0);  // move forward while button held
            } else {
                transfer.setPower(-0.25); // slight pullback when button not pressed
            }
        
            //dpadUpPreviouslyPressed = dpadUpPressed;
            

            // -----------------
            // Telemetry
            // -----------------
            telemetry.addData("Spindex Target", armTargetTicks);
            telemetry.addData("Spindex Position", armMotor.getCurrentPosition());
            telemetry.addData("Tracking Enabled", trackingEnabled);
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addData("Distance (m)", filteredDistance);
            telemetry.addData("Firing Active", firingSequenceActive);
            //telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Actual Velocity", shooterMotor.getVelocity());   

            telemetry.update();
            idle();
        }
    }

    // =====================
    // Spindex Hold Function
    // =====================
    void setAndHoldArmPosition(int targetTicks) {
        armMotor.setTargetPosition(targetTicks);
        ((DcMotorEx) armMotor).setVelocity(550);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // =====================
    // Shooter Power Lookup
    // =====================
    private double getShooterVelocityFromDistance(double distanceMeters) {

    double[][] SHOOTER_TABLE = {
            {0.75, 1800}, {1.00, 1875}, {1.25, 1950}, {1.50, 2025},
            {1.75, 2100}, {2.00, 2175}, {2.25, 2250}, {2.50, 2325}
    };

    // Clamp distance to table range
    if (distanceMeters <= SHOOTER_TABLE[0][0]) return SHOOTER_TABLE[0][1];
    if (distanceMeters >= SHOOTER_TABLE[SHOOTER_TABLE.length - 1][0])
        return SHOOTER_TABLE[SHOOTER_TABLE.length - 1][1];

    // Linear interpolation
    for (int i = 0; i < SHOOTER_TABLE.length - 1; i++) {
        double d0 = SHOOTER_TABLE[i][0];
        double d1 = SHOOTER_TABLE[i + 1][0];

        if (distanceMeters >= d0 && distanceMeters <= d1) {
            double v0 = SHOOTER_TABLE[i][1];
            double v1 = SHOOTER_TABLE[i + 1][1];
            double t = (distanceMeters - d0) / (d1 - d0);
            return v0 + (v1 - v0) * t; // <- send actual velocity
        }
    }

    return 0; // fallback, shouldn’t happen
    }
}
