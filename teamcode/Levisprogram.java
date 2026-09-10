package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Levisprogram")
public class Levisprogram extends LinearOpMode {

    // Drivetrain
    DcMotor flDrive, frDrive, rlDrive, rrDrive;

    // Mechanisms
    DcMotor intake, transfer, shooter;
    DcMotorEx spindex;

    // Turret
    CRServo turretLeft, turretRight;
    static final double TURRET_POWER = 0.3;

    // Hood
    Servo hood;
    double hoodPos = 0.5;
    static final double HOOD_STEP = 0.005;

    // Spindex Encoder Control
    int spindexTargetTicks = 0;
    final int ARM_STEP_TICKS = 128;

    // 🚀 Increased speed here
    final double SPINDEX_VELOCITY = 700;

    boolean dpadRightWasPressed = false;
    boolean dpadLeftWasPressed = false;
    boolean yWasPressed = false;
    boolean halfStepMode = false;

    @Override
    public void runOpMode() {

        // Hardware Map
        flDrive = hardwareMap.dcMotor.get("motor4");
        frDrive = hardwareMap.dcMotor.get("motor0");
        rlDrive = hardwareMap.dcMotor.get("motor5");
        rrDrive = hardwareMap.dcMotor.get("motor1");

        intake   = hardwareMap.dcMotor.get("motor2");
        transfer = hardwareMap.dcMotor.get("motor6");
        shooter  = hardwareMap.dcMotor.get("motor3");
        spindex  = (DcMotorEx) hardwareMap.dcMotor.get("motor7");

        hood = hardwareMap.get(Servo.class, "servo4");

        turretLeft  = hardwareMap.get(CRServo.class, "servo6");
        turretRight = hardwareMap.get(CRServo.class, "servo5");

        // Motor Directions
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Spindex Setup
        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setTargetPosition(0);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setVelocity(SPINDEX_VELOCITY);

        hood.setPosition(hoodPos);

        telemetry.addLine("Levisprogram Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // =====================
            // DRIVETRAIN
            // =====================
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

            // =====================
            // SHOOTER (100%)
            // =====================
            if (gamepad1.right_trigger > 0) {
                shooter.setPower(1.0);
            } else {
                shooter.setPower(0);
            }

            // =====================
            // TRANSFER
            // =====================
            if (gamepad1.dpad_up) {
                transfer.setPower(1.0);
            } else {
                transfer.setPower(-0.25);
            }

            // =====================
            // INTAKE (A/B)
            // =====================
            if (gamepad1.a) intake.setPower(1.0);
            if (gamepad1.b) intake.setPower(0.0);

            // =====================
            // HOOD
            // =====================
            if (gamepad1.dpad_down) hoodPos += HOOD_STEP;
            hoodPos = Range.clip(hoodPos, 0.0, 1.0);
            hood.setPosition(hoodPos);

            // =====================
            // TURRET
            // =====================
            if (gamepad1.right_bumper) {
                turretLeft.setPower(-TURRET_POWER);
                turretRight.setPower(-TURRET_POWER);
            } else if (gamepad1.left_bumper) {
                turretLeft.setPower(TURRET_POWER);
                turretRight.setPower(TURRET_POWER);
            } else {
                turretLeft.setPower(0);
                turretRight.setPower(0);
            }

            // =====================
            // SPINDEX MODE TOGGLE (Y)
            // =====================
            if (gamepad1.y && !yWasPressed) {
                halfStepMode = !halfStepMode;
            }
            yWasPressed = gamepad1.y;

            // =====================
            // SPINDEX CONTROL
            // =====================
            int stepSize = halfStepMode ? ARM_STEP_TICKS / 2 : ARM_STEP_TICKS;

            if (gamepad1.dpad_right && !dpadRightWasPressed) {
                spindexTargetTicks += stepSize;
            }

            if (gamepad1.dpad_left && !dpadLeftWasPressed) {
                spindexTargetTicks -= stepSize;
            }

            dpadRightWasPressed = gamepad1.dpad_right;
            dpadLeftWasPressed = gamepad1.dpad_left;

            setAndHoldSpindexPosition(spindexTargetTicks);

            telemetry.addData("Spindex Mode", halfStepMode ? "Half Step" : "Full Step");
            telemetry.addData("Spindex Target", spindexTargetTicks);
            telemetry.addData("Spindex Position", spindex.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }

    void setAndHoldSpindexPosition(int targetTicks) {
        spindex.setTargetPosition(targetTicks);
        spindex.setVelocity(SPINDEX_VELOCITY);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
