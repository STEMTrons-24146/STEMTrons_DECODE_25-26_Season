package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "FAKE SIMPLE TELEOP")
public class simpleTeleOp extends LinearOpMode {

    // =====================
    // Drivetrain
    // =====================
    DcMotor flDrive, frDrive, rlDrive, rrDrive;

    // =====================
    // Mechanisms
    // =====================
    // =====================
    // Turret (CRServos)
    // =====================
    // =====================
    // Hood Servo
    // =====================
    @Override
    public void runOpMode() {

        // =====================
        // Hardware Map
        // =====================
        flDrive = hardwareMap.dcMotor.get("motor0");
        frDrive = hardwareMap.dcMotor.get("motor1");
        rlDrive = hardwareMap.dcMotor.get("motor2");
        rrDrive = hardwareMap.dcMotor.get("motor3");

    
        // =====================
        // Drivetrain Setup
        // =====================
        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      
        telemetry.addLine("DecodeTeleOp ready");
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

        }
    }
}
