package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "auto?")
public class Automaybe extends LinearOpMode {

    DcMotor flDrive, frDrive, rlDrive, rrDrive;
    DcMotor armMotor, shooterMotor, transfer;
    Servo hoodServo;

    int armTargetTicks = 0;

    private static final double SPIN_UP_TIME = 1.7;
    private static final double ARM_COOLDOWN = 0.7;
    private static final double TRANSFER_MOVE_TIME = 0.4;

    @Override
    public void runOpMode() {

        // Hardware Mapping
        flDrive = hardwareMap.dcMotor.get("motor4");
        frDrive = hardwareMap.dcMotor.get("motor0");
        rlDrive = hardwareMap.dcMotor.get("motor5");
        rrDrive = hardwareMap.dcMotor.get("motor1");

        armMotor = hardwareMap.get(DcMotor.class, "motor7");
        shooterMotor = hardwareMap.get(DcMotor.class, "motor3");
        transfer = hardwareMap.get(DcMotor.class, "motor6");
        hoodServo = hardwareMap.get(Servo.class, "servo4");

        // Drive direction
        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Arm setup
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);

        // Transfer setup
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setTargetPosition(0);
        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(0.8);

        waitForStart();
        if (!opModeIsActive()) return;

        // ---------------------------------
        // LOCK Shooter + Hood (DO NOT CHANGE)
        // ---------------------------------
        hoodServo.setPosition(0.35);
        ((DcMotorEx) shooterMotor).setVelocity(1900);

        sleep((long)(SPIN_UP_TIME * 1000));

        // ---------------------------------
        // FIRE SEQUENCE (time-based)
        // ---------------------------------

        for (int cycle = 0; cycle < 3 && opModeIsActive(); cycle++) {

            // Move arm
            armTargetTicks += (cycle == 2) ? 64 : 128;
            armMotor.setTargetPosition(armTargetTicks);
            sleep((long)(ARM_COOLDOWN * 1000));

            // Transfer forward
            transfer.setTargetPosition(-116);
            sleep((long)(TRANSFER_MOVE_TIME * 1000));

            // Transfer back
            transfer.setTargetPosition(0);
            sleep((long)(TRANSFER_MOVE_TIME * 1000));
        }

        // Final small arm adjustment
        armTargetTicks += 64;
        armMotor.setTargetPosition(armTargetTicks);
        sleep((long)(ARM_COOLDOWN * 1000));

        // Stop shooter
        ((DcMotorEx) shooterMotor).setVelocity(0);

        // ---------------------------------
        // MOVE FORWARD AFTER SHOOTING
        // ---------------------------------
        flDrive.setPower(1.0);
        frDrive.setPower(1.0);
        rlDrive.setPower(1.0);
        rrDrive.setPower(1.0);

        sleep(500);

        flDrive.setPower(0);
        frDrive.setPower(0);
        rlDrive.setPower(0);
        rrDrive.setPower(0);

        telemetry.addLine("Auto Complete");
        telemetry.update();
        sleep(2000);
    }
}
