package org.firstinspires.ftc.teamcode.testing.autoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "LinearMovementONLY")
public class odoTest1 extends LinearOpMode {

    DcMotor flDrive, frDrive, rlDrive, rrDrive;
    DcMotor yOdo, xOdo;

    final double ODO_WHEEL_DIAMETER = 32; // mm
    final double TICKS_PER_REVOLUTION = 2000;
    final double ODO_WHEEL_CIRCUMFERENCE = Math.PI * ODO_WHEEL_DIAMETER;
    final double TICKS_PER_MM = TICKS_PER_REVOLUTION / ODO_WHEEL_CIRCUMFERENCE;
    final double TICKS_PER_INCH = TICKS_PER_MM * 25.4;

    @Override
    public void runOpMode() {
        flDrive = hardwareMap.get(DcMotor.class, "drive0");
        rlDrive = hardwareMap.get(DcMotor.class, "drive1");
        frDrive = hardwareMap.get(DcMotor.class, "drive2");
        rrDrive = hardwareMap.get(DcMotor.class, "drive3");

        xOdo = hardwareMap.get(DcMotor.class, "odo0");
        yOdo = hardwareMap.get(DcMotor.class, "odo1");

        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rlDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        xOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            moveForward(24);
            sleep(1000);

            moveBackward(24);
            sleep(1000);

            strafeLeft(24);
            sleep(1000);

            strafeRight(24);
            sleep(1000);
        }
    }

    private void moveForward(double inches) {
        int startY = yOdo.getCurrentPosition();
        double target = inches * TICKS_PER_INCH;

        while (opModeIsActive()) {
            int yPos = yOdo.getCurrentPosition() - startY;

            if (yPos >= target) break;

            power(0.3, 0.3, 0.3, 0.3);
        }
        stopAll();
    }

    private void moveBackward(double inches) {
        int startY = yOdo.getCurrentPosition();
        double target = inches * TICKS_PER_INCH;

        while (opModeIsActive()) {
            int yPos = yOdo.getCurrentPosition() - startY;

            if (yPos <= -target) break;

            power(-0.3, -0.3, -0.3, -0.3);
        }
        stopAll();
    }

    private void strafeLeft(double inches) {
        int startX = xOdo.getCurrentPosition();
        double target = inches * TICKS_PER_INCH;

        while (opModeIsActive()) {
            int xPos = xOdo.getCurrentPosition() - startX;

            if (xPos >= target) break;

            power(-0.3, 0.3, 0.3, -0.3);
        }
        stopAll();
    }

    private void strafeRight(double inches) {
        int startX = xOdo.getCurrentPosition();
        double target = inches * TICKS_PER_INCH;

        while (opModeIsActive()) {
            int xPos = xOdo.getCurrentPosition() - startX;

            if (xPos <= -target) break;

            power(0.3, -0.3, -0.3, 0.3);
        }
        stopAll();
    }

    private void power(double fl, double rl, double fr, double rr) {
        flDrive.setPower(fl);
        rlDrive.setPower(rl);
        frDrive.setPower(fr);
        rrDrive.setPower(rr);
    }

    private void stopAll() {
        power(0, 0, 0, 0);
    }
}
