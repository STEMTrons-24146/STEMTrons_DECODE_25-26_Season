package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "spindexticks", group = "TeleOp")
public class spindexlastoneihope extends LinearOpMode {

    DcMotor spindex;
    DcMotor transfer;
    DcMotor outtake;

    // Positions
    final int SPINDEX_DOWN = 0;
    final int SPINDEX_UP = -116;

    // Timing
    final double UP_TIME_SEC = 1.5;

    // State
    boolean sequenceRunning = false;
    double sequenceStartTime = 0;

    // Button tracking
    boolean bWasPressed = false;
    boolean aWasPressed = false;
    boolean lbWasPressed = false;

    // Manual trim
    int spindexTargetTicks = 0;
    final int SPINDEX_STEP_TICKS = 50;

    @Override
    public void runOpMode() {

        // Hardware init

        transfer = hardwareMap.get(DcMotor.class, "motor6");
        

        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setTargetPosition(0);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindex.setPower(0.5);

        telemetry.addLine("spindex Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---- Manual trim (A / B) ----
            if (!sequenceRunning) {
                if (gamepad1.a && !aWasPressed) {
                    spindexTargetTicks += SPINDEX_STEP_TICKS;
                }
                if (gamepad1.b && !bWasPressed) {
                    spindexTargetTicks += SPINDEX_STEP_TICKS / 2;
                }
            }
            aWasPressed = gamepad1.a;
            bWasPressed = gamepad1.b;

            // ---- Single-button timed sequence ----
            if ((gamepad1.left_trigger >0) && !lbWasPressed && !sequenceRunning) {
                sequenceRunning = true;
                sequenceStartTime = getRuntime();
                spindexTargetTicks = SPINDEX_UP;
            }
            lbWasPressed = (gamepad1.left_trigger>0);

            // ---- Auto return after 1500 ms ----
            if (sequenceRunning) {
                if (getRuntime() - sequenceStartTime >= UP_TIME_SEC) {
                    spindexTargetTicks = SPINDEX_DOWN;
                    sequenceRunning = false;
                }
            }

            // ---- Transfer ----
            transfer.setPower(-gamepad1.left_trigger);

            // ---- Outtake ----
            if (gamepad1.x) {
                outtake.setPower(-1.0);
            } else {
                outtake.setPower(0);
            }

            // ---- Hold position ----
            spindex.setTargetPosition(spindexTargetTicks);

            // ---- Telemetry ----
            telemetry.addData("Target", spindexTargetTicks);
            telemetry.addData("Position", spindex.getCurrentPosition());
            telemetry.addData("Sequence", sequenceRunning);
            telemetry.update();
        }
    }
}
