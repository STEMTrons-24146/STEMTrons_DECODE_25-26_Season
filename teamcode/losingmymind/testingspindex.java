package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "testingspindex", group = "TeleOp")
public class testingspindex extends LinearOpMode {

    DcMotor armMotor;

    // Target position in encoder ticks
    int armTargetTicks = 0;

    // How many ticks to move per button press
    final int ARM_STEP_TICKS = 128; // <-- CHANGE THIS VALUE

    // Button state tracking
    boolean aWasPressed = false;
    boolean bWasPressed = false; // <--- track b button

    @Override
    public void runOpMode() {

        // Hardware init
        armMotor = hardwareMap.get(DcMotor.class, "motor7");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ((DcMotorEx) armMotor).setVelocity(400);

        telemetry.addLine("Arm Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Detect rising edge of gamepad1.a
            if (gamepad1.a && !aWasPressed) {
                armTargetTicks += ARM_STEP_TICKS;
            }
            aWasPressed = gamepad1.a;

            // Detect rising edge of gamepad1.b
            if (gamepad1.b && !bWasPressed) {
                armTargetTicks += ARM_STEP_TICKS / 2; // half step
            }
            bWasPressed = gamepad1.b;

            // Hold arm at target position
            setAndHoldArmPosition(armTargetTicks);

            // Telemetry
            telemetry.addData("Arm Target (ticks)", armTargetTicks);
            telemetry.addData("Arm Position (ticks)", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    // ---- Arm control function ----
    void setAndHoldArmPosition(int targetTicks) {
        armMotor.setTargetPosition(targetTicks);
        ((DcMotorEx) armMotor).setVelocity(400);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}


//not original