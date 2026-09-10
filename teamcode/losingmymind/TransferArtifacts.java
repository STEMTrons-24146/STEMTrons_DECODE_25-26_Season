package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Transfer Artifact", group="TeleOp")
public class TransferArtifacts extends LinearOpMode {

    private DcMotor motor6;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize hardware
        motor6 = hardwareMap.get(DcMotor.class, "motor6");

        // Wait for start button
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                // Reset timer and start motor
                timer.reset();
                motor6.setPower(-0.7);

                // Spin for 1000 milliseconds (1 second)
                while (opModeIsActive() && timer.milliseconds() < 2000) {
                    // Optional: Telemetry for debug
                    telemetry.addData("Motor5 Power", motor6.getPower());
                    telemetry.addData("Time elapsed (ms)", timer.milliseconds());
                    telemetry.update();
                }

                // Stop motor
                motor6.setPower(0.0);
            }

            // Other TeleOp code can go here
            idle();
        }
    }
}
