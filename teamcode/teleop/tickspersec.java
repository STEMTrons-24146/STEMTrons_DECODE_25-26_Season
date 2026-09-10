package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Velocity Control Test", group="Test")
public class tickspersec extends OpMode {

    private DcMotorEx motor;

    // Set your desired velocity here (ticks per second)
    private double targetVelocity = 2500;  

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotorEx.class, "motor3");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        // Optional: Adjust velocity with gamepad
        if (gamepad1.dpad_up) {
            targetVelocity += 50;
        }
        if (gamepad1.dpad_down) {
            targetVelocity -= 50;
        }

        // Set velocity (closed-loop control)
        motor.setVelocity(targetVelocity);

        // Telemetry
        telemetry.addData("Target Velocity (ticks/sec)", targetVelocity);
        telemetry.addData("Actual Velocity (ticks/sec)", motor.getVelocity());
        telemetry.addData("Motor Power Being Applied", motor.getPower());
        telemetry.update();
    }
}
