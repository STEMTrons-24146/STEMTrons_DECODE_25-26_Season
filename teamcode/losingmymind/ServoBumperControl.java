package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Bumper Control")
public class ServoBumperControl extends OpMode {

    private Servo servo;
    private double servoPos = 0.5;     // start at middle
    private static final double STEP = 0.005; // amount to move per press

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo0");
        servo.setPosition(servoPos);
    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger > 0) {
            servoPos += STEP;
        }

        if (gamepad1.left_trigger > 0) {
            servoPos -= STEP;
        }

        // keep value between 0 and 1
        servoPos = Range.clip(servoPos, 0.0, 1.0);

        servo.setPosition(servoPos);

        telemetry.addData("Servo Position", servoPos);
        telemetry.update();
    }
}
