package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Analog Max Voltage Test")
public class AnalogMaxVoltageTest extends OpMode {

    private AnalogInput analog1;

    @Override
    public void init() {
        // "analog1" must match the name in the Robot Configuration
        analog1 = hardwareMap.get(AnalogInput.class, "analog1");
    }

    @Override
    public void loop() {
        double maxVoltage = analog1.getMaxVoltage();

        telemetry.addData("Analog1 Max Voltage", maxVoltage);
        telemetry.update();
    }
}
