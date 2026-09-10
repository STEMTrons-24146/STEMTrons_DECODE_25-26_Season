package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveSystem {

    private DcMotor flDrive, frDrive, rlDrive, rrDrive;
    private static final double MAX_POWER = 0.5; // Default max power for standard move commands

    public DriveSystem(DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr) {
        flDrive = fl;
        frDrive = fr;
        rlDrive = rl;
        rrDrive = rr;
    }

    /**
     * Sets all motor powers to 1.0 for a specific duration in milliseconds.
     * @param durationMs The time to drive in milliseconds.
     */
    public void fullPowerDuration(int durationMs) {
        // Set all powers to 1.0
        flDrive.setPower(1.0);
        frDrive.setPower(1.0);
        rlDrive.setPower(1.0);
        rrDrive.setPower(1.0);

        try {
            Thread.sleep(durationMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Stop all motors
        stop();
    }

    public void stop() {
        flDrive.setPower(0);
        frDrive.setPower(0);
        rlDrive.setPower(0);
        rrDrive.setPower(0);
    }

    /**
     * Existing time-based movement command (Seconds based).
     */
    public void move(double x, double y, double z) {
        double fl = x + y + z;
        double fr = x - y - z;
        double rl = x - y + z;
        double rr = x + y - z;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                              Math.max(Math.abs(rl), Math.abs(rr)));
        if (max > 1.0) {
            fl /= max; fr /= max; rl /= max; rr /= max;
        }

        fl *= MAX_POWER;
        fr *= MAX_POWER;
        rl *= MAX_POWER;
        rr *= MAX_POWER;

        flDrive.setPower(fl);
        frDrive.setPower(fr);
        rlDrive.setPower(rl);
        rrDrive.setPower(rr);

        double duration = Math.max(Math.max(Math.abs(x), Math.abs(y)), Math.abs(z)) * 1000;

        try {
            Thread.sleep((long) duration);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        stop();
    }
}