package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSystem {

    // Hardware
    private DcMotorEx shooterMotor;
    private DcMotor transfer;
    private DcMotor armMotor;
    private Servo hoodServo;

    // Timer (replaces stageStart = getRuntime())
    private ElapsedTime timer = new ElapsedTime();

    // State
    private boolean firing = false;
    private int fireStage = 0;

    // Timing constants
    private static final double SPIN_UP_TIME = 1.7;
    private static final double ARM_COOLDOWN = 0.7;
    private static final double TRANSFER_MOVE_TIME = 0.4;

    // Fixed settings
    private static final double FIXED_HOOD = 0.35;
    private static final double FIXED_SHOOTER_SPEED = 1900;

    // Arm tracking (exact behavior preserved)
    private int armTargetTicks = 0;

    public ShooterSystem(DcMotorEx shooterMotor,
                         DcMotor transfer,
                         DcMotor armMotor,
                         Servo hoodServo) {

        this.shooterMotor = shooterMotor;
        this.transfer = transfer;
        this.armMotor = armMotor;
        this.hoodServo = hoodServo;

        // Ensure arm motor is in RUN_TO_POSITION
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);

        transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        transfer.setPower(1.0);
    }

    // ======================================================
    // PUBLIC METHOD — ZERO ARGUMENTS
    // Call this repeatedly in your OpMode loop
    // ======================================================
    public void shoot() {

        // Always hold constant settings while firing
        hoodServo.setPosition(FIXED_HOOD);
        shooterMotor.setVelocity(FIXED_SHOOTER_SPEED);

        if (!firing) {
            firing = true;
            fireStage = 0;
            timer.reset();
        }

        double elapsed = timer.seconds();

        switch (fireStage) {

            case 0: // Spin-up
                if (elapsed >= SPIN_UP_TIME) {

                    double steps = armTargetTicks / 128.0;
                    if (Math.abs((steps - Math.floor(steps)) - 0.5) <= 0.1 ||
                        Math.abs(steps - Math.floor(steps)) <= 0.1)
                        armTargetTicks += 64;

                    fireStage = 1;
                    timer.reset();
                }
                break;

            case 1:
                if (elapsed >= ARM_COOLDOWN) {
                    transfer.setTargetPosition(-116);
                    fireStage = 2;
                    timer.reset();
                }
                break;

            case 2:
                if (elapsed >= TRANSFER_MOVE_TIME) {
                    transfer.setTargetPosition(0);
                    fireStage = 3;
                    timer.reset();
                }
                break;

            case 3:
                if (elapsed >= TRANSFER_MOVE_TIME) {
                    armTargetTicks += 128;
                    fireStage = 4;
                    timer.reset();
                }
                break;

            case 4:
                if (elapsed >= ARM_COOLDOWN) {
                    transfer.setTargetPosition(-116);
                    fireStage = 5;
                    timer.reset();
                }
                break;

            case 5:
                if (elapsed >= TRANSFER_MOVE_TIME) {
                    transfer.setTargetPosition(0);
                    fireStage = 6;
                    timer.reset();
                }
                break;

            case 6:
                if (elapsed >= TRANSFER_MOVE_TIME) {
                    armTargetTicks += 128;
                    fireStage = 7;
                    timer.reset();
                }
                break;

            case 7:
                if (elapsed >= ARM_COOLDOWN) {
                    transfer.setTargetPosition(-116);
                    fireStage = 8;
                    timer.reset();
                }
                break;

            case 8:
                if (elapsed >= TRANSFER_MOVE_TIME) {
                    transfer.setTargetPosition(0);
                    fireStage = 9;
                    timer.reset();
                }
                break;

            case 9:
                if (elapsed >= TRANSFER_MOVE_TIME) {
                    armTargetTicks += 64;
                    fireStage = 10;
                    timer.reset();
                }
                break;

            case 10:
                if (elapsed >= ARM_COOLDOWN) {
                    shooterMotor.setVelocity(0);
                    firing = false;
                }
                break;
        }

        // Always update arm motor target
        armMotor.setTargetPosition(armTargetTicks);
    }

    public boolean isFiring() {
        return firing;
    }
}
