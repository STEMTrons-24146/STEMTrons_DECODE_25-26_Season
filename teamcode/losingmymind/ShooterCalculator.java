package org.firstinspires.ftc.teamcode; // same package as decode.java

public class ShooterCalculator {

    // --- Coefficients for Hood Setting (Cubic Fit) ---
    private static final double HOOD_A = 0.05854;
    private static final double HOOD_B = 0.01604;
    private static final double HOOD_C = -0.60279;
    private static final double HOOD_D = 1.35164;

    // --- Coefficients for Shooter Speed/Ticks (Cubic Fit) ---
    private static final double TICKS_A = 58.300;
    private static final double TICKS_B = -184.90;
    private static final double TICKS_C = 513.95;
    private static final double TICKS_D = 1043.79;

    public static class ShooterState {
        public final double hoodPosition;
        public final double shooterSpeed;

        public ShooterState(double hoodPosition, double shooterSpeed) {
            this.hoodPosition = hoodPosition;
            this.shooterSpeed = shooterSpeed;
        }
    }

    public static ShooterState calculate(double z) {
        double dist = Math.max(0.58, Math.min(z, 1.75));
        double hood = (HOOD_A * Math.pow(dist, 3)) + (HOOD_B * Math.pow(dist, 2)) + (HOOD_C * dist) + HOOD_D;
        double speed = (TICKS_A * Math.pow(dist, 3)) + (TICKS_B * Math.pow(dist, 2)) + (TICKS_C * dist) + TICKS_D;
        return new ShooterState(hood, speed);
    }
}
