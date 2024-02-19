package frc.robot.util;

public class MercMath {
    public static double sqaureInput(double input) {
        return input > 0.0 ? Math.pow(input, 2) : -Math.pow(input, 2);
    }
}
