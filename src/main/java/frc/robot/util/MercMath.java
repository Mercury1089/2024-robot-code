package frc.robot.util;

import java.util.function.Supplier;

public class MercMath {
    public static Supplier<Double> zeroSupplier = () -> 0.0;

    public static double sqaureInput(double input) {
        return input > 0.0 ? Math.pow(input, 2) : -Math.pow(input, 2);
    }
}
