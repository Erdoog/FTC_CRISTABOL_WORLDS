package util;

public class InputScaler {
    public static double scaleInputMedium(double x) {
        return x * (0.28 * x * x + 0.22);
    }

    public static double scaleInputLow(double x) {
        return x * (0.17 * x * x + 0.10);
    }

    public static double scaleInputHigh(double x) {
        return x * (0.58 * x * x + 0.42);
    }
}
