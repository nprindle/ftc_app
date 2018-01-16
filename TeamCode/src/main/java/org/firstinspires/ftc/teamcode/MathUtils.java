package org.firstinspires.ftc.teamcode;

public final class MathUtils {

    private MathUtils() {

    }

    // Ticks in a full revolution according to motor specifications
    // AndyMark NeveRest Classic 40 Gearmotor
    private static final double TICKS = 1920;
    // Tetrix DC Motor
    // private static final double TICKS = 1440;

    // Measurements used to calculate ticks from distance
    private static final double WHEEL_DIAMETER_IN_INCHES = 4.0;
    private static final double WHEEL_BASE_IN_INCHES     = 15.5;
    private static final double TICKS_PER_INCH           = TICKS /
            (WHEEL_DIAMETER_IN_INCHES * Math.PI);
    private static final double TICKS_PER_DEGREE         = (Math.PI *
            WHEEL_BASE_IN_INCHES * TICKS_PER_INCH) / 360.0;

    /**
     * Index to motor associations:
     * <p>
     * 0     1      2         3
     * frontLeft, frontRight, backLeft, backRight
     * <p>
     * Uses the positions of the joysticks to determine the relative velocities
     * to move each of the wheels in order to strafe
     */
    public static float[] calculateVelocities(float left_x, float left_y,
                                              float right_x, float right_y) {
        double r          = Math.hypot(left_x, left_y);
        double robotAngle = Math.atan2(left_y, left_x) - Math.PI / 4;
        return new float[]{
                (float) (r * Math.cos(robotAngle) + right_x),
                (float) (r * Math.sin(robotAngle) - right_x),
                (float) (r * Math.sin(robotAngle) + right_x),
                (float) (r * Math.cos(robotAngle) - right_x),
                };
    }

    // The same method as above but used in an autonomous context
    // i.e. no controller input necessary
    public static float[] calculateVelocities(double power, double degrees) {
        double radians = Math.toRadians(degrees) - Math.PI / 4;
        return new float[]{
                (float) (power * Math.cos(radians)),
                (float) (power * Math.sin(radians)),
                (float) (power * Math.sin(radians)),
                (float) (power * Math.cos(radians)),
                };
    }

    // inchesPerUnit for some unit u should be equal to the ratio inches/unit
    // standard ratios are provided as unit names in the autonomous template
    // e.g. INCHES = 1.0 in/in, CM = 0.3937 in/cm, etc.
    public static int distanceToTicks(double distance, double inchesPerUnit) {
        return (int) (distance * inchesPerUnit * TICKS_PER_INCH);
    }

    // Convert degrees of a point turn to encoder ticks
    public static int degreesToTicks(double degrees) {
        return Math.abs((int) (degrees * TICKS_PER_DEGREE));
    }

    // Generic helper method to restrict a value between a certain range
    public static double constrain(double lower, double upper, double val) {
        if (val > upper)
            return upper;
        if (val < lower)
            return lower;
        return val;
    }

    // Generic helper method to restrict the position of a servo
    public static double constrainServo(double val) {
        return constrain(0.0, 1.0, val);
    }

    // Generic helper method to determine whether or not a value is within
    // a certain range of some truth value
    // Originally used in turning based on gyroscope headings, as the turn
    // could overshoot the target heading
    public static boolean within(float value, float truth, float error) {
        return Math.abs(value - truth) <= error;
    }

}
