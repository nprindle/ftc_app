package org.firstinspires.ftc.teamcode

@Suppress("unused")
object MathUtils2 {

    // Ticks in a full revolution according to motor specifications
    // AndyMark NeveRest Classic 40 Gearmotor
    private val TICKS = 1920.0
    // Tetrix DC Motor
    // private static final double TICKS = 1440;

    // Measurements used to calculate ticks from distance
    private val WHEEL_DIAMETER_IN_INCHES = 4.0
    private val WHEEL_BASE_IN_INCHES = 15.5
    private val TICKS_PER_INCH = TICKS / (WHEEL_DIAMETER_IN_INCHES * Math.PI)
    private val TICKS_PER_DEGREE = Math.PI *
            WHEEL_BASE_IN_INCHES * TICKS_PER_INCH / 360.0

    /**
     * Index to motor associations:
     *
     *
     * 0     1      2         3
     * frontLeft, frontRight, backLeft, backRight
     *
     *
     * Uses the positions of the joysticks to determine the relative velocities
     * to move each of the wheels in order to strafe
     */
    fun calculateVelocities(left_x: Float, left_y: Float,
                            right_x: Float, right_y: Float): FloatArray {
        val r = Math.hypot(left_x.toDouble(), left_y.toDouble())
        val robotAngle = Math.atan2(left_y.toDouble(), left_x.toDouble()) - Math.PI / 4
        return floatArrayOf(
                (r * Math.cos(robotAngle) + right_x).toFloat(),
                (r * Math.sin(robotAngle) - right_x).toFloat(),
                (r * Math.sin(robotAngle) + right_x).toFloat(),
                (r * Math.cos(robotAngle) - right_x).toFloat())
    }

    // The same method as above but used in an autonomous context
    // i.e. no controller input necessary
    fun calculateVelocities(power: Double, degrees: Double): FloatArray {
        val radians = Math.toRadians(degrees) - Math.PI / 4
        return floatArrayOf(
                (power * Math.cos(radians)).toFloat(),
                (power * Math.sin(radians)).toFloat(),
                (power * Math.sin(radians)).toFloat(),
                (power * Math.cos(radians)).toFloat())
    }

    // inchesPerUnit for some unit u should be equal to the ratio inches/unit
    // standard ratios are provided as unit names in the autonomous template
    // e.g. INCHES = 1.0 in/in, CM = 0.3937 in/cm, etc.
    fun distanceToTicks(distance: Double, inchesPerUnit: Double): Int {
        return (distance * inchesPerUnit * TICKS_PER_INCH).toInt()
    }

    // Convert degrees of a point turn to encoder ticks
    fun degreesToTicks(degrees: Double): Int {
        return Math.abs((degrees * TICKS_PER_DEGREE).toInt())
    }

    // Generic helper method to restrict a value between a certain range
    fun constrain(lower: Double, upper: Double, value: Double): Double {
        return if (value < lower) lower else if (value > upper) upper else value
    }

    // Generic helper method to restrict the position of a servo
    fun constrainServo(`val`: Double): Double {
        return constrain(0.0, 1.0, `val`)
    }

    // Generic helper method to determine whether or not a value is within
    // a certain range of some truth value
    // Originally used in turning based on gyroscope headings, as the turn
    // could overshoot the target heading
    fun within(value: Float, truth: Float, error: Float): Boolean {
        return Math.abs(value - truth) <= error
    }

}

