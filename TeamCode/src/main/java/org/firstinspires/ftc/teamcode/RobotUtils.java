package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class RobotUtils {

    private RobotUtils() {

    }

    // The access key required to interface with the Vuforia library
    private static final String VUFORIA_LICENSE_KEY =
        "AbL7LFz/////AAAAGWhvpgZGq0UrltWm0ta5FBNn13kH1NYN6f/Juwmhj7v05WK+H9pdat5tgHe2" +
            "YWLnHgsOnSF3VmR2AEnCBHPQynT7y0b3q4QYXiMfXvewvs+BRq1sZrSS7epVa3rFvXvAM0En2hvZ" +
            "kO02NHc2nSW8QvF39gjE6xCOS74sNPCm3IaYzZb/zzlDXcDLi0nH1VUMjmxB+2Y4Pc/5OMjxpwbl" +
            "lJ5v7UkQtFHYGOT672J3G34PVQK2z6MFQrExJL78yIr7Z43z55Pey2R7FLe3k2ixVkvULI1mVVNk" +
            "fc0M+KVoMTgBGeSu9HH31QBjo5narwZB5kg+BJhPv2ToNzcFjNhOrRYdpolK5I/nprvQIB2fNyDM";

    // Helper method to register a motor with the hardware map
    public static DcMotor registerMotor(HardwareMap map, String name,
                                        boolean reversed, String mode) {
        DcMotor motor = map.dcMotor.get(name);
        if (reversed)
            motor.setDirection(DcMotor.Direction.REVERSE);
        else
            motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        switch (mode.toLowerCase()) {
            case "position":
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "encoder":
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case "default":
            default:
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return motor;
    }

    // More specific helper method to register motor without
    // worrying about the default RunMode
    public static DcMotor registerMotor(HardwareMap map, String name, boolean reversed) {
        return registerMotor(map, name, reversed, "default");
    }

    // Helper method to register a servo with the hardware map
    public static Servo registerServo(HardwareMap map, String name, boolean reversed) {
        Servo servo = map.servo.get(name);
        if (reversed)
            servo.setDirection(Servo.Direction.REVERSE);
        else
            servo.setDirection(Servo.Direction.FORWARD);
        return servo;
    }

    // Helper method to register a servo, including a
    // default position to move to during initialization
    public static Servo registerServo(HardwareMap map, String name,
                                      boolean reversed, double defaultPos) {
        Servo servo = map.servo.get(name);
        if (reversed)
            servo.setDirection(Servo.Direction.REVERSE);
        else
            servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(defaultPos);
        return servo;
    }

    // Helper method to register a continuous rotation servo
    public static CRServo registerCRServo(HardwareMap map, String name,
                                          boolean reversed, double power) {
        CRServo crServo = map.crservo.get(name);
        if (reversed)
            crServo.setDirection(CRServo.Direction.REVERSE);
        else
            crServo.setDirection(CRServo.Direction.FORWARD);
        crServo.setPower(power);
        return crServo;
    }

    // Helper method to register a continuous rotation servo
    // without providing a default speed during initialization
    public static CRServo registerCRServo(HardwareMap map, String name, boolean reversed) {
        CRServo crServo = map.crservo.get(name);
        if (reversed)
            crServo.setDirection(CRServo.Direction.REVERSE);
        else
            crServo.setDirection(CRServo.Direction.FORWARD);
        return crServo;
    }

    // Accessor method for the license key defined above
    public static String getVuforiaLicenseKey() {
        return VUFORIA_LICENSE_KEY;
    }

    // Method to retrieve the sensor manager from the app context for use in
    // utilizing the built-in sensors in the phone
    public static SensorManager getSensorManager(HardwareMap map) {
        return (SensorManager) map.appContext.getSystemService(Context.SENSOR_SERVICE);
    }

}
