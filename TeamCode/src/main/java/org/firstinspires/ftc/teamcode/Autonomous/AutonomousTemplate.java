package org.firstinspires.ftc.teamcode.Autonomous;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MathUtils;
import org.firstinspires.ftc.teamcode.RobotUtils;

import java.util.ArrayList;
import java.util.concurrent.Callable;

// This class is extended by all autonomous subprograms run by the team
// Each one implements only 'void auto()', but has access to all motors,
// sensors, and helper methods defined in this class
abstract class AutonomousTemplate extends LinearOpMode implements SensorEventListener {

    // Tracks time since beginning, usually used for runtime.seconds();
    public ElapsedTime runtime = new ElapsedTime();

    // Fields used to interface with the Vuforia library to identify relics
    private VuforiaLocalizer            vuforia;
    private VuforiaTrackable            relicTemplate;
    private ArrayList<VuforiaTrackable> trackables;

    // Fields relating to the built-in phone sensors
    private SensorManager mSensorManager;
    // private Sensor mAccelerometer;
    // private Sensor mMagneticField;
    float[] gravity;
    float[] geomagnetic;
    float[] rotationMatrix = new float[9];
    float[] identity       = new float[9];
    float[] orientation    = new float[3];
    float   azimuth        = 0.0f;
    float   roll           = 0.0f;
    float   pitch          = 0.0f;

    DcMotor frontLeft, frontRight, backLeft, backRight, lift, flicker;
    // Array used for block power changes to the wheels
    DcMotor[] wheels;

    // Servos used to move the grabber forwards or backwards
    CRServo topRight, topLeft, bottomRight, bottomLeft;

    // The arms of the block grabber
    Servo grabLeft, grabRight;
    // The servo at the end of the arm used for hitting the balls
    Servo flickerExt;

    ColorSensor colorSensor;

    // Enumerations (usually bad practice, use enums instead)
    // Used to simplify the turn method
    // Set to 1 and -1 so -1 * DIR will yield the opposite direction
    final static int
            RIGHT = 1,
            LEFT  = -1;

    // Units, all in terms of inches (should still use enumerations)
    // These are used for driving and strafing measurements
    // Converting to inches allows MathUtils to calculate encoder ticks
    final static double
            IN = 1.0,
            FT = 12.0,
            M  = 39.37,
            CM = M / 100.0;

    // Fields to simplify identification of the relic
    final static RelicRecoveryVuMark
            VM_UNKNOWN = RelicRecoveryVuMark.UNKNOWN,
            VM_LEFT    = RelicRecoveryVuMark.LEFT,
            VM_CENTER  = RelicRecoveryVuMark.CENTER,
            VM_RIGHT   = RelicRecoveryVuMark.RIGHT;

    // The heart of VuMark identification.
    // In each autonomous, this Callable (like a genericized Runnable),
    // is submitted to an ExecutorService. This allows the robot to search
    // for the relic in the background while performing other tasks, and then
    // blocks the thread until the relic is identified when it needs to make
    // a decision based off of it. This is done by using the Future returned
    // by ExecutorService.submit()
    final Callable<RelicRecoveryVuMark> vuMarkFinder = new
            Callable<RelicRecoveryVuMark>() {
                @Override
                public RelicRecoveryVuMark call() {
                    // Continually search for a mark until one is recognized
                    RelicRecoveryVuMark mark;
                    do
                        mark = getVuMark();
                    while (mark.equals(VM_UNKNOWN));
                    return mark;
                }
            };

    @Override
    public void runOpMode() {
        // Allows Telemetry.Item-style telemetry to work
        telemetry.setAutoClear(false);
        telemetry.clear();

        // Phone sensor registration
        mSensorManager = RobotUtils.getSensorManager(hardwareMap);

        // if (mAccelerometer != null) {
        //     mSensorManager.registerListener(this, mAccelerometer,
        //         SensorManager.SENSOR_DELAY_NORMAL);
        // } else {
        //     telemetry.addLine("No accelerometer available");
        // }
        // if (mMagneticField != null) {
        //     mSensorManager.registerListener(this, mMagneticField,
        //         SensorManager.SENSOR_DELAY_NORMAL);
        // } else {
        //     telemetry.addLine("No magnetic field sensor available");
        // }
        telemetry.addLine("Phone sensors registered");
        telemetry.update();

        // Initialization phase
        // If Tetrix, should be opposite (false, true, false, true)
        frontLeft = RobotUtils.registerMotor(hardwareMap, "left", true, "position");
        frontRight = RobotUtils.registerMotor(hardwareMap, "right", false, "position");
        backLeft = RobotUtils.registerMotor(hardwareMap, "backLeft", true, "position");
        backRight = RobotUtils.registerMotor(hardwareMap, "backRight", false, "position");

        lift = RobotUtils.registerMotor(hardwareMap, "lift", true, "default");
        flicker = RobotUtils.registerMotor(hardwareMap, "flicker", true, "default");

        topLeft = RobotUtils.registerCRServo(hardwareMap, "topLeft", false, 0.0);
        topRight = RobotUtils.registerCRServo(hardwareMap, "topRight", true, 0.0);
        bottomLeft = RobotUtils.registerCRServo(hardwareMap, "bottomLeft", false, 0.0);
        bottomRight = RobotUtils.registerCRServo(hardwareMap, "bottomRight", true, 0.0);

        grabLeft = RobotUtils.registerServo(hardwareMap, "grabLeft", true);
        grabRight = RobotUtils.registerServo(hardwareMap, "grabRight", false);
        flickerExt = RobotUtils.registerServo(hardwareMap, "flickerExt", true);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        wheels = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        telemetry.addData("Status", "initialized");
        telemetry.update();

        // Relic tracking initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                                                        .getIdentifier("cameraMonitorViewId",
                                                                "id", hardwareMap.appContext
                                                                        .getPackageName());
        VuforiaLocalizer.Parameters parameters =
                new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = RobotUtils.getVuforiaLicenseKey();
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackers =
                this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        trackables = new ArrayList<VuforiaTrackable>();
        trackables.addAll(relicTrackers);
        relicTemplate = relicTrackers.get(0);
        relicTemplate.setName("relicTemplate");

        // Wait for "play" button to be pressed
        waitForStart();
        // Start camera for relic identification
        relicTrackers.activate();

        // Set total time to 0
        runtime.reset();

        // Execute the autonomous sequence defined in subclass
        auto();

        // Ensure that all motors are halted after autonomous execution
        stopMotors(frontLeft, frontRight, backLeft, backRight, lift);
    }

    // Each autonomous subprogram will define an auto() method that contains
    // the set of instructions to run in that autonomous
    public abstract void auto();

    // Used for phone sensor readings
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    // Used for phone sensor updates
    @Override
    public void onSensorChanged(SensorEvent event) {
        // assume phone is lying on a table
        // azimuth = angle around z (rotated on table)
        // pitch = angle around x (tilted forwards/backwards)
        // roll = angle around y (tilted left/right)
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
            gravity = event.values.clone();
        else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
            geomagnetic = event.values.clone();
        // calculate orientation using accelerometer and geomagnetic sensors
        SensorManager.getRotationMatrix(rotationMatrix, identity, gravity, geomagnetic);
        SensorManager.getOrientation(rotationMatrix, orientation);
        azimuth = orientation[0];
        pitch = orientation[1];
        roll = orientation[2];
    }

    // Helper method to wait for specified lengths of time
    public void delay(double seconds) {
        sleep((long) (1000.0 * seconds));
    }

    // Generic waiting, usually for a full hardware cycle
    public void delay() {
        idle();
    }

    // Returns false if at least one wheel has completed its path
    // Assures that if one wheel stops, the rest do as well.
    public boolean allWheelsBusy() {
        for (DcMotor motor : wheels)
            if (!motor.isBusy())
                return false;
        return true;
    }

    // Allows for movement forwards of backwards a specified distance
    public void drive(double distance, double units, double power) {
        resetEncoders();
        if (opModeIsActive()) {
            int ticks = MathUtils.distanceToTicks(distance, units);
            for (DcMotor wheel : wheels)
                wheel.setTargetPosition(ticks);
            powerMotors(power, frontLeft, frontRight, backLeft, backRight);
            while (allWheelsBusy() && !isStopRequested())
                sleep(10);
            stopMotors(wheels);
        }
    }

    // Allows for omnidirectional strafing by calculating the relative velocities
    // necessary for each wheel to achieve a strafe
    // direction is given in degrees from the positive x axis, similar to a unit circle
    public void strafe(double distance, double units, int direction, double power) {
        resetEncoders();
        distance *= 1.0 / 1.3875;
        if (opModeIsActive()) {
            float[] powers = MathUtils.calculateVelocities(0.5, direction);
            for (float p : powers)
                telemetry.addLine(Float.toString(p));
            telemetry.update();
            int ticks = MathUtils.distanceToTicks(distance, units);
            frontLeft.setTargetPosition((int) Math.signum(powers[0]) * ticks);
            frontRight.setTargetPosition((int) Math.signum(powers[1]) * ticks);
            backLeft.setTargetPosition((int) Math.signum(powers[2]) * ticks);
            backRight.setTargetPosition((int) Math.signum(powers[3]) * ticks);
            powerMotors(powers[0], frontLeft);
            powerMotors(powers[1], frontRight);
            powerMotors(powers[2], backLeft);
            powerMotors(powers[3], backRight);
            while (allWheelsBusy() && !isStopRequested())
                sleep(10);
            stopMotors(wheels);
        }
        telemetry.clear();
    }

    // Allows for turning a given amount of degrees by assuming the wheel base forms
    // the diameter of a circle and then using that diameter to calculate equivalent
    // ticks of distance
    public void turn(double degrees, int direction, double power) {
        int factor = direction;
        resetEncoders();
        if (opModeIsActive()) {
            int arcTicks = MathUtils.degreesToTicks(degrees);

            frontLeft.setTargetPosition(arcTicks * factor);
            backLeft.setTargetPosition(arcTicks * factor);
            frontRight.setTargetPosition(arcTicks * -factor);
            backRight.setTargetPosition(arcTicks * -factor);

            powerMotors(factor * power, frontLeft, backLeft);
            powerMotors(-factor * power, frontRight, backRight);
            while (allWheelsBusy() && !isStopRequested())
                sleep(10);

            stopMotors(wheels);
        }
    }

    // Method to supply a given power to a provided list of motors
    public void powerMotors(double power, DcMotor... motors) {
        if (opModeIsActive())
            for (DcMotor motor : motors)
                motor.setPower(power);
    }

    // Halt provided list of motors
    public void stopMotors(DcMotor... motors) {
        powerMotors(0, motors);
    }

    // Run a list of motors at a given power for a given time
    public void runMotors(double power, double seconds, DcMotor... motors) {
        powerMotors(power, motors);
        delay(seconds);
        stopMotors(motors);
    }

    // Resets all current encoder positions to 0 and restores
    // run to position mode
    public void resetEncoders() {
        if (opModeIsActive())
            for (DcMotor motor : wheels) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

    }

    // Returns any relic that was spotted by Vuforia, or VM_UNKNOWN otherwise
    public RelicRecoveryVuMark getVuMark() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    // Method used to display a relic
    public String vuMarkToString(RelicRecoveryVuMark marky) {
        if (marky.equals(VM_CENTER))
            return "CENTER";
        else if (marky.equals(VM_LEFT))
            return "LEFT";
        else if (marky.equals(VM_RIGHT))
            return "RIGHT";
        else
            return "UNKNOWN";
    }

    // Moves a servo, but only as far in either direction as it can go
    public void changeServoPos(Servo servo, double d) {
        servo.setPosition(MathUtils.constrainServo(servo.getPosition() + d));
    }

    // Toggle driving with normal continuous power instead of using encoders
    public void powerDrive() {
        for (DcMotor motor : wheels)
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Toggle using encoders instead of driving with power
    // Synonymous with resetting the encoders
    public void encoderDrive() {
        resetEncoders();
    }

}
