package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MathUtils;
import org.firstinspires.ftc.teamcode.RobotUtils;

// Phases: init, init_loop, start, loop, stop
@TeleOp(name = "8573 TeleOp", group = "TeleOp")
public class TeamTeleOp extends OpMode {

    // Keeps track of time since beginning of execution
    private ElapsedTime runtime = new ElapsedTime();

    // Wheels
    // Lift controls the raising/lowering of the grabber arms in order to stack blocks
    private DcMotor frontLeft, frontRight,
            backLeft, backRight,
            lift;

    // The arm used in autonomous for hitting the balls
    private DcMotor flicker;
    private Servo flickerExt;

    // The four servos to extend the grabber
    private CRServo topRight, topLeft, bottomRight, bottomLeft;
    // The arms of the block grabber
    private Servo grabLeft, grabRight;
    // Variable used to cut the speed of the wheels
    // Also used to manage acceleration when driving
    private double scale = 0;
    // Used to track the position of the block grabbing arms
    private double grabLeftPos = 0.0, grabRightPos = 0.0;

    @Override
    public void loop() {
        // All joystick values, with the annoying negative y problem corrected
        float
                left_x = gamepad1.left_stick_x,
                left_y = -gamepad1.left_stick_y,
                right_x = gamepad1.right_stick_x,
                right_y = -gamepad1.right_stick_y,
                left_x_2 = gamepad2.left_stick_x,
                left_y_2 = -gamepad2.left_stick_y,
                right_x_2 = gamepad2.right_stick_x,
                right_y_2 = -gamepad2.right_stick_y;

        // Left trigger will halve the speed
        // Right trigger will double the speed
        // Right bumper will slowly accelerate
        if (gamepad1.left_trigger > 0.1) {
            scale = 0.25;
        } else if (gamepad1.right_trigger > 0.1) {
            scale = 1.0;
        } else if (gamepad1.right_bumper) {
            if (gamepad1.x)
                scale = 0;
            else if (scale < Math.hypot(left_x, left_y))
                scale += 0.05;
        } else {
            scale = 0.5;
        }

        // Use joystick positions to calculate strafing velocities
        float[] vs = MathUtils.calculateVelocities(left_x, left_y, right_x, right_y);
        frontLeft.setPower(vs[0] * scale);
        frontRight.setPower(vs[1] * scale);
        backLeft.setPower(vs[2] * scale);
        backRight.setPower(vs[3] * scale);

        // Use the right joystick on gamepad 2 to control the upper lift
        if (Math.abs(right_y_2) > 0.1) {
            topRight.setPower(right_y_2);
            bottomLeft.setPower(right_y_2);
        } else {
            topRight.setPower(0);
            bottomLeft.setPower(0);
        }

        // The left joystick similarly controls the lower lift
        if (Math.abs(left_y_2) > 0.1) {
            topLeft.setPower(left_y_2);
            bottomRight.setPower(left_y_2);
        } else {
            topLeft.setPower(0);
            bottomRight.setPower(0);
        }
        // The following would also work:
        // topLeft.setPower(left_y_2);
        // bottomRight.setPower(left_y_2);

        // 'a' and 'b' of gamepad 2 open and close the block grabbers
        if (gamepad2.a) {
            grabLeftPos = MathUtils.constrainServo(grabLeftPos - 0.1);
            grabRightPos = MathUtils.constrainServo(grabRightPos - 0.1);
        } else if (gamepad2.b) {
            grabLeftPos = MathUtils.constrainServo(grabLeftPos + 0.1);
            grabRightPos = MathUtils.constrainServo(grabRightPos + 0.1);
        }
        grabLeft.setPosition(grabLeftPos);
        grabRight.setPosition(grabRightPos);

        // The triggers of gamepad 2 will control the lift speed, with
        // the right trigger raising it and the left lowering it
        if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
            lift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        } else {
            lift.setPower(0);
        }

        // The flicker occasionally falls because it's a DC motor, so
        // gamepad 1 can lift it back up if necessary
        flicker.setPower(gamepad1.a ? 0.3 : gamepad1.b ? -0.3 : 0.0);

        // Telemetry data to be shown every cycle
        // Float formatting used in an attempt to align the numbers in the
        // non-monospace telemetry
        telemetry.addData("Seconds elapsed", (int) runtime.seconds());
        telemetry.addLine(String.format("GL:%.02f", grabLeft.getPosition()));
        telemetry.addLine(String.format("GR:%.02f", grabRight.getPosition()));
        telemetry.addLine("Wheels: FL, FR, BL, BR");
        telemetry.addLine(String.format("%+.02f", frontLeft.getPower()));
        telemetry.addLine(String.format("%+.02f", frontRight.getPower()));
        telemetry.addLine(String.format("%+.02f", backLeft.getPower()));
        telemetry.addLine(String.format("%+.02f", backRight.getPower()));
        telemetry.update();
    }

    @Override
    public void init() {
        // Allows telemetry to be updated every loop
        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialization phase
        // If Tetrix, should be opposite (false, true, false, true)
        frontLeft = RobotUtils.registerMotor(hardwareMap, "left", true, "default");
        frontRight = RobotUtils.registerMotor(hardwareMap, "right", false, "default");
        backLeft = RobotUtils.registerMotor(hardwareMap, "backLeft", true, "default");
        backRight = RobotUtils.registerMotor(hardwareMap, "backRight", false, "default");
        lift = RobotUtils.registerMotor(hardwareMap, "lift", true, "default");

        topLeft = RobotUtils.registerCRServo(hardwareMap, "topLeft", true);
        topRight = RobotUtils.registerCRServo(hardwareMap, "topRight", false);
        bottomLeft = RobotUtils.registerCRServo(hardwareMap, "bottomLeft", true);
        bottomRight = RobotUtils.registerCRServo(hardwareMap, "bottomRight", false);

        grabLeft = RobotUtils.registerServo(hardwareMap, "grabLeft", true, grabLeftPos);
        grabRight = RobotUtils.registerServo(hardwareMap, "grabRight", false, grabRightPos);

        flicker = RobotUtils.registerMotor(hardwareMap, "flicker", true, "default");
        flickerExt = RobotUtils.registerServo(hardwareMap, "flickerExt", true);

        telemetry.addData("Status", "Initialization complete");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    // simply ensures motors are stopped when start is pressed
    @Override
    public void start() {
        powerMotors(0, frontLeft, frontRight, backLeft, backRight, lift);
        runtime.reset();
    }

    // ensures that motors are halted when telemetry ends
    // otherwise, would run continually for several seconds
    @Override
    public void stop() {
        powerMotors(0, frontLeft, frontRight, backLeft, backRight, lift);
        telemetry.addData("Status", "Stopped");
        telemetry.update();
        runtime.reset();
    }

    // Helper method to make block power changes to a given list of motors
    public void powerMotors(double power, DcMotor... motors) {
        for (DcMotor motor : motors)
            motor.setPower(power);
    }

}
