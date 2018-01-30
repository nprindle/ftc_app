package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MathUtils;
import org.firstinspires.ftc.teamcode.RobotUtils;

@TeleOp(name = "8573 TeleOp", group = "TeleOp")

public class TeamTeleOp extends OpMode {
    // Keeps track of time since beginning of execution
    private ElapsedTime runtime = new ElapsedTime();

    // Wheels
    private DcMotor frontLeft, frontRight,
            backLeft, backRight, firstFlip;

    // The arm used in autonomous for hitting the balls
    private Servo flicker, secondFlip;

    // The four servos to extend the grabber
    private CRServo leftFly, rightFly;
    // The arms of the block grabber
    private Servo grabLeft, grabRight;
    // Variable used to cut the speed of the wheels
    // Also used to manage acceleration when driving
    private double scale       = 0;
    // Used to track the position of the block grabbing arms
    private double grabLeftPos = 0.0, grabRightPos = 0.5, secondFlipPos = 0.0, flickerPos = 0.4;

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

        if (Math.abs(left_y_2) > 0.1) {
            telemetry.addLine("Entered the if statement of the first flip");
            telemetry.addData("First Flip Power:", left_y_2);
            firstFlip.setPower(left_y_2);
        } else {
            firstFlip.setPower(0);
        }

        if (Math.abs(right_y_2) > 0.1) {
            if (right_y_2 < 0) {
                secondFlipPos = MathUtils.constrainServo(secondFlipPos - 0.05);
            } else if (right_y_2 > 0) {
                secondFlipPos = MathUtils.constrainServo(secondFlipPos + 0.05);
            }
        }
        secondFlip.setPosition(secondFlipPos);
        telemetry.addData("Second Flip Pos in loop", secondFlip.getPosition());

        if (gamepad2.left_trigger > 0.1) {
            leftFly.setPower(1.0);
            rightFly.setPower(1.0);
        } else if (gamepad2.right_trigger > 0.1) {
            leftFly.setPower(-1.0);
            rightFly.setPower(-1.0);
        } else {
            leftFly.setPower(0.0);
            rightFly.setPower(0.0);
        }

        telemetry.addData("Flicker Pos in the loop", flicker.getPosition());

        if (gamepad2.x) {
            firstFlip.setPower(0.8);
            delay(0.5);
            firstFlip.setPower(0.0);
            grabLeft.setPosition(1.0);
            grabRight.setPosition(1.0);
            firstFlip.setPower(-0.8);
            delay(0.3);
            firstFlip.setPower(0.0);
        }

        // 'a' and 'b' of gamepad 2 open and close the block grabbers
        if (gamepad2.a) {
            grabLeftPos = MathUtils.constrainServo(grabLeftPos - 0.05);
            grabRightPos = MathUtils.constrainServo(grabRightPos - 0.05);
        } else if (gamepad2.b) {
            grabLeftPos = MathUtils.constrainServo(grabLeftPos + 0.05);
            grabRightPos = MathUtils.constrainServo(grabRightPos + 0.05);
        }
        grabLeft.setPosition(grabLeftPos);
        grabRight.setPosition(grabRightPos);
        telemetry.addData("grabLeftPos", grabLeftPos);
        telemetry.addData("grabRightPos", grabRightPos);


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

        firstFlip = RobotUtils.registerMotor(hardwareMap, "firstFlip", false, "default");

        leftFly = RobotUtils.registerCRServo(hardwareMap, "leftFly", true, 0.0);
        rightFly = RobotUtils.registerCRServo(hardwareMap, "rightFly", false, 0.0);

        grabLeft = RobotUtils.registerServo(hardwareMap, "grabLeft", true, grabLeftPos);
        grabRight = RobotUtils.registerServo(hardwareMap, "grabRight", false, grabRightPos);

        flicker = RobotUtils.registerServo(hardwareMap, "flicker", false, flickerPos);
        secondFlip = RobotUtils.registerServo(hardwareMap, "secondFlip", false, secondFlipPos);

        telemetry.addData("Status", "Initialization complete");
        telemetry.addData("Flicker Pos", flicker.getPosition());
        telemetry.addData("Second Flip Pos", secondFlip.getPosition());
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    // simply ensures motors are stopped when start is pressed
    @Override
    public void start() {
        powerMotors(0, frontLeft, frontRight, backLeft, backRight);
        runtime.reset();
    }

    public void delay(double seconds) {
        try {
            Thread.sleep((long) (1000.0 * seconds));
        } catch (InterruptedException e) {
        }
    }

    // ensures that motors are halted when telemetry ends
    // otherwise, would run continually for several seconds
    @Override
    public void stop() {
        powerMotors(0, frontLeft, frontRight, backLeft, backRight);
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
