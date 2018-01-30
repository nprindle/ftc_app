package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Autonomous(name = "Auto Blue", group = "Autonomous")
public class AutonomousBlue extends AutonomousTemplate {

    @Override
    public void auto() {
        // Begins running the relic finder in the background 
        ExecutorService executor = Executors.newCachedThreadPool();
        // Future will contain the value if it is found by the time it is needed
        Future<RelicRecoveryVuMark> finder = executor.submit(vuMarkFinder);

        // Telemetry field to track current state of opmode
        Telemetry.Item phase     = telemetry.addData("Phase", "").setRetained(true);
        Telemetry.Item ballColor = telemetry.addData("Ball", "").setRetained(true);
        Telemetry.Item markItem  = telemetry.addData("VuMark", "").setRetained(true);

        grabLeft.setPosition(0.0);
        grabRight.setPosition(0.0);
        lift.setPower(0.3);
        delay(0.5);
        lift.setPower(0.0);
        delay(2);

        delay();
        // Labeled block to delimit subtask
        // Also allows for named block collapsing in OnBot
        colorBall:
        {
            phase.setValue("Color Ball");
            telemetry.update();
            encoderDrive();
            topLeft.setPower(0.5);
            bottomRight.setPower(0.5);
            delay();
            topLeft.setPower(0.0);
            bottomRight.setPower(0.0);

            // Lower color sensor to balls
            runMotors(0.2, 0.5, flicker);
            flickerExt.setPosition(0.0);
            delay();
            // Turn on color sensor's light
            colorSensor.enableLed(true);
            // Additional field added to show ball that was found
            while (!isStopRequested()) {
                if (colorSensor.red() > colorSensor.blue()) {
                    ballColor.setValue("red");
                    // knock off red ball
                    turn(20, LEFT, 0.2);
                    // raise arm
                    flickerExt.setPosition(0.0);
                    delay();
                    runMotors(-0.4, 0.5, flicker);
                    // re-center robot
                    turn(20, RIGHT, 0.2);
                    break;
                } else if (colorSensor.blue() > colorSensor.red()) {
                    ballColor.setValue("blue");
                    // knock off red ball
                    turn(20, RIGHT, 0.2);
                    // raise arm
                    flickerExt.setPosition(0.0);
                    delay();
                    runMotors(-0.4, 0.5, flicker);
                    // re-center robot
                    turn(20, LEFT, 0.2);
                    break;
                } else {
                    // notify driver that no color can be successfully identified,
                    // possibly signaling an error with the color sensor
                    ballColor.setValue(String.format("unknown %d %d %d",
                            colorSensor.red(),
                            colorSensor.blue(),
                            colorSensor.green()));
                    telemetry.update();
                }
            }
            telemetry.update();
            // deactivate color sensor
            colorSensor.enableLed(false);
        }

        telemetry.clear();
        delay();
        RelicRecoveryVuMark mark;
        searchVuMark:
        {
            phase.setValue("Search VuMark");
            telemetry.update();
            encoderDrive();

            // back up off of board
            drive(21, IN, 0.4);
            // move towards pillars
            strafe(2.5, FT, 180, 0.8);
            turn(90, LEFT, 0.3);

            // Force Future to complete, otherwise yield default value
            if (finder.isDone()) {
                try {
                    mark = finder.get();
                } catch (InterruptedException | ExecutionException e) {
                    telemetry.addLine("Future.get() interrupted, defaulting to VM_LEFT");
                    mark = VM_LEFT;
                }
            } else {
                telemetry.addLine("Mark not found, defaulting to VM_LEFT");
                // Cease attempting to find the relic
                finder.cancel(true);
                mark = VM_LEFT;
            }
            markItem.setValue(vuMarkToString(mark));
            telemetry.update();
        }

        delay();
        placeBlock:
        {
            phase.setValue("Place Block");
            telemetry.update();
            encoderDrive();
            if (mark.equals(VM_RIGHT))
                strafe(18.5, IN, 180, 1.0);
            else if (mark.equals(mark.equals(VM_CENTER)))
                strafe(26, IN, 180, 1.0);
            else if (mark.equals(VM_LEFT))
                strafe(33.5, IN, 180, 1.0);
            drive(7, IN, 0.3);
            grabLeft.setPosition(1.0);
            grabRight.setPosition(1.0);
            drive(-3.0, IN, 0.3);
        }

        // Wait until end of autonomous or until the player requests a stop
        phase.setValue("Waiting for stop");
        stopMotors(wheels);
        resetEncoders();
        while (runtime.seconds() < 30.0 && !isStopRequested())
            sleep(10);
    }
}
