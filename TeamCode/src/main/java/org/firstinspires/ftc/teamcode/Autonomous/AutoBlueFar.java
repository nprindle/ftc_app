package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Autonomous(name = "Auto Blue Far", group = "Autonomous")
public class AutoBlueFar extends AutonomousTemplate {

    @Override
    public void auto() {
        // Begins running the relic finder in the background
        ExecutorService executor = Executors.newCachedThreadPool();
        // Future will contain the value if it is found by the time it is needed
        Future<RelicRecoveryVuMark> finder = executor.submit(vuMarkFinder);
        Telemetry.Item phase = telemetry.addData("Phase:", "")
                                        .setRetained(true);
        Telemetry.Item markItem = telemetry.addData("VuMark", "")
                                           .setRetained(true);

        // ******* Color Ball *******
        phase.setValue("Color Ball");
        telemetry.update();
        encoderDrive();
        // topLeft.setPower(0.5);
        // bottomRight.setPower(0.5);
        // delay();
        // topLeft.setPower(0.0);
        // bottomRight.setPower(0.0);

        // Lower color sensor to balls
        flicker.setPosition(0.0);
        delay();
        // Turn on color sensor's light
        colorSensor.enableLed(true);
        // Additional field added to show ball that was found
        while (!isStopRequested()) {
            if (colorSensor.red() > colorSensor.blue()) {
                drive(2, IN, 0.3);
                idle();
                flicker.setPosition(0.4);
                drive(-45, IN, 0.5);
                break;
            } else if (colorSensor.blue() > colorSensor.red()) {
                drive(-2, IN, 0.3);
                idle();
                flicker.setPosition(0.4);
                drive(-41, IN, 0.3);
                break;
            }
        }
        telemetry.update();
        // deactivate color sensor
        colorSensor.enableLed(false);

        // ******* Search VuMark *******
        RelicRecoveryVuMark mark;
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

        // ******* Strafe to cryptogram *******
        phase.setValue("Strafe to cryptogram");
        // default strafe distance; VM_LEFT or VM_UNKNOWN
        double strafeDistance = 0.0;
        if (mark.equals(VM_CENTER)) {
            strafeDistance = 7.5;
        } else if (mark.equals(VM_RIGHT)) {
            strafeDistance = 7.5 * 2;
        }
        strafe(strafeDistance, IN, 180, 0.4);

        // ******* Insert block in cryptogram *******
        phase.setValue("Insert block");
        grabLeft.setPosition(1.0);
        grabRight.setPosition(1.0);

        firstFlip.setPower(-0.3);
        delay(1.0);
        firstFlip.setPower(0.0);
        secondFlip.setPosition(1.0);

        delay(1.0);
        secondFlip.setPosition(0.0);

        // Wait until end of autonomous or until the player requests a stop
        phase.setValue("Waiting for stop");
        stopMotors(wheels);
        resetEncoders();
        while (runtime.seconds() < 30.0 && !isStopRequested())
            sleep(10);
    }
}

