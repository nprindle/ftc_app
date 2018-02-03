package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.ExecutionException;

@Autonomous(name = "Auto Blue Far", group = "Autonomous")
public class AutoBlueFar extends AutonomousTemplate {

    @Override
    public void auto() {
        Telemetry.Item phase = telemetry.addData("Phase:", "") .setRetained(true);
        Telemetry.Item markItem = telemetry.addData("VuMark", "") .setRetained(true);

        // ******************** Color Ball ********************
        phase.setValue("Color Ball");
        telemetry.update();
        encoderDrive();

        // Lower color sensor to balls
        flicker.setPosition(0.0);
        delay();
        colorSensor.enableLed(true);
        while (!isStopRequested()) {
            if (colorSensor.red() > colorSensor.blue()) {
                drive(2, IN, 0.3);
                idle();
                flicker.setPosition(0.4);
                drive(-34, IN, 0.5);
                break;
            } else if (colorSensor.blue() > colorSensor.red()) {
                drive(-2, IN, 0.3);
                idle();
                flicker.setPosition(0.4);
                drive(-30, IN, 0.5);
                break;
            }
        }
        telemetry.update();
        // Deactivate color sensor
        colorSensor.enableLed(false);

        // ******************** Search VuMark ********************
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

        // ******************** Strafe to cryptogram ********************
        phase.setValue("Strafe to cryptogram");
        // default strafe distance; corresponds to VM_LEFT or VM_UNKNOWN
        double strafeDistance = 0.0;
        if (mark.equals(VM_CENTER)) {
            strafeDistance = 7.5;
        } else if (mark.equals(VM_RIGHT)) {
            strafeDistance = 7.5 * 2;
        }
        strafe(strafeDistance, IN, 180, 0.4);

        // ******************** Insert block in cryptogram ********************
        phase.setValue("Insert block");
        grabLeft.setPosition(1.0);
        grabRight.setPosition(1.0);

        firstFlip.setTargetPosition(-1194);
        firstFlip.setPower(0.5);
        while (firstFlip.isBusy() && !isStopRequested()) {
            telemetry.clear();
            telemetry.addLine("First flip running");
        }
        firstFlip.setPower(0.0);
        delay();
        secondFlip.setPosition(1.0);
        delay();
        secondFlip.setPosition(0.0);

        // Wait until end of autonomous, or until the player requests a stop
        phase.setValue("Waiting for stop");
        stopMotors(wheels);
        resetEncoders();
        while (runtime.seconds() < 30.0 && !isStopRequested())
            sleep(10);
    }
}

