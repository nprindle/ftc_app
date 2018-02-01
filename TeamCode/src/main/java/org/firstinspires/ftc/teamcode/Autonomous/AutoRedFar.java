package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Autonomous(name = "Auto Red Far", group = "Autonomous")
public class AutoRedFar extends AutonomousTemplate {

    @Override
    public void auto() {
        // Begins running the relic finder in the background
        ExecutorService executor = Executors.newCachedThreadPool();
        // Future will contain the value if it is found by the time it is needed
        Future<RelicRecoveryVuMark> finder = executor.submit(vuMarkFinder);
        Telemetry.Item              phase  = telemetry.addData("Phase:", "").setRetained(true);


        // Wait until end of autonomous or until the player requests a stop
        phase.setValue("Waiting for stop");
        stopMotors(wheels);
        resetEncoders();
        while (runtime.seconds() < 30.0 && !isStopRequested())
            sleep(10);
    }
}

