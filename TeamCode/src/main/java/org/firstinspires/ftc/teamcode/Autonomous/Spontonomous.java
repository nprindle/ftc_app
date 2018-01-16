package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Spontonomous", group = "Team Autonomous")
public class Spontonomous extends AutonomousTemplate {

    @Override
    public void auto() {
        encoderDrive();

        grabLeft.setPosition(0.0);
        grabRight.setPosition(0.0);
        lift.setPower(0.3);
        delay(0.5);
        lift.setPower(0.0);
        delay(2);

        runMotors(0.2, 0.5, flicker);
        flickerExt.setPosition(0.0);
        delay();
        colorSensor.enableLed(true);
        while (!isStopRequested()) {
            if (colorSensor.red() > colorSensor.blue()) {
                turn(20, LEFT, 0.2);
                flickerExt.setPosition(0.0);
                delay();
                runMotors(-0.4, 0.5, flicker);
                turn(20, RIGHT, 0.2);
                break;
            } else if (colorSensor.blue() > colorSensor.red()) {
                turn(20, RIGHT, 0.2);
                flickerExt.setPosition(0.0);
                delay();
                runMotors(-0.4, 0.5, flicker);
                turn(20, LEFT, 0.2);
                break;
            }
        }
        colorSensor.enableLed(false);

        strafe(2.5, FT, 0, 0.7);
        drive(-6, IN, 0.3);

        while (runtime.seconds() < 30 && !isStopRequested()) {
            sleep(10);
        }
    }

}
