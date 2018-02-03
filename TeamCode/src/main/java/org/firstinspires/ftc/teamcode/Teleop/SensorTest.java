package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "GyroSensor", group = "TeleOp")

public class SensorTest extends OpMode {

    // Gyroscope              gyroscope;
    // HiTechnicNxtGyroSensor hiTechnicNxtGyroSensor;
    // double   degrees  = 0.0;
    // double[] velocDeg = new double[2];

    public void init() {
        // gyroscope = hardwareMap.get(Gyroscope.class, "gyro");
        // hiTechnicNxtGyroSensor = hardwareMap.get(HiTechnicNxtGyroSensor.class, "gyro");
    }

    @Override
    public void init_loop() {

    }

    // simply ensures motors are stopped when start is pressed
    @Override
    public void start() {
//        hiTechnicNxtGyroSensor.calibrate();
    }

    // ensures that motors are halted when telemetry ends
    // otherwise, would run continually for several seconds
    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    public void loop() {
    }

    // public void isTurning() {
    //     double          raw      = hiTechnicNxtGyroSensor.readRawVoltage();
    //     double          bias     = hiTechnicNxtGyroSensor.getBiasVoltage();
    //     double          current  = System.nanoTime();
    //     AngularVelocity velocity = hiTechnicNxtGyroSensor.getAngularVelocity(AngleUnit.DEGREES);
    //     int             adjVeloc = Math.abs(velocity.zRotationRate) < 1 ? 0 : (int) velocity
    //             .zRotationRate;
    //     degrees += ((velocity.acquisitionTime - current) * velocity.zRotationRate / 1000000000);
    //     telemetry.addData("rate", "%.4f deg/s", adjVeloc);
    //     telemetry.addData("deg", "%.4f deg", degrees * 180);
    //     if ((Math.abs(degrees * 180)) > 89 && (Math.abs(degrees * 180)) < 91) {
    //         telemetry.addData("IN THE IF", degrees * 180);
    //     }
    //     telemetry.update();

    // }


    // public double avgVelocity() {
    //     double[] vs      = new double[5];
    //     double   average = 0.0;
    //     double   result  = 0.0;
    //     int      count   = 0;
    //     for (int i = 0; i < 5; i++) {
    //         vs[i] = hiTechnicNxtGyroSensor.getAngularVelocity(AngleUnit.DEGREES).zRotationRate;
    //         average += vs[i] / 5;
    //     }
    //     double stdev = 0;
    //     for (double x : vs) {
    //         stdev += Math.abs(x - average) / 4;
    //     }
    //     for (double x : vs) {
    //         if (x - average < 100 * stdev)
    //             count++;
    //         result += x;
    //     }
    //     return (average);
    // }


}
