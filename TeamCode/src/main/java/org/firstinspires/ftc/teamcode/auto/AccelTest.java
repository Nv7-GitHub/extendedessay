package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.io.FileNotFoundException;

@Autonomous(name = "Acceleration and Current Draw Test")
@Config
public class AccelTest extends LinearOpMode {
    MecanumDrive drive;
    Logging log;
    DcMotor enc;
    public static double SPEED_INITIAL = 1.5; // Volts
    public static double[] TEST_SPEED = {1, 1.2, 1.4, 1.6, 1.8}; // Volts
    public static int SAMPLES = 3;

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    double dir = 0;
    public void setDrivePower(double voltage) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(voltage/getBatteryVoltage()*dir, 0), 0));
    }

    public void testSpeed(double increase) {
        long start = System.currentTimeMillis();
        long prevT = System.nanoTime();
        double prevPos = enc.getCurrentPosition();
        double prevVel = 0;
        int samples = 0;
        double totalCurrent = 0;
        double totalAccel = 0;
        setDrivePower(SPEED_INITIAL + increase);
        while (System.currentTimeMillis() - start < 150) {
            double dt = (System.nanoTime() - prevT)/1000000000.0;
            double vel = (enc.getCurrentPosition() - prevPos)/dt;
            double accel = (vel - prevVel)/dt;
            prevT = System.nanoTime();
            prevPos = enc.getCurrentPosition();
            prevVel = vel;

            totalAccel += Math.abs(accel);
            totalCurrent += drive.getDriveCurrent();
            samples++;
        }

        log.Write(new double[]{increase, SPEED_INITIAL, totalAccel/samples, totalCurrent/samples});
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        try {
            log = new Logging(new String[]{"Voltage Increase", "Initial Voltage", "Average Acceleration", "Average Current Drawn"});
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        enc = hardwareMap.get(DcMotor.class, "leftRear");

        // Actually run the test
        for (double testSpeed : TEST_SPEED) {
            dir = 1;
            for (int i = 0; i < SAMPLES; i++) {
                setDrivePower(SPEED_INITIAL);
                sleep(1000);

                testSpeed(testSpeed);
            }

            dir = -1;
            for (int i = 0; i < SAMPLES; i++) {
                setDrivePower(SPEED_INITIAL);
                sleep(1000);

                testSpeed(testSpeed);
            }
        }
        setDrivePower(0);

        log.Close();
    }
}
