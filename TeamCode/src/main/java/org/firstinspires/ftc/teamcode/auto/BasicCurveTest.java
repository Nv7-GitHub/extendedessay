package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.io.FileNotFoundException;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Basic Curve Test")
@Config
public class BasicCurveTest extends LinearOpMode {
    MecanumDrive drive;
    Logging log;
    public static double SPEED_INITIAL = 1.5; // Volts
    public static double SPEED_FINAL = 3; // Volts

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

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        try {
            log = new Logging(new String[]{"Time", "Voltage", "Position", "Velocity", "Acceleration", "Current Draw"});
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        DcMotor enc = hardwareMap.get(DcMotor.class, "leftRear");
        enc.setDirection(DcMotorSimple.Direction.REVERSE);

        // Actually run the test
        // 0 power
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        long start = System.currentTimeMillis();
        long prevT = System.nanoTime();
        double prevPos = enc.getCurrentPosition();
        double prevVel = 0;
        while (System.currentTimeMillis() - start < 200) {
            double dt = (System.nanoTime() - prevT)/1000000000.0;
            double vel = (enc.getCurrentPosition() - prevPos)/dt;
            double accel = (vel - prevVel)/dt;
            prevT = System.nanoTime();
            prevPos = enc.getCurrentPosition();
            prevVel = vel;
            log.Write(new double[]{System.currentTimeMillis() - start, 0, -prevPos, -vel, -accel, drive.getDriveCurrent()});
        }

        // SPEED_INITIAL voltage
        start = System.currentTimeMillis();
        for (int i = 1; i <= 3; i++) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(SPEED_INITIAL/getBatteryVoltage(), 0), 0));
            while (System.currentTimeMillis() - start < 1000*i + 400*(i-1)) {
                double dt = (System.nanoTime() - prevT)/1000000000.0;
                double vel = (enc.getCurrentPosition() - prevPos)/dt;
                double accel = (vel - prevVel)/dt;
                prevT = System.nanoTime();
                prevPos = enc.getCurrentPosition();
                prevVel = vel;
                log.Write(new double[]{System.currentTimeMillis() - start, SPEED_INITIAL, -prevPos, -vel, -accel, drive.getDriveCurrent()});
            }


            // SPEED_FINAL
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(SPEED_FINAL/getBatteryVoltage(), 0), 0));
            while (System.currentTimeMillis() - start < 1400*i) {
                double dt = (System.nanoTime() - prevT)/1000000000.0;
                double vel = (enc.getCurrentPosition() - prevPos)/dt;
                double accel = (vel - prevVel)/dt;
                prevT = System.nanoTime();
                prevPos = enc.getCurrentPosition();
                prevVel = vel;
                log.Write(new double[]{System.currentTimeMillis() - start, SPEED_FINAL, -prevPos, -vel, -accel, drive.getDriveCurrent()});
            }
        }

        log.Close();
    }
}
