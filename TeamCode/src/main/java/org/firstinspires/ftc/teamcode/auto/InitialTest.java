package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "AccelCurve")
@Config
public class InitialTest extends LinearOpMode {
    MecanumDrive drive;
    BNO055IMU imu;
    public static double SPEED = 0.1;
    public static double OFFSET = 0.1;
    public static int SAMPLES = 6;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.ACCGYRO;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);

        while (opModeInInit()) {
            Acceleration a = imu.getOverallAcceleration();
            telemetry.addData("ax", a.xAccel);
            telemetry.addData("ay", a.yAccel);
            telemetry.addData("az", a.zAccel);
            telemetry.addData("status", imu.getSystemStatus());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(SPEED, 0), 0));
        sleep(200);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(SPEED + OFFSET, 0), 0));
        double sum = 0;
        for (int i = 0; i < SAMPLES; i++) {
            Acceleration a = imu.getOverallAcceleration();
            Log.d("accel", String.valueOf(a.yAccel)); // Use ay for forward, ax for strafe
            sum += a.yAccel;
            sleep(10);
        }
        Log.d("accel_avg", String.valueOf(sum/SAMPLES));
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
