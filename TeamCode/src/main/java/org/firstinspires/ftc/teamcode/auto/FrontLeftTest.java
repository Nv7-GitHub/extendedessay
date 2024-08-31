package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class FrontLeftTest extends LinearOpMode {
    public static double POWER = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "leftFront");
        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(POWER);
            telemetry.addData("POWER", POWER);
            telemetry.update();
        }
    }
}
