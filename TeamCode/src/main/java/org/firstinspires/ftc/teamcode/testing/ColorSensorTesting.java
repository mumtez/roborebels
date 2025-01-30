package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config

@TeleOp(name = "Color Test", group = "TESTING")
public class ColorSensorTesting extends LinearOpMode {

    Robot robot;

    public static double COLOR_THRESHOLD = 0.02;
    public static double YELLOW_THRESHOLD = 0.01;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();
        // START

        // LOOP
        while (opModeIsActive()) {
            NormalizedRGBA colors = robot.intake.senseColor(); // Important: only make 1 i2c call per loop

            if (robot.intake.senseColor().blue > COLOR_THRESHOLD){
                gamepad1.rumble(5000);
                gamepad2.rumble(5000);
            }


            telemetry.addData("Blue in bot", robot.intake.senseColor().blue > COLOR_THRESHOLD);
            telemetry.addData("Red in bot",  robot.intake.senseColor().red > COLOR_THRESHOLD);
            telemetry.addData("Yellow in bot",  robot.intake.senseColor().red > YELLOW_THRESHOLD || robot.intake.senseColor().red > YELLOW_THRESHOLD);

            telemetry.addData("Red",  robot.intake.senseColor().red);
            telemetry.addData("Blue", robot.intake.senseColor().blue);
            telemetry.update();


        }
    }
}