package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
@TeleOp(name = "Claw Fix Test", group = "TESTING")
public class ClawFixTesting extends LinearOpMode {

    Robot robot;
    public static double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this);

        robot.slides.setMode(RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);

        // LOOP
        while (opModeIsActive()) {

            robot.slides.setTarget(500);

            robot.slides.updatePIDControl();

            telemetry.addData("SLIDE REFERENCE", robot.slides.position);
            telemetry.update();
        }
    }
}