package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@Autonomous(name = "AVOID AUTON")
public class AutonAvoid extends LinearOpMode {

    public static int DRIVE_FORWARD = 130;
    public static int DRIVE_STRAFE = -550;
    public static int PARK_DIST = 2800;

    public static int PARK_DIST_2 = -600;

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.initAuton();

        while (this.opModeInInit()) {
            telemetry.addLine("Initialized!");
            telemetry.addData("FL", robot.fl.getCurrentPosition());
            telemetry.addData("FR", robot.fr.getCurrentPosition());
            telemetry.addData("BL", robot.bl.getCurrentPosition());
            telemetry.addData("BR", robot.br.getCurrentPosition());
            telemetry.addData("slideR", robot.slideLeft.getCurrentPosition());
            telemetry.addData("slideR", robot.slideRight.getCurrentPosition());

            telemetry.addData("heading", robot.getHeading());
            telemetry.update();
        }

        robot.encodeDriveForward(DRIVE_FORWARD, .5);
        robot.encodeDriveStrafe(DRIVE_STRAFE, .5);

        robot.turnByGyro(-45);
        robot.stopReset();
        deposit();

        robot.turnByGyro(0);

        robot.encodeDriveForward(-1 * PARK_DIST_2, 0.5);
        robot.waitTime(1000);

        robot.encodeDriveStrafe(PARK_DIST, 0.5);
        robot.encodeDriveForward(PARK_DIST_2, 0.5);

        robot.rotateIntakeFlat();


    }

    public void pickUp() {
        robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT / 2);
        robot.waitTime(500);

        robot.rotateIntakeOut();
        robot.intake.setPower(1);
        robot.waitTime(500);

        robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT);
        robot.waitTime(500);

        robot.intake.setPower(0);
        robot.rotateIntakeUp();
        robot.waitTime(500);

        robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_IN);
        robot.waitTime(500);

        robot.intake.setPower(-1);
        robot.waitTime(500);

        robot.rotateIntakeUp();
    }

    public void deposit() {
        robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_IN / 4);
        robot.setSlideUpPos(Robot.VERTICAL_SLIDE_UP, .8);
        robot.outtakeOut();
        robot.waitTime(1000);

        robot.outtakeIn();
        robot.setSlideUpPos(Robot.VERTICAL_SLIDE_DOWN, 1);
    }
}