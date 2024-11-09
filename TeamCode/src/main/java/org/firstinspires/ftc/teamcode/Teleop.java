package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

    Robot robot;
    double out;

    boolean flipIn = true;
    boolean slideOut = false;
    boolean timerDone = false;

    boolean timerDone2 = false;

    boolean outtaking = false;

    int timer = 60;
    int timer2 = 60;

    double horizontalPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();
        // START

        // LOOP
        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                robot.imu.resetYaw();
            }

            // === FIELD CENTRIC ===

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = AngleUnit.DEGREES.toRadians(robot.getHeading());

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Slow mode
            if (gamepad1.right_bumper) {
                frontLeftPower *= 0.4;
                backLeftPower *= 0.4;
                frontRightPower *= 0.4;
                backRightPower *= 0.4;
            }

            robot.fl.setPower(frontLeftPower);
            robot.bl.setPower(backLeftPower);
            robot.fr.setPower(frontRightPower);
            robot.br.setPower(backRightPower);


            if (gamepad2.dpad_down) {
                slideOut = false;
                timer2 = 0;
                timerDone2 = false;
            }

            if (!timerDone2) {
                timer2++;
            }
            if (timer2 > 35) {
                timerDone2 = true;
            }
            if (timerDone2 && !slideOut) {
                horizontalPos = Robot.HORIZONTAL_SLIDE_IN;
            }

            if (gamepad2.right_stick_y != 0 && robot.slideOUT.getPosition() < 0.4) {
                horizontalPos = Robot.HORIZONTAL_SLIDE_OUT / 4;
            }


            if (gamepad2.dpad_up) {
                ;
                if (robot.slideOUT.getPosition() != Robot.HORIZONTAL_SLIDE_OUT) {
                    timer = 0;
                }
                horizontalPos = Robot.HORIZONTAL_SLIDE_OUT;
                timerDone = false;
                slideOut = true;
            }

            robot.setHorizontalSlidePos(horizontalPos);

            if (!timerDone) {
                timer++;
            }
            if (timer > 35) {
                timerDone = true;
            }

            if (slideOut && timerDone) {
                if (gamepad2.a) {
                    robot.rotateIntakeOut();
                } else {
                    robot.rotateIntakeFlat();
                }
            } else {
                robot.rotateIntakeUp();
            }


            robot.setVerticalSlidePower(-gamepad2.right_stick_y);
            telemetry.addData("stick", gamepad2.right_stick_y);


            if (gamepad1.b) {
                outtaking = true;
            } else {
                outtaking = false;
            }

            if (outtaking && robot.slideUP.getCurrentPosition() > 10) {
                robot.outtakeOut();
            } else {
                robot.outtakeIn();
            }


            if (gamepad2.right_bumper) {
                robot.intake.setPower(1);
            } else if (gamepad2.left_bumper) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0);
            }

            if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5) {
                if (gamepad1.dpad_left) {
                    robot.hang.setPower(1);
                } else if (gamepad1.dpad_right) {
                    robot.hang.setPower(-1);
                } else {
                    robot.hang.setPower(0);
                }
            }


            telemetry.addData("FL", robot.fl.getCurrentPosition());
            telemetry.addData("FR", robot.fr.getCurrentPosition());
            telemetry.addData("BL", robot.bl.getCurrentPosition());
            telemetry.addData("BR", robot.br.getCurrentPosition());
            telemetry.addData("INTAKE ROTATE POS", robot.flipper.getPosition());
            telemetry.addData("INTAKE POW", robot.intake.getPower());
            telemetry.addData("OUTTAKE POS", robot.outtake.getPosition());
            telemetry.addData("H SLIDE POS", robot.slideOUT.getPosition());
            telemetry.addData("V SLIDE ENC", robot.slideUP.getCurrentPosition());
            telemetry.addData("HANG ENC", robot.hang.getCurrentPosition());
            telemetry.update();
        }
    }
}