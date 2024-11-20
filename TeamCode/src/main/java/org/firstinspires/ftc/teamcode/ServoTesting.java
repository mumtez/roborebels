package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ServoTesting")
public class ServoTesting extends LinearOpMode {

    Robot robot;

    boolean slideOut = false;
    boolean outtaking = false;

    int timer = 60;
    int timer2 = 60;

    boolean timerDone = false;
    boolean timerDone2 = false;

    double horizontalPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();
        // START

        // LOOP
        while (opModeIsActive()) {
           if (gamepad1.a){
               robot.slideOUT.setPosition(0.5);
           }
           else{
               robot.slideOUT.setPosition(0);
           }



           if (gamepad1.b){
               robot.flipper.setPosition((0.5));
           }
           else{
               robot.flipper.setPosition(0);
           }


            if (gamepad1.x){
                robot.intake.setPower(1);
            }
            else{
                robot.intake.setPower(0);
            }

            if (gamepad1.y){
                robot.outtake.setPosition((0.5));
            }
            else{
                robot.outtake.setPosition(0);
            }
        }
    }
}