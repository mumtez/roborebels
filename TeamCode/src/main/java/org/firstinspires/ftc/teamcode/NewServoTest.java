package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "NewServoTest")
public class NewServoTest extends LinearOpMode {



    RobotTesting robot;


    double x = 0;
    double y = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotTesting(this);

        waitForStart();
        // START

        // LOOP
        while (opModeIsActive()) {


            x -= gamepad1.right_stick_y/1000;


            x = Math.max(.005, Math.min(x,.4));
            y = 1 / (1 + (Math.exp(-4*x) ) );
            //telemetry.addData("x post sig: ", x);

            //x *= (2.0/5.0);


            robot.slideOUT.setPosition(y);


            //robot.slideOUT.setPosition(  (Math.log( (double) Math.round(gamepad1.right_stick_y*1000d) / 1000d) * 10)   );
            //robot.slideOUT.setPosition(   -(double) Math.round(gamepad1.right_stick_y*1000d) / 1000d   );


            if (gamepad1.b){
                //robot.slideOUT.setPosition(0.5);
            }

            telemetry.addData("Button: ", gamepad1.a);
            telemetry.addData("x: ", x);
            telemetry.addData("intake pos: ", robot.slideOUT.getPosition());

            telemetry.update();
        }
    }
}