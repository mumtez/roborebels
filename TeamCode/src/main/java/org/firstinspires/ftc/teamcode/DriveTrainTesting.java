package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DriveTrainTesting")
public class DriveTrainTesting extends LinearOpMode {

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
               robot.fl.setPower(1);
           }
           else{
               robot.fl.setPower(0);
           }



           if (gamepad1.b){
               robot.fr.setPower((1));
           }
           else{
               robot.fr.setPower(0);
           }

            if (gamepad1.x){
                robot.bl.setPower(1);
            }
            else{
                robot.bl.setPower(0);
            }

            if (gamepad1.y){
                robot.br.setPower((1));
            }
            else{
                robot.br.setPower(0);
            }
        }
    }
}