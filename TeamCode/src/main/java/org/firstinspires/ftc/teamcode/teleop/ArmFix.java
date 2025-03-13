package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name = "Arm Fix")
public class ArmFix extends LinearOpMode {

    NewRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NewRobot(this);

        // LOOP
        while (opModeIsActive()) {
            if (gamepad1.cross) {
                robot.claw.clawUpArm.setPosition(Claw.upArmTransfer);
            }
        }
    }
}