package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;

@TeleOp(name = "Testing TELEOP", group = "MAIN")
public class TestingTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        NewRobot robot = new NewRobot(this, AllianceColor.RED, false);
        new MotorEncoderTesting(this, robot).run();
        //Same as base teleop but the telem just outputs encoders
    }
}
