package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDTesting extends LinearOpMode {

    DcMotorEx motor;

    //TODO: TUNNIG (https://www.robotsforroboticists.com/pid-control/)
    double integralSum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()){
            double power = PIDControl(100, motor.getCurrentPosition());

            motor.setPower(power);

        }

    }

    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivitive = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * kp) + (derivitive * kd) + (integralSum * ki);
        return output;
    }
}
