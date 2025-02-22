package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class HorizontalSlides {

    public static int MAX_POS = 2000;  //TODO: Should probably tune this so we dont break the slides

    public static int TRANSFER = 430;
    public static int DEFAULT = 600;
    public static int UP = 2000;
    public static int SPECIMEN = 330;
    public static int PRE_TRANSFER = 900;

    public static double kp = 0.01;
    public static double ki = 0;
    public static double kd = 0.0001;
    public static double KG = 0.07;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;
    private int targetPos = DEFAULT;
    private int offset = 0;
    public int position = 0;

    public final DcMotor hSlide;

    public final TouchSensor touch;

    public HorizontalSlides(LinearOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        hSlide = hardwareMap.dcMotor.get("lu");

        hSlide.setDirection(Direction.FORWARD);
        hSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        hSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);

        touch = hardwareMap.touchSensor.get("t");
    }

    public void setMode(RunMode mode) {
        hSlide.setMode(mode);
    }

    public void setPower(double pow) {
        hSlide.setPower(pow);
    }

    public void setTarget(int targetPos) {
        timer.reset();
        lastError = 0;
        integralSum = 0;
        this.targetPos = Math.max(0, Math.min(MAX_POS, targetPos));
    }

    public int getTarget() {
        return this.targetPos;
    }

    private void updatePosition() {
        int curPos = this.hSlide.getCurrentPosition();
        if (touch.isPressed()) {
            this.offset = curPos - VerticalSlides.TRANSFER;
        }
        this.position = curPos - this.offset;
    }

    public boolean atTarget(int threshold) {
        return Math.abs(this.position - this.targetPos) < threshold;
    }

    public void updatePIDControl() {
        this.updatePosition();
        if (this.position < 10 && this.targetPos < 10) {
            this.setPower(0);
            return;
        }

        double error = this.targetPos - this.position;
        double dt = timer.seconds();

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        lastError = error;

        double pow = (error * kp) + (derivative * kd) + (integralSum * ki) + KG;
        timer.reset();

        this.setPower(pow);
    }

}
