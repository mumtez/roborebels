package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

// CONFIG

//


@Config
public class Robot {
    public static double HORIZONTAL_SLIDE_IN = 0;
    public static double HORIZONTAL_SLIDE_OUT = 0.4;


    public static double INTAKE_OUT = 0.88;

    public static double INTAKE_UP = .32;
    public static double INTAKE_FLAT = .75;

    public static double OUTTAKE_IN = 0.8;
    public static double OUTTAKE_OUT = 0.1;

    public static int VERTICAL_SLIDE_UP = 2800;
    public static int VERTICAL_SLIDE_DOWN = 0;

    public static double GYRO_TURN_P = .055;
    public static double KG = 0.07;
    public static double HEADING_THRESHOLD = 1;

    public final IMU imu;
    public final DcMotor fl, fr, bl, br;
    public final DcMotor slideLeft;
    public final DcMotor slideRight;

    public final DcMotor hang;
    public final ServoImplEx slideOUT;
    public final CRServoImplEx intake;
    public final ServoImplEx flipper, outtake;

    private final LinearOpMode opMode;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        // BULK CACHING
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Drivetrain
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(Direction.REVERSE);
        fr.setDirection(Direction.FORWARD);
        bl.setDirection(Direction.REVERSE);
        br.setDirection(Direction.FORWARD);

        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Hang
        hang = hardwareMap.dcMotor.get("hang");
        hang.setMode(RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Slides
        slideLeft = hardwareMap.dcMotor.get("lu");
        slideRight = hardwareMap.dcMotor.get("ru");

        slideLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setDirection(Direction.REVERSE);
        slideRight.setDirection(Direction.REVERSE);

        //slideOUT.setDirection(Direction.FORWARD);

        slideLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        //slideOUT.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        slideLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(RunMode.RUN_WITHOUT_ENCODER);


        slideOUT = (ServoImplEx) hardwareMap.servo.get("so");
        //slideOUT

        // Intake

        intake = (CRServoImplEx) hardwareMap.crservo.get("in");
        outtake = (ServoImplEx) hardwareMap.servo.get("out");
        flipper = (ServoImplEx) hardwareMap.servo.get("flip");

        flipper.setDirection(Servo.Direction.REVERSE);

        // Sensor
        //intakeSense = hardwareMap.get(DistanceSensor.class, "ins");

    }

    public void initAuton() {
        this.rotateIntakeUp();
        this.outtakeIn();
        this.setHorizontalSlidePos(HORIZONTAL_SLIDE_IN);
        this.fl.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.bl.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.fr.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.br.setMode(RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHorizontalSlidePos(double pos) {
        pos = Range.clip(pos, HORIZONTAL_SLIDE_IN, HORIZONTAL_SLIDE_OUT);
        this.slideOUT.setPosition(pos);
    }

    public void outtakeOut() {
        outtake.setPosition(OUTTAKE_OUT);
    }

    public void outtakeIn() {
        outtake.setPosition(OUTTAKE_IN);
    }


    public void rotateIntakeFlat() {
        this.flipper.setPosition(INTAKE_FLAT);
    }

    public void rotateIntakeUp() {
        this.flipper.setPosition(INTAKE_UP);
    }

    public void rotateIntakeOut() {
        this.flipper.setPosition(INTAKE_OUT);
    }

    public void setVerticalSlidePower(double pow) {
        slideRight.setPower(pow + KG);
        slideLeft.setPower(pow + KG);


    }

    public void setSlideUpPos(int pos, double pow) {
        setVerticalSlidePower(0);

        slideLeft.setTargetPosition(pos);


        slideLeft.setMode(RunMode.RUN_TO_POSITION);
        slideRight.setMode(RunMode.RUN_TO_POSITION);


        setVerticalSlidePower(pow);

        while (this.opMode.opModeIsActive() && Math.abs(slideLeft.getCurrentPosition() - pos) > 30) {
            // Wait for slide to end
        }

        setVerticalSlidePower(0);
        slideLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(RunMode.RUN_WITHOUT_ENCODER);

    }

    public void pickUp() {
        setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT / 2);
        waitTime(500);

        rotateIntakeOut();
        intake.setPower(1);
        waitTime(500);

        setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT);
        waitTime(500);

        intake.setPower(0);
        rotateIntakeUp();
        waitTime(500);

        setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_IN);
        waitTime(500);

        intake.setPower(-1);
        waitTime(500);

        rotateIntakeUp();
    }

    public void deposit() {
        setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_IN / 4);
        setSlideUpPos(Robot.VERTICAL_SLIDE_UP, .8);
        outtakeOut();
        waitTime(1000);

        outtakeIn();
        setSlideUpPos(Robot.VERTICAL_SLIDE_DOWN, 1);
    }

    public double getHeading() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setDriveTrainPower(double frPow, double flPow, double brPow, double blPow) {
        fr.setPower(frPow);
        fl.setPower(flPow);
        br.setPower(brPow);
        bl.setPower(blPow);
    }

    public void turnByGyro(double targetDegrees) {
        this.fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.br.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.bl.setMode(RunMode.RUN_WITHOUT_ENCODER);

        double headingError = targetDegrees - getHeading();
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) {
            headingError -= 360;
        }
        while (headingError <= -180) {
            headingError += 360;
        }

        int x = 0;
        ElapsedTime timer = new ElapsedTime();
        // keep looping while we are still active, and not on heading.
        // Max time: 1/2 second
        while (this.opMode.opModeIsActive() && x < 5 && timer.milliseconds() < 500) {

            headingError = targetDegrees - getHeading();

            // Normalize the error to be within +/- 180 degrees
            while (headingError > 180) {
                headingError -= 360;
            }
            while (headingError <= -180) {
                headingError += 360;
            }

            double turnSpeed = Range.clip(headingError * GYRO_TURN_P, -0.6, 0.6);
            this.setDriveTrainPower(turnSpeed, -turnSpeed, turnSpeed, -turnSpeed);

            if (Math.abs(headingError) <= HEADING_THRESHOLD) {
                x++;
            } else {
                x = 0;
            }

            opMode.telemetry.addData("target", targetDegrees);
            opMode.telemetry.addData("cur", getHeading());
            opMode.telemetry.addData("error", headingError);
            opMode.telemetry.addData("speed", turnSpeed);
            opMode.telemetry.update();
        }

        this.setDriveTrainPower(0, 0, 0, 0);
        this.fl.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.bl.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.fr.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.br.setMode(RunMode.STOP_AND_RESET_ENCODER);
    }

    public void waitTime(double ms) {
        double startTime = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
        }
    }

    public void encodeDriveForward(double disto, double y) {
        int targetTicks = distanceToEncoderTicks(disto);

        setDriveTrainPower(0, 0, 0, 0);

        fr.setTargetPosition(targetTicks + fr.getCurrentPosition());
        fl.setTargetPosition(targetTicks + fl.getCurrentPosition());
        br.setTargetPosition(targetTicks + br.getCurrentPosition());
        bl.setTargetPosition(targetTicks + bl.getCurrentPosition());

        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDriveTrainPower(y, y, y, y);

        while (opMode.opModeIsActive() && fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
            // Do Nothing
        }

        setDriveTrainPower(0, 0, 0, 0);
    }

    public int distanceToEncoderTicks(double distanceMM) {
        double circumference = Math.PI * 96;
        double cpr = 537.7;
        double ticksPerMM = cpr / circumference;
        return (int) (ticksPerMM * distanceMM);
    }

    public void stopReset() {
        this.fl.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.bl.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.fr.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.br.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.waitTime(200);
    }

    public void encodeDriveStrafe(double disto, double x) {
        int targetTicks = distanceToEncoderTicks(disto);

        setDriveTrainPower(0, 0, 0, 0);

        fr.setTargetPosition(-targetTicks + fr.getCurrentPosition());
        fl.setTargetPosition(targetTicks + fl.getCurrentPosition());
        br.setTargetPosition(targetTicks + br.getCurrentPosition());
        bl.setTargetPosition(-targetTicks + bl.getCurrentPosition());

        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDriveTrainPower(x, x, x, x);

        while (opMode.opModeIsActive() && fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
            // Wait for drive to end
        }

        setDriveTrainPower(0, 0, 0, 0);

    }
}