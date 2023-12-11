package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.odom.TankDrive;

@Config
@Autonomous(name = "Spline Auton")
public class SplineAuton extends LinearOpMode {

    public static int dfDist = 700;
    public static int leftDist = 500;

    public static int leftAng = 90;
    public static int centerAng = -90;
    public static int lAng = 90;
    public static int rAng = -90;

    public static int lxpos = -40;
    public static int lxpos2 = -20;

    public static int mxpos = -40;
    public static int mxpos2 = -20;

    public static int rxpos = -40;
    public static int rxpos2 = -20;

    public static int ypos = -36;

    Robot robot;
    private VisionPortal portal;
    BluePropThreshold processor;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        processor = new BluePropThreshold();
        robot = new Robot(this);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();

        while (opModeInInit()) {
            telemetry.addData("Location", processor.getElePos());
            telemetry.addData("Left", processor.averagedLeftBox);
            telemetry.addData("Right", processor.averagedRightBox);
            telemetry.addData("Thresh", BluePropThreshold.blueThreshold);
            telemetry.update();
        }

        Position x = processor.getElePos();
        switch (x) {
            case LEFT:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(lxpos, ypos), Math.toRadians(180))
                                .splineTo(new Vector2d(lxpos2, ypos), Math.toRadians(180))
                                .build());


                break;

            case CENTER:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(mxpos, ypos), Math.toRadians(180))
                                .splineTo(new Vector2d(mxpos2, ypos), Math.toRadians(180))
                                .build());


                break;

            case RIGHT:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(rxpos, ypos), Math.toRadians(180))
                                .splineTo(new Vector2d(rxpos2, ypos), Math.toRadians(180))
                                .build());
                break;
        }

        sleep(99999999);

        switch (x) {
            case LEFT:

                break;
            case RIGHT:

                break;

            case CENTER:

                break;
        }
    }
}