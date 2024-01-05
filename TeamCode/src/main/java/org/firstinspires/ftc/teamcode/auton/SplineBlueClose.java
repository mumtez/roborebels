package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue Close Spline Autonimous")
public class SplineBlueClose extends LinearOpMode {
    public FtcDashboard dash = FtcDashboard.getInstance();
    Robot robot;
    private VisionPortal portal;
    BluePropThreshold processor;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, 60, Math.toRadians(-90)));
        processor = new BluePropThreshold();
        robot = new Robot(this);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();

        Position x = processor.getElePos();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(48, 36, Math.toRadians(180)), Math.toRadians(-90))
                        .waitSeconds(3)
                        .strafeTo(new Vector2d(18, 24))
                        .waitSeconds(1)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(6, 60), Math.toRadians(180))
                        .strafeTo(new Vector2d(-36, 60))
                        .setTangent(Math.toRadians(-180))
                        .splineToConstantHeading(new Vector2d(-60, 36), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-54, 12), Math.toRadians(-90))
                        .waitSeconds(2)
                        .setTangent(Math.toRadians(0.1))
                        .splineToConstantHeading(new Vector2d(48, 36), Math.toRadians(90))
                        .build());
       }
    }
