package org.firstinspires.ftc.teamcode.auton.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;


@Autonomous(name = "BASIC ROADRUNNER")
public class BasicRoadrunner extends LinearOpMode{
    Pose2d bucket = new Pose2d(new Vector2d(-54, -54), Math.toRadians(225));

    Pose2d park1 = new Pose2d(new Vector2d(-48, -36), Math.toRadians(180));
    Vector2d park2 = new Vector2d(35, -36);
    Vector2d park3 = new Vector2d(48, -62);

    private  LinearOpMode opMode;
    private  Telemetry telemetry;
    private  HardwareMap hardwareMap;
    private  MecanumDrive drive;

    public Pose2d start = new Pose2d(-10.5, -62, Math.toRadians(90));

    private  Robot robot;

    public FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(dash.getTelemetry(), opMode.telemetry);
        hardwareMap = opMode.hardwareMap;

        robot = new Robot(this);
        drive = new MecanumDrive(hardwareMap, start);

        //bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(bucket, Math.toRadians(260))
                        .build()
        );

        robot.deposit();

        //park
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(park1, Math.toRadians(80))
                        .waitSeconds(.5)

                        .strafeToLinearHeading(park2, Math.toRadians(180))
                        .waitSeconds(.5)

                        .strafeToLinearHeading(park3, Math.toRadians(180))
                        .build()

        );


    }

}

