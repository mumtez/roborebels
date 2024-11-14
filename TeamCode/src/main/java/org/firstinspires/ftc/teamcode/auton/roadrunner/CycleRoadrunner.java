package org.firstinspires.ftc.teamcode.auton.roadrunner;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(name = "CYCLE ROADRUNNER")
public class CycleRoadrunner {
    Pose2d bucket = new Pose2d(new Vector2d(-54, -54), Math.toRadians(225));
    Vector2d leftBlock = new Vector2d(-48, -23.5);
    Pose2d middleBlock = new Pose2d(new Vector2d(-56, -48), Math.toRadians(270));
    Pose2d rightBlock = new Pose2d(new Vector2d(-48, -48), Math.toRadians(270));

    Pose2d park1 = new Pose2d(new Vector2d(-48, -36), Math.toRadians(180));
    Vector2d park2 = new Vector2d(35, -36);
    Vector2d park3 = new Vector2d(48, -62);

    //public static VelConstraint slowVel = new TranslationalVelConstraint(40);
    //public static AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final MecanumDrive drive;

    public Pose2d start = new Pose2d(-10.5, -62, Math.toRadians(90));


    private final Robot robot;

    public FtcDashboard dash = FtcDashboard.getInstance();


    public CycleRoadrunner(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = new MultipleTelemetry(dash.getTelemetry(), opMode.telemetry);
        this.hardwareMap = opMode.hardwareMap;

        this.robot = new Robot(this.opMode);
        this.drive = new MecanumDrive(hardwareMap, start);
    }


    public void run() {

        //bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(bucket, Math.toRadians(260))
                        .build()
        );

        robot.deposit();

        //right block
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(rightBlock, Math.toRadians(100))
                        .build()
        );

        robot.pickUp();

        //bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(bucket, Math.toRadians(200))
                        .build()
        );

        robot.deposit();

        //middle block
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(middleBlock, Math.toRadians(160))
                        .build()
        );

        robot.pickUp();

        //bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToSplineHeading(bucket, Math.toRadians(280))
                        .build()
        );

        robot.deposit();

        //left block
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(leftBlock, Math.toRadians(180))
                        .build()
        );

        robot.pickUp();


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
