package org.firstinspires.ftc.teamcode.auton.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;



@Config
@Autonomous(name = "CYCLE ROADRUNNER", group = "ROADRUNNER")
public class ClawBucketRoadRunner extends LinearOpMode {

    public static Pose2d START = new Pose2d(-30.5, -62, Math.toRadians(0));

    public static Vector2d bucketVec = new Vector2d(-52, -52);

    public static Pose2d bucket;
    public static Vector2d leftBlock = new Vector2d(-48, -23.5);
    public static Pose2d middleBlock = new Pose2d(new Vector2d(-56, -48), Math.toRadians(90));
    public static Pose2d rightBlock = new Pose2d(new Vector2d(-48, -48), Math.toRadians(90));

    public static Pose2d park1 = new Pose2d(new Vector2d(-48, -36), Math.toRadians(180));
    public static Vector2d park2 = new Vector2d(35, -36);
    public static Vector2d park3 = new Vector2d(48, -62);

    public static double turnBlockAngleDeg = 60;

    //public static VelConstraint slowVel = new TranslationalVelConstraint(40);
    //public static AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

    private Robot robot;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {


        // INIT
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot = new Robot(this);
        drive = robot.drive;
        drive.localizer.setPose(START);

        bucket = new Pose2d(bucketVec, Math.toRadians(45));
        
        //drive = new MecanumDrive(hardwareMap, START);

        robot.initAuton();
        waitForStart();
        // START

        //bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .setTangent(90)
                        .splineToLinearHeading(bucket, Math.toRadians(225))
                        .build()
        );

        robot.waitTime(500);

        placeBucket();

        robot.waitTime(500);

        //middle block
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineToLinearHeading(middleBlock, Math.toRadians(160))
                        .build()
        );

        pickUp();

        //bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(bucketVec, 45)
                        .build()
        );

        placeBucket();

        /*

        //robot.deposit();

        //left block
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(leftBlock, Math.toRadians(180))
                        .build()
        );

        //robot.pickUp();

        //bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineToSplineHeading(bucket, Math.toRadians(260))
                        .build()
        );

        //robot.deposit();

        //park
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineToSplineHeading(park1, Math.toRadians(80))
                        .waitSeconds(.5)

                        .strafeToLinearHeading(park2, Math.toRadians(180))
                        .waitSeconds(.5)

                        .strafeToLinearHeading(park3, Math.toRadians(180))
                        .build()
        );

        */
    }

    public void placeBucket() {

        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_UP, 0.8);

        robot.endSlideUpPos(Robot.VERTICAL_SLIDE_UP);

        //robot.waitTime(5000);

        robot.claw.setBucket();

        robot.waitTime(1550);

        robot.claw.clawOpen();

        robot.waitTime(1050);

        robot.claw.setWall();

        robot.waitTime(1050);


    }

    public void pickUp(){
        robot.startSlideUpPos(800, 0.8);
        robot.claw.setWall();
        robot.waitTime(1000);

        robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_TRANSFER + 0.07);
        robot.waitTime(1000);


        robot.rotateIntakeDown();
        robot.intake.setPower(-1);
        robot.waitTime(500);


        robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT);
        robot.waitTime(3000);

        robot.rotateIntakeFlat();


        robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_TRANSFER);
        robot.waitTime(300);

        robot.intake.setPower(0);
        robot.waitTime(700);

        robot.claw.setTransfer();
        robot.waitTime(1000);

        robot.claw.clawOpen();
        robot.waitTime(1000);

        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_DEFAULT, 0.8);
        robot.waitTime(1000);

        robot.claw.clawClose();

    }

}

