package org.firstinspires.ftc.teamcode.odom.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.odom.TankDrive;

public final class SplineTest extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
      MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -60, 0));

      waitForStart();

      Actions.runBlocking(
          drive.actionBuilder(drive.pose)
              .splineTo(new Vector2d(0, 0), Math.PI / 3)
              .splineTo(new Vector2d(12, -60), Math.PI * 3)
              .build());
    } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
      TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

      waitForStart();

      Actions.runBlocking(
          drive.actionBuilder(drive.pose)
              .splineTo(new Vector2d(30, 30), Math.PI / 2)
              .splineTo(new Vector2d(60, 0), Math.PI)
              .lineToXSplineHeading(30, 0)
              .build());


    } else {
      throw new AssertionError();
    }
  }
}
