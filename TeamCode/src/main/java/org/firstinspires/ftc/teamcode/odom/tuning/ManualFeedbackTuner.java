package org.firstinspires.ftc.teamcode.odom.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.odom.TankDrive;
import org.firstinspires.ftc.teamcode.odom.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.odom.TwoDeadWheelLocalizer;

@Disabled
public final class ManualFeedbackTuner extends LinearOpMode {

  public static double DISTANCE = 64;

  @Override
  public void runOpMode() throws InterruptedException {
    if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
      MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

      if (drive.localizer instanceof TwoDeadWheelLocalizer) {
        if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0
            && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
          throw new RuntimeException(
              "Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }
      } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
        if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0
            && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0
            && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
          throw new RuntimeException(
              "Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }
      }
      waitForStart();

      while (opModeIsActive()) {
        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(0, DISTANCE), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(0))
                .build());
      }
    } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
      TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

      if (drive.localizer instanceof TwoDeadWheelLocalizer) {
        if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0
            && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
          throw new RuntimeException(
              "Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }
      } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
        if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0
            && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0
            && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
          throw new RuntimeException(
              "Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }
      }
      waitForStart();

      while (opModeIsActive()) {
        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(DISTANCE)
                .lineToX(0)
                .build());
      }
    } else {
      throw new RuntimeException();
    }
  }
}
