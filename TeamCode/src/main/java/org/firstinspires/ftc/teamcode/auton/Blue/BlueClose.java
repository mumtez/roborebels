package org.firstinspires.ftc.teamcode.auton.Blue;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auton.CycleDirection;
import org.firstinspires.ftc.teamcode.auton.ParkPosition;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class BlueClose {

  // FOR NO TEETH
  public static Pose2d BACKDROP_START = new Pose2d(13 - 1, 61, Math.toRadians(270));


  public static Vector2d PARK_CORNER = new Vector2d(60, 64);
  public static Vector2d PARK_CENTER = new Vector2d(55, 12);


  //public static VelConstraint slowVel = new TranslationalVelConstraint(40);
  //public static AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

  private final LinearOpMode opMode;
  private final Telemetry telemetry;
  private final HardwareMap hardwareMap;

  private final double delaySeconds;
  private final CycleDirection cycle;
  private final ParkPosition parkPosition;
  private final Vector2d parkVec;
  private final Robot robot;
  private final MecanumDrive drive;
  public FtcDashboard dash = FtcDashboard.getInstance();

  public BlueClose(LinearOpMode opMode, CycleDirection cycle, ParkPosition parkPosition,
      double delaySeconds) {
    this.opMode = opMode;
    this.telemetry = new MultipleTelemetry(dash.getTelemetry(), opMode.telemetry);
    this.hardwareMap = opMode.hardwareMap;
    this.cycle = cycle;
    this.delaySeconds = delaySeconds;

    this.parkPosition = parkPosition;
    if (Objects.requireNonNull(parkPosition) == ParkPosition.CENTER) {
      this.parkVec = PARK_CENTER;
    } else {
      this.parkVec = PARK_CORNER;
    }

    this.drive = new MecanumDrive(hardwareMap, BACKDROP_START);
    this.robot = new Robot(this.opMode);
  }

  public void run() {


    Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                    // Spike
                    .splineTo(new Vector2d(20, 20), 45)
                    .build()
    );



    robot.waitTime(100);
  }
}