package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.MeepMeep.Background;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.awt.Image;
import java.awt.Toolkit;


public class MeepMeepTesting {

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(600);

    Pose2d bucket = new Pose2d(new Vector2d(-54, -54), Math.toRadians(225));




    Image image = Toolkit.getDefaultToolkit().getImage("C:\\Users\\mayom\\StudioProjects\\roborebels24-25\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\IntoTheDeepMeepMeep.png");

    meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_OFFICIAL);
    System.out.println("bg set");

    RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeRedDark())
        .build();



    right.runAction(right.getDrive().actionBuilder(new Pose2d(-10.5, -62, Math.toRadians(90)))
            //Board

        //.strafeToSplineHeading(new Vector2d(-48, -62), Math.toRadians(180))

                            .splineToSplineHeading(bucket, Math.toRadians(260) )
                            .waitSeconds(.2)


                            .splineToLinearHeading(new Pose2d(new Vector2d(-48, -48), Math.toRadians(270)), Math.toRadians(100))
                            .waitSeconds(.2)

                            .splineToSplineHeading(bucket, Math.toRadians(200))
                            .waitSeconds(.2)

                            .splineToLinearHeading(new Pose2d(new Vector2d(-56, -48), Math.toRadians(270)), Math.toRadians(160))
                            .waitSeconds(.2)

                            .splineToSplineHeading(bucket, Math.toRadians(280))
                            .waitSeconds(.2)

                            .strafeToLinearHeading(new Vector2d(-48, -23.5), Math.toRadians(180))
                            .waitSeconds(.2)

                            .splineToSplineHeading(bucket, Math.toRadians(260))
                            .waitSeconds(.2)

                            .splineToSplineHeading(new Pose2d(new Vector2d(-48, -36), Math.toRadians(180)), Math.toRadians(80))
                            .waitSeconds(.2)

                            .strafeToLinearHeading(new Vector2d(35, -36), Math.toRadians(180))
                            .waitSeconds(.2)

                            .strafeToLinearHeading(new Vector2d(48, -62), Math.toRadians(180))
                            .waitSeconds(.2)

        //.waitSeconds(.2)

            .build());



    meepMeep.setBackground(Background. FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(right)

        .start();
  }
}