package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    public static void main(String[] args) {

        MeepMeep mm = new MeepMeep((800));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(58, 13, Math.toRadians(-30)))

                        // Spin up outtake and shoot (simulated delay only in MeepMeep)
                        .waitSeconds(1)

                        // Drive to pick up more rings
                        .splineToLinearHeading(new Pose2d(37, 30, Math.toRadians(90)), Math.toRadians(0))

                        .strafeTo(new Vector2d(36, 48))

                        .waitSeconds(1)

                        // Drive back to shooting line
                        .splineToLinearHeading(new Pose2d(58, 13, Math.toRadians(-50)), Math.toRadians(-20))

                        .waitSeconds(3)

                        // Park or back up
                        .lineToX(30)

                        .build()
        );


        mm.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setTheme(new ColorSchemeBlueDark())
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();


    }
}