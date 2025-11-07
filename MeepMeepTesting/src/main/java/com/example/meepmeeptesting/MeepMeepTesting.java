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
                myBot.getDrive().actionBuilder(new Pose2d(-62, 36, Math.toRadians(0)))
                        .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(315)), Math.toRadians(-20))
                        .splineToLinearHeading(new Pose2d(-11, 30, Math.toRadians(270)), Math.toRadians(0))
                        .strafeTo(new Vector2d(-11,52))
                        .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(315)), Math.toRadians(45))
                        .strafeTo(new Vector2d(20,0))
                        .build()
        );






        mm.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setTheme(new ColorSchemeBlueDark())
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();


    }
}