package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BlueFarMeepMeepTest {

    public static void main(String[] args) {

        MeepMeep mm = new MeepMeep((800));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(62, -16, Math.toRadians(0)))
                        .splineToLinearHeading(new Pose2d(55, -5, Math.toRadians(30)), Math.toRadians(0))
                        .stopAndAdd(() -> { })   // outtake on
                        .waitSeconds(1)
                        .stopAndAdd(() -> { })   // intake on
                        .waitSeconds(3)
                        .stopAndAdd(() -> { })   // outtake off
                        .stopAndAdd(() -> { })   // intake off

                        .splineToLinearHeading(new Pose2d(37, -28, Math.toRadians(270)), Math.toRadians(180))
                        .stopAndAdd(() -> { })   // intake on

                        .strafeTo(new Vector2d(36, -48))
                        .waitSeconds(1)
                        .stopAndAdd(() -> { })   // intake off

                        .splineToLinearHeading(new Pose2d(55, -5, Math.toRadians(30)), Math.toRadians(20))
                        .stopAndAdd(() -> { })   // outtake on
                        .stopAndAdd(() -> { })   // intake on
                        .waitSeconds(3)

                        .stopAndAdd(() -> { })   // outtake off
                        .stopAndAdd(() -> { })   // intake off

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