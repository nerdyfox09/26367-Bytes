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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(50, -15, Math.toRadians(35)))
                //spin up flywheels
                //run intake
                //stop flywheels
                //stop intake
                .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(90)), Math.toRadians(180)) //pick up more balls
                //run intake here
                .strafeTo(new Vector2d(36,-38))
                .splineToLinearHeading(new Pose2d(50, -15, Math.toRadians(35)), Math.toRadians(20)) //drive back to line
                //stop intake
                //spin up outtake
                //start intake
                //stop outtake
                //stop intake
                .build());

                mm.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setTheme(new ColorSchemeBlueDark())
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();


    }
}