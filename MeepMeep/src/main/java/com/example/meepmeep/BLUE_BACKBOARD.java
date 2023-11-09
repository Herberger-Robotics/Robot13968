package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BLUE_BACKBOARD {
    public static void main(String args[]) {
        MeepMeep meepMeep = new MeepMeep(800);

        double init_x=14;
        double init_y=61;
        Pose2d startPos = new Pose2d(init_x, init_y, Math.toRadians(270.00));

        //Blue, backdrop side, center
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(init_x +  00,init_y -28))
                                .build()
                );

        //Blue, backdrop side, left
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(init_x + 7,init_y -23)) //the variables are so this can be easily tested through dashboard


                                .build()
                );
        //Blue, backdrop side, right
        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                              //  .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(init_x ,init_y -23)) //the variables are so this can be easily tested through dashboard
                               // .turn(Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(init_x-3, init_y-31, Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .addEntity(myBot3)

                .start();
    }
}
