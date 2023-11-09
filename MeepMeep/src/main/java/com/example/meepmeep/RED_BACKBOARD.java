package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RED_BACKBOARD {
    public static void main(String args[]) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        double INIT_X=14;
        double INIT_Y=-61;
        Pose2d startPos = new Pose2d(INIT_X, INIT_Y, Math.toRadians(90.00));

        //Red, backdrop side, center
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(INIT_X +  00,INIT_Y +28))
                                .build()
                );

        //Red, backdrop side, right
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(INIT_X +  7,INIT_Y +23)) //the variables are so this can be easily tested through dashboard


                                .build()
                );
        //Red, backdrop side, left
        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineToLinearHeading(new Pose2d(INIT_X-3, INIT_Y+31, Math.toRadians(180)))
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
