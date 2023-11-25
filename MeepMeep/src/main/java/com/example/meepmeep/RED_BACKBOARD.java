package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        RoadRunnerBotEntity middleBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineTo(new com.acmerobotics.roadrunner.geometry.Vector2d(INIT_X +  00,INIT_Y +29))
                                .forward(-4)

                                .lineToSplineHeading(new Pose2d(INIT_X +32,INIT_Y +26,Math.toRadians(-180)))
                                .strafeLeft(30)
                                .build()
                );

        //Red, backdrop side, right
        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)

                                .splineTo(new Vector2d(INIT_X+17, INIT_Y+30),Math.toRadians(180))
                                .lineTo(new Vector2d(INIT_X+35,INIT_Y+20))
                                .strafeLeft(15)
                                .build()
                );
        //Red, backdrop side, left
        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)

                                .lineToLinearHeading(new Pose2d(INIT_X-3, INIT_Y+32, Math.toRadians(180)))
                                .forward(-34)
                                .strafeLeft(26)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)

                .addEntity(leftBot)

                .start();
    }
}
