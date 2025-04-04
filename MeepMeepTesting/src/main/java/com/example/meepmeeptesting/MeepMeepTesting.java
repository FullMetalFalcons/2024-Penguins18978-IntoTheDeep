package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Set up constants for the size of the robot
        int botWidth = 18;

        // Set up constants for "preset" field locations
        int STARTING_POSITION_Y = -70 + (botWidth/2);
        int STARTING_POSITION_X = -48 + (botWidth/2);

        int SCORING_POSITION_X = -48;
        int SCORING_POSITION_Y = -48;

        int PARKING_POSITION_X = -24;
        int PARKING_POSITION_Y = -12;


        // Create the virtual robot that will move
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(90), Math.toRadians(90), 12)
                .build();


        // Set starting pose and run a sample trajectory (the +0's are use to make the parameter names appear)
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(STARTING_POSITION_X, STARTING_POSITION_Y, Math.toRadians(90)))
                .waitSeconds(0.5)
                // Move into Scoring Position
                .splineToLinearHeading(new Pose2d( SCORING_POSITION_X, SCORING_POSITION_Y, Math.toRadians(225) ), Math.toRadians(180))
                .waitSeconds(1)
                // Pickup 2nd sample
                .strafeToLinearHeading(new Vector2d( SCORING_POSITION_X, SCORING_POSITION_Y+12), Math.toRadians(90))
                .waitSeconds(1)
                // Move into Scoring Position
                .strafeToLinearHeading(new Vector2d( SCORING_POSITION_X, SCORING_POSITION_Y), Math.toRadians(225))
                .waitSeconds(1)
                // Low level ascent
                .strafeToConstantHeading(new Vector2d(-36, SCORING_POSITION_Y))
                .strafeToLinearHeading(new Vector2d( -36, -24), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d( PARKING_POSITION_X, PARKING_POSITION_Y, 0), 0)
                .build());


        // Set up MeepMeep appearances & visual settings
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}