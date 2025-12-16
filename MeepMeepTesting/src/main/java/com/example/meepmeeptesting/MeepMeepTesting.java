package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);
        int side = 1; // 1 = current side (blue), -1 = mirrored (red)
        // --------------------------
        // Adjustable depth
        // --------------------------
        double collectDepth = -48 * side;   // ← change this only

        // --------------------------
        // Key poses
        // --------------------------
        Vector2d shootingPos = new Vector2d(-24, -14);
        double shootingRot = Math.toRadians(-125) * side;
        double collectingRot = Math.toRadians(90) * side;

        double shootWaitTime = 4.2;

        double collectingAttackOffset = 0;

        // Mirror all Y coordinates
        Vector2d mirroredShootingPos = new Vector2d(shootingPos.x, shootingPos.y * side);
        Vector2d collect1 = new Vector2d(-12 - collectingAttackOffset, -30 * side);
        Vector2d collect2 = new Vector2d(12 - collectingAttackOffset, -30 * side);
        Vector2d collect3 = new Vector2d(35 - collectingAttackOffset, -30 * side);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(
                        60, 60,
                        Math.toRadians(180),
                        Math.toRadians(180),
                        15
                )
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(
                                new Pose2d(-49, -49 * side, Math.toRadians(-125))
                        )

                        // 1️⃣ Move to shooting position
                        .strafeToLinearHeading(mirroredShootingPos, shootingRot)
                        .waitSeconds(shootWaitTime)

                        // 2️⃣ Collect Run #1
                        .strafeToLinearHeading(collect1, collectingRot)
                        .strafeToLinearHeading(new Vector2d(collect1.x + collectingAttackOffset, collectDepth), collectingRot)

                        // Return + Shoot #1
                        .strafeToLinearHeading(mirroredShootingPos, shootingRot)
                        .waitSeconds(shootWaitTime)

                        // 3️⃣ Collect Run #2
                        .strafeToLinearHeading(collect2, collectingRot)
                        .strafeToLinearHeading(new Vector2d(collect2.x + collectingAttackOffset, collectDepth), collectingRot)

                        // Return + Shoot #2
                        .strafeToLinearHeading(mirroredShootingPos, shootingRot)
                        .waitSeconds(shootWaitTime)

                        // 4️⃣ Collect Run #3
                        .strafeToLinearHeading(collect3, collectingRot)
                        .strafeToLinearHeading(new Vector2d(collect3.x + collectingAttackOffset, collectDepth), collectingRot)



                        // Return + Shoot #3
                        .strafeToLinearHeading(mirroredShootingPos, shootingRot)

                        .build()
        );

        meepMeep
                .setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
