package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Localization Test")
public class LocalizationTest extends OpMode {
    private PoseUpdater poseUpdater;

    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(new Pose2d(0, 0, Math.toRadians(0)));

        telemetry.addData("Pose Updater:", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        poseUpdater.update();

        telemetry.addData("X", String.format("%.2f", poseUpdater.getPose().position.x));
        telemetry.addData("Y", String.format("%.2f", poseUpdater.getPose().position.y));
        telemetry.addData("Heading", String.format("%.2f", Math.toDegrees(poseUpdater.getPose().heading.toDouble())));
        telemetry.update();
    }
}