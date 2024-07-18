package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@TeleOp(name = "Lock To Test Pedro", group = "Pedro TeleOp")
public class LockToPedro extends LinearOpMode {

    Follower follower;

    RevBlinkinLedDriver lights;

    boolean holdingInitialized = false;

    @Override
    public void runOpMode() {
        // Initialize the drive with starting position at (0, 0, 0) radians
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose2d(0, 0, Math.toRadians(0)));

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        // Wait for the start button to be pressed
        waitForStart();

        // Exit if stop is requested
        if (isStopRequested()) return;

        // Main loop - runs while the op mode is active and stop is not requested
        while (opModeIsActive() && !isStopRequested()) {
            if (!holdingInitialized) {
                follower.holdPoint(new BezierPoint(new Point(new Pose2d(0, 0, Math.toRadians(0)))), Math.toRadians(0));
                holdingInitialized = true;
            }

            if (follower.getTranslationalError().getMagnitude() > 1 || follower.headingError > Math.toRadians(10)) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            follower.update();
        }
    }
}