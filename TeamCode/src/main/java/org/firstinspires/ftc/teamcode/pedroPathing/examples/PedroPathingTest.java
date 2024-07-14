package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Lights;

@Autonomous(name = "Pedro Pathing Test")
public class PedroPathingTest extends OpMode {
    private Telemetry telemetryA;
    private Follower follower;
    private PathChain path;
    Lights lights;

    // HEADING DOES NOT MATTER HERE WHEN USING FOR POINTS
    private final Pose2d startPose = new Pose2d(15, -63, Math.toRadians(90));
    private final Pose2d toLeftSpikeRedIntermediatePose = new Pose2d(15, -37.2, Math.toRadians(180));
    private final Pose2d leftSpikeRed = new Pose2d(9.6, -34.8, Math.toRadians(180));

    private final Pose2d leftBackdropRed = new Pose2d(49.5, -26.2, Math.toRadians(180));

    @Override
    public void init() {
        lights = new Lights(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(startPose),
                        new Point(toLeftSpikeRedIntermediatePose),
                        new Point(leftSpikeRed)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addParametricCallback(0.5, () -> lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA))
                .addPath(new BezierLine(
                        new Point(follower.getPose()),
                        new Point(leftBackdropRed)
                ))
                .addTemporalCallback(0, () -> lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED))
                .addParametricCallback(0.75, () -> telemetryA.addData("Yay telemetery", "!!"))
                .build();

        follower.followPath(path);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if (follower.isBusy()) {
            follower.telemetryDebug(telemetryA);
        }
    }
}