package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Flag;
import org.firstinspires.ftc.teamcode.Lights;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TouchSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Pedro Pathing Actions Test 2", group = "Pedro Auto")
public final class PedroPathingActionsTest2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        final Point redBackdropStartPose = RobotConstants.FieldLocations.RED_BACKDROP_START_POSE;
        final Point redToLeftSpikeMarkMiddlePose = RobotConstants.FieldLocations.RED_TO_LEFT_SPIKE_MARK_MIDDLE_POSE;
        final Point redLeftSpikeMark = RobotConstants.FieldLocations.RED_LEFT_SPIKE_MARK;
        final Point redLeftBackdrop = RobotConstants.FieldLocations.RED_LEFT_BACKDROP;
        final Point redToCornerParkingMiddlePose = RobotConstants.FieldLocations.RED_TO_CORNER_PARKING_MIDDLE_POSE;
        final Point redCornerParking = RobotConstants.FieldLocations.RED_CORNER_PARKING;
        Pose2d currentPos;

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose2d(redBackdropStartPose.getX(), redBackdropStartPose.getX(), Math.toRadians(90)));

        Lights lights = new Lights(hardwareMap);
        Flag flag = new Flag(hardwareMap);
        TouchSensor touchSensor = new TouchSensor(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        flag.flagDown();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Init loop - replaces waitForStart(). Ready to run! Time in init:", getRuntime());
            telemetry.update();
            sleep(50); // Don't waste CPU looping
        }

        timer.reset();

        currentPos = follower.getPose();

        Path purplePath = new Path(new BezierCurve(
                new Point(currentPos),
                redToLeftSpikeMarkMiddlePose,
                redLeftSpikeMark));

        purplePath.setLinearHeadingInterpolation(currentPos.heading.toDouble(), Math.toRadians(180));

        follower.followPath(purplePath);

        while (follower.isBusy()) {
            if (follower.getCurrentTValue() > 0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            }
            follower.update();
        }

        currentPos = follower.getPose();

        Path yellowPath = new Path(new BezierLine(
                new Point(currentPos),
                redLeftBackdrop));

        yellowPath.setConstantHeadingInterpolation(Math.toRadians(180));

        follower.followPath(yellowPath, true);

        while (follower.isBusy()) {
            follower.update();
        }

        while (!touchSensor.getState()) {
            follower.update();
            telemetry.addLine("Waiting for touch sensor to be pressed");
            telemetry.update();
        }

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        flag.flagDown();

        currentPos = follower.getPose();

        Path parkInCorner = new Path(new BezierCurve(
                new Point(currentPos),
                redToCornerParkingMiddlePose,
                redCornerParking
        ));
        parkInCorner.setConstantHeadingInterpolation(Math.toRadians(180));

        follower.followPath(parkInCorner);

        while (follower.isBusy()) {
            follower.update();
        }

        requestOpModeStop();
    }
}