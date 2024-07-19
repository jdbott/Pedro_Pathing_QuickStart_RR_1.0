package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Flag;
import org.firstinspires.ftc.teamcode.Lights;
import org.firstinspires.ftc.teamcode.TouchSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import static org.firstinspires.ftc.teamcode.RobotConstants.FieldLocations;

@Autonomous(name = "Pedro Pathing Actions Test", group = "Pedro Auto")
public class PedroPathingActionsTest extends OpMode {

    private final Point redBackdropStartPose = FieldLocations.RED_BACKDROP_START_POSE;
    private final Point redToLeftSpikeMarkMiddlePose = FieldLocations.RED_TO_LEFT_SPIKE_MARK_MIDDLE_POSE;
    private final Point redLeftSpikeMark = FieldLocations.RED_LEFT_SPIKE_MARK;
    private final Point redLeftBackdrop = FieldLocations.RED_LEFT_BACKDROP;
    private final Point redToCornerParkingMiddlePose = FieldLocations.RED_TO_CORNER_PARKING_MIDDLE_POSE;
    private final Point redCornerParking = FieldLocations.RED_CORNER_PARKING;

    private Telemetry telemetryA;
    private Follower follower;
    private Lights lights;
    private Flag flag;
    private TouchSensor touchSensor;
    private Timer pathTimer;
    private Path toSpikeMark, toBackdrop, parkInCorner;
    private int pathState;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose2d(15, -63, Math.toRadians(90)));

        lights = new Lights(hardwareMap);
        flag = new Flag(hardwareMap);
        touchSensor = new TouchSensor(hardwareMap);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        pathTimer = new Timer();
        pathState = 0;
        buildPaths();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        flag.flagDown();

        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void start() {
        follower.followPath(toSpikeMark, true);
        setPathState(0);
    }

    @Override
    public void loop() {
        try {
            follower.update();
        } catch (NullPointerException e) {
            telemetryA.addData("Error", "NullPointerException in follower.update()");
            telemetryA.addData("Message", e.getMessage());
            telemetryA.update();
        }
        telemetryA.update();
        autoPathUpdate();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                // Following the first path to the spike mark
                if (follower.getCurrentTValue() > 0.5) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                }
                if (!follower.isBusy()) {
                    setPathState(1);
                }
                break;

            case 1:
                // Follow next path
                follower.followPath(toBackdrop, true);
                setPathState(2);
                break;

            case 2:
                // Following the second path to the backdrop
                if (follower.getCurrentTValue() > 0.5) {
                    flag.flagUp();
                }
                if (!follower.isBusy()) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    setPathState(3);
                }
                break;

            case 3:
                // Parking
                if (touchSensor.getState()) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    flag.flagDown();
                    follower.followPath(parkInCorner, true);
                    setPathState(4);
                } else {
                    telemetryA.addData("Path Status:", "waiting for touch sensor to be pressed");
                }
                break;

            case 4:
                // Following the path to the corner parking
                if (!follower.isBusy()) {
                    setPathState(-1); // End of paths, request stop
                }
                break;

            default:
                requestOpModeStop();
                // No further action
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        toSpikeMark = new Path(new BezierCurve(
                redBackdropStartPose,
                redToLeftSpikeMarkMiddlePose,
                redLeftSpikeMark));
        toSpikeMark.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(137.6));

        toBackdrop = new Path(new BezierLine(
                redLeftSpikeMark,
                redLeftBackdrop
        ));
        toBackdrop.setLinearHeadingInterpolation(Math.toRadians(137.6), Math.toRadians(180));

        parkInCorner = new Path(new BezierCurve(
                redLeftBackdrop,
                redToCornerParkingMiddlePose,
                redCornerParking
        ));
        parkInCorner.setConstantHeadingInterpolation(Math.toRadians(180));
    }
}