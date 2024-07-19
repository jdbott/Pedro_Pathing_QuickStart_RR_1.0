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
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import static org.firstinspires.ftc.teamcode.RobotConstants.FieldLocations;

@Autonomous(name = "Pedro Pathing 2+6", group = "Pedro Auto")
public class PedroPathing2_6 extends OpMode {

    private final Point redBackdropStartPose = FieldLocations.RED_BACKDROP_START_POSE;
    private final Point redToLeftSpikeMarkMiddlePose = FieldLocations.RED_TO_LEFT_SPIKE_MARK_MIDDLE_POSE;
    private final Point redLeftSpikeMark = FieldLocations.RED_LEFT_SPIKE_MARK;
    private final Point redLeftBackdrop = FieldLocations.RED_LEFT_BACKDROP;
    private final Point redToCornerParkingMiddlePose = FieldLocations.RED_TO_CORNER_PARKING_MIDDLE_POSE;
    private final Point redCornerParking = FieldLocations.RED_CORNER_PARKING;
    private final Point redToStackStart = FieldLocations.RED_TO_STACK_START;
    private final Point redToStackMiddlePose = FieldLocations.RED_TO_STACK_MIDDLE_POSE;
    private final Point redStack1 = FieldLocations.RED_STACK1;
    private final Point redStack2 = FieldLocations.RED_STACK2;
    private final Point redRightBackdrop = FieldLocations.RED_RIGHT_BACKDROP;

    private Telemetry telemetryA;
    private Follower follower;
    private Lights lights;
    private Flag flag;
    private Timer pathTimer;
    private Path toSpikeMark, toBackdrop, parkInCorner, toStack1, toStack2, backToBackdrop;
    private int pathState;
    private int cycleCount = 0;
    private Path chosenStackPath;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose2d(15, -63, Math.toRadians(90)));
        lights = new Lights(hardwareMap);
        flag = new Flag(hardwareMap);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        buildPaths();
        pathState = 0;
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
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 3:
                // Parking
                follower.followPath(parkInCorner, true);
                setPathState(4);
                break;

            case 4:
                // Following the path to the corner parking
                if (follower.getCurrentTValue() > 0.5) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
                if (!follower.isBusy()) {
                    setPathState(-1); // End of paths, request stop
                }
                break;

            case 5:
                // Going to stack or parking
                if (cycleCount >= 3) {
                    setPathState(3); // Go to parking
                } else {
                    chosenStackPath = cycleCount < 2 ? toStack1 : toStack2;
                    chosenStackPath.setConstantHeadingInterpolation(Math.toRadians(180));
                    follower.followPath(chosenStackPath, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Setting heading towards end of path
                if (follower.getCurrentTValue() > 0.6) {
                    chosenStackPath.setConstantHeadingInterpolation(Math.toRadians(135));
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                }
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

            case 7:
                // Coming back
                follower.followPath(backToBackdrop, true);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                setPathState(8);
                break;

            case 8:
                // Waiting until we are back before starting process again
                if (!follower.isBusy()) {
                    cycleCount++;
                    setPathState(5);
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

        // Ensure the toStack path starts from the current pose
        toStack1 = new Path(new BezierCurve(
                redToStackStart,
                redToStackMiddlePose,
                redStack1
        ));
        toStack1.setConstantHeadingInterpolation(Math.toRadians(180));

        toStack2 = new Path(new BezierCurve(
                redToStackStart,
                redToStackMiddlePose,
                redStack2
        ));
        toStack2.setConstantHeadingInterpolation(Math.toRadians(180));

        // Ensure the backToBackdrop path starts from the current pose
        backToBackdrop = new Path(new BezierCurve(
                redToStackMiddlePose,
                redToStackStart,
                redRightBackdrop
        ));
        backToBackdrop.setConstantHeadingInterpolation(Math.toRadians(180));
    }
}