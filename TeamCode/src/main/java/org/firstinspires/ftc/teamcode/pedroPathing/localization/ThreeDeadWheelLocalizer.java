package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double par0YTicks = -2073.6674288446384; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 2130.2641987906904; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = -2293.4870955228225; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    public final IMU imu;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;

    private double lastRawHeadingVel, headingVelOffset;
    private Rotation2d lastHeading;

    private Pose2d pose = new Pose2d(0.0,0.0,0.0);

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick, IMU imu) {
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));

        // TODO: reverse encoder directions if needed
        par0.setDirection(DcMotorSimple.Direction.FORWARD);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.FORWARD);

        this.imu = imu;

        lastPar0Pos = par0.getPositionAndVelocity().position;
        lastPar1Pos = par1.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;

        this.inPerTick = inPerTick;
    }

    public void update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        Log.d("IMU heading: ", Double.toString(angles.getYaw(AngleUnit.RADIANS)));

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            Twist2dDual<Time> t = new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );

            pose = pose.plus(t.value());
            return;
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);
        lastHeading = heading;
        if (Math.abs(headingVel) < Math.toRadians(0)) { // We don't use dead wheels for heading anymore :(
            headingDelta = (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks);
            headingVel =  (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks);
        }

        Log.d("localizer", "par0 pos " + Double.toString(par0PosVel.position));
        Log.d("localizer", "par1 pos " + Double.toString(par1PosVel.position));
        Log.d("localizer", "perp pas " + Double.toString(perpPosVel.position));
        Log.d("localizer", "par0 delta " + Double.toString(par0PosDelta));
        Log.d("localizer", "par1 delta " + Double.toString(par1PosDelta));
        Log.d("localizer", "perp delta " + Double.toString(perpPosDelta));
        Log.d("localizer", "heading delta " + Double.toString(headingDelta));
        Log.d("localizer", "heading vel " + Double.toString(headingVel));
        Log.d("TwistVal1: ", Double.toString((PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks)));
        Log.d("TwistVal2: ", Double.toString((PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks)));
        Log.d("TwistVal3: ", Double.toString((PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta)));
        Log.d("TwistVal4: ", Double.toString((PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity)));

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
//                                (-PARAMS.perpXTicks * headingDelta + perpPosDelta*1.025),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        Log.d("localizer", "delta x " + Double.toString(twist.line.x.value()));
        Log.d("localizer", "delta y " + Double.toString(twist.line.y.value()));

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        pose = pose.plus(twist.value());
        Log.d("Position_x: ", Double.toString(pose.position.x));
        Log.d("Position_y: ", Double.toString(pose.position.y));
        Log.d("Heading: ", Double.toString(pose.heading.toDouble()));
    }

    public void setPoseEstimate(Pose2d poseEstimate) {
        this.pose = poseEstimate;
    }

    public Pose2d getPoseEstimate() {
        return this.pose;
    }

    public void resetHeading(double newHeading) {
        this.pose = new Pose2d(this.pose.position.x, this.pose.position.y, newHeading);
    }
}
