package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pipeline.AlliancePosition;

@Config
@Autonomous(name = "Blue Near (LEFT) Cycle Auto (2+4)", group = "BLUE Auto", preselectTeleOp = "Manual Drive")
public class BlueLeftCycleAuto extends NearCycleAutoBase {

    @Override
    protected void onInit() {
        super.onInit();
        this.start = BlueLeftAuto.blue_start;
        this.start_forward = BlueLeftAuto.blue_start_forward;
        this.backdrop = BlueLeftAuto.blue_backdrop;
        this.spike = BlueLeftAuto.blue_spike;

        this.cycleStart = new Pose2d[]{
                new Pose2d(spike[0].position.x, 11.5, Math.toRadians(-180)),
                new Pose2d(spike[1].position.x, 11.5, Math.toRadians(-180)),
                new Pose2d(spike[2].position.x, 11.5, Math.toRadians(-180))
        };

        this.stackAlignment = new Pose2d(-45.0, 11.5, Math.toRadians(-180.85));
        this.stackIntake1 = new Pose2d(-58.0, 11.5, Math.toRadians(-180));
        this.stackIntake2 = new Pose2d(-58.5, 12.0, Math.toRadians(-180));
        this.safeTrussPassStop = new Pose2d(-51.0, 11.5, Math.toRadians(-180));

        this.backdropAlignment = new Pose2d(38.0, 11.5, Math.toRadians(-180));

        this.cycleScore = new Pose2d[]{
                new Pose2d(47.8, 33.0, Math.toRadians(-180)),
                new Pose2d(47.8, 33.0, Math.toRadians(-180)),
                new Pose2d(47.8, 36.0, Math.toRadians(-180))
        };

        this.parking = new Pose2d(47.0, 24.0, Math.toRadians(-180));
    }

    protected AlliancePosition getAlliance() {
        return AlliancePosition.BLUE;
    }

    @Override
    protected void printDescription() {
        telemetry.addData("Description", "BLUE Near (Left-Side) Cycle Auto");
    }

}
