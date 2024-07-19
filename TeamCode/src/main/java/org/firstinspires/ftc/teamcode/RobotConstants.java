package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public final class RobotConstants {
    // Prevent instantiation
    private RobotConstants() {}

    public static final class FieldLocations {
        public static final Point RED_BACKDROP_START_POSE = new Point(15, -63, Point.CARTESIAN);
        public static final Point RED_TO_LEFT_SPIKE_MARK_MIDDLE_POSE = new Point(15, -37.2, Point.CARTESIAN);
        public static final Point RED_LEFT_SPIKE_MARK = new Point(9.6, -34.8, Point.CARTESIAN);
        public static final Point RED_LEFT_BACKDROP = new Point(49.5, -26.2, Point.CARTESIAN);
        public static final Point RED_TO_CORNER_PARKING_MIDDLE_POSE = new Point(45, -52, Point.CARTESIAN);
        public static final Point RED_CORNER_PARKING = new Point(55, -60, Point.CARTESIAN);
        public static final Point RED_TO_STACK_START = new Point(18, -61, Point.CARTESIAN);
        public static final Point RED_TO_STACK_MIDDLE_POSE = new Point(-31.5, -61, Point.CARTESIAN);
        public static final Point RED_STACK1 = new Point(-56.25, -41, Point.CARTESIAN);
        public static final Point RED_STACK2 = new Point(-55.5, -30, Point.CARTESIAN);
        public static final Point RED_RIGHT_BACKDROP = new Point(49.5, -46, Point.CARTESIAN);
    }

    public static final class FlagServoThings {
        public static final double FLAG_DOWN = 0.5;
        public static final double FLAG_UP = 0.8;
    }
}