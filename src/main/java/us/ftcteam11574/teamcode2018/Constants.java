package us.ftcteam11574.teamcode2018;

public class Constants {
    static final double LATCH_SERVO_OPEN = 0.8;
    static final double LATCH_SERVO_CLOSED = 0.0;

    static final double WINCH_ENCODER_COUNTS_PER_MM = 157.08 / (60 * 2 * 24);

    static final double DRIVE_ENCODER_COUNTS_PER_MM = (20 * 9.6775) / (40 * 1 * 28);

    static final double DRIVE_SPEED_DETACH = 0.4;
    static final double DRIVE_SPEED_TO_PARK = 0.7;

    static final double WINCH_SPEED_NORMAL = 0.6;
    static final double WINCH_SPEED_FAST = 1.0;
}
