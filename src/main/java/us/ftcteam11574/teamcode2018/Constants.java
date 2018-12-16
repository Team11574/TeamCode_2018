package us.ftcteam11574.teamcode2018;

public class Constants {
    static final double LATCH_SERVO_OPEN = 0.8;
    static final double LATCH_SERVO_CLOSED = 0.0;

    private static final double WINCH_SPOOL_DIAMETER = 157.08;
    private static final double WINCH_CHAIN_RATIO = 32.0/16.0;
    private static final double WINCH_MOTOR_CPR = 24;
    private static final double WINCH_MOTOR_RATIO = 60;
    private static final double WINCH_ENCODER_COUNTS_PER_REV =
            WINCH_CHAIN_RATIO * WINCH_MOTOR_RATIO * WINCH_MOTOR_CPR;
    static final double WINCH_ENCODER_COUNTS_PER_MM =
            WINCH_ENCODER_COUNTS_PER_REV / WINCH_SPOOL_DIAMETER;

    private static final double DRIVE_SPROCKET_TEETH = 20;
    private static final double DRIVE_TRACK_PITCH = 9.6775;
    private static final double DRIVE_TRACK_MM_PER_REV =
            DRIVE_SPROCKET_TEETH * DRIVE_TRACK_PITCH;

    private static final double DRIVE_CHAIN_RATIO = 16.0/32.0;
    private static final double DRIVE_MOTOR_CPR = 28;
    private static final double DRIVE_MOTOR_RATIO = 40;
    private static final double DRIVE_ENCODER_COUNTS_PER_REV =
            DRIVE_CHAIN_RATIO * DRIVE_MOTOR_RATIO * DRIVE_MOTOR_CPR;
    static final double DRIVE_ENCODER_COUNTS_PER_MM =
            DRIVE_ENCODER_COUNTS_PER_REV / DRIVE_TRACK_MM_PER_REV ;

    static final double DRIVE_SPEED_DETACH = 0.4;
    static final double DRIVE_SPEED_TO_PARK = 0.7;

    static final double WINCH_SPEED_NORMAL = 0.6;
    static final double WINCH_SPEED_FAST = 1.0;
}
