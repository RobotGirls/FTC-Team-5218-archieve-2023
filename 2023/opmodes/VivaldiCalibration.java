package opmodes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/**
 * Created by Lizzie on 10/20/2018.
 */
public class VivaldiCalibration {
    public static final double BUNGEE_BOX_DEPLOY = 0.6;
    public static final double BUNGEE_BOX_STOW = -0.3;

    public static final double MARKER_DEPLOYED = 60.0 / 256.0;
    public static final double MARKER_STOWED = 170.0/256.0;
    public static final int LIFT_ENCODER_COUNT = 16200;
    public static final int BUNGEE_ENCODER_COUNT = 5;
    public static final double LIFT_LEFT_UP = -0.8;
    public static final double LIFT_LEFT_DOWN = 0.8;
    public static final double LIFT_RIGHT_UP = -0.8;
    public static final double LIFT_RIGHT_DOWN = 0.8;
    public static final double LIFT_STOP = 0.0;
    public static final double FOUR_BAR_UP = -0.6;
    public static final double FOUR_BAR_DOWN = 0.6;
    public static final double GYRO_MULTIPLIER = 0.3 / 10.0;
    public static final int OFFSET_SLOP = 12;

    // light constants for autonomous initialization
    public static final RevBlinkinLedDriver.BlinkinPattern MINERAL_LEFT_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;
    public static final RevBlinkinLedDriver.BlinkinPattern MINERAL_CENTER_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
    public static final RevBlinkinLedDriver.BlinkinPattern MINERAL_RIGHT_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    public static final RevBlinkinLedDriver.BlinkinPattern MINERAL_UNKNOWN_PATTERN = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE;

    // light constants for autonomous paths
    public static final RevBlinkinLedDriver.BlinkinPattern AUTONOMOUS_HANGING_PATTERN = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
    public static final RevBlinkinLedDriver.BlinkinPattern AUTONOMOUS_LANDED_PATTERN = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
    public static final RevBlinkinLedDriver.BlinkinPattern AUTONOMOUS_MARKER_DROP_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
    public static final RevBlinkinLedDriver.BlinkinPattern AUTONOMOUS_PARKING_PATTERN = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
    public static final RevBlinkinLedDriver.BlinkinPattern AUTONOMUOS_DONE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.WHITE;

    public static final RevBlinkinLedDriver.BlinkinPattern ENDGAME_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    public static final RevBlinkinLedDriver.BlinkinPattern TELEOP_PATTERN = RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE;

    public static final double TURN_SPEED = -0.1;
}