package club.towr5291.functions;

/**
 * Created by LZTDD0 on 11/7/2016.
 */

public abstract class Constants {
    //BEACON
    public static final double BEACON_WIDTH = 21.8;     //entire beacon width
    public static final double BEACON_HEIGHT = 14.5;    //entire beacon height
    public static final double BEACON_WH_RATIO = BEACON_WIDTH / BEACON_HEIGHT; //entire beacon ratio

    public enum ObjectColours {
        OBJECT_RED,
        OBJECT_BLUE,
        OBJECT_RED_BLUE,
        OBJECT_BLUE_RED,
        OBJECT_RED_LEFT,
        OBJECT_RED_RIGHT,
        OBJECT_BLUE_LEFT,
        OBJECT_BLUE_RIGHT,
        UNKNOWN;

        public String toString() {
            switch (this) {
                case OBJECT_RED:
                    return "RED";
                case OBJECT_BLUE:
                    return "BLUE";
                case OBJECT_RED_BLUE:
                    return "RED_BLUE";
                case OBJECT_RED_LEFT:
                    return "RED_LEFT";
                case OBJECT_RED_RIGHT:
                    return "RED_RIGHT";
                case OBJECT_BLUE_RED:
                    return "BLUE_RED";
                case OBJECT_BLUE_LEFT:
                    return "BLUE_LEFT";
                case OBJECT_BLUE_RIGHT:
                    return "BLUE_RIGHT";
                case UNKNOWN:
                default:
                    return "???";
            }
        }

    }
}
