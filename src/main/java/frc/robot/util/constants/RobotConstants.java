package frc.robot.util.constants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;



/*
 * this file contains information that pertains to the actions of the entire
 * robot as a whole, like its dimensions and mass
 *
 * also, all of the CANIDs have been centralized here, for a convenient lookup
 */
public final class RobotConstants {
    public static final Distance WIDTH = Inches.of(25);
    public static final Distance LENGTH = Inches.of(30);

    public static final Mass TOTAL_MASS = Kilogram.of(107);
    public static final MomentOfInertia MOMENT_OF_INERTIA =
        KilogramSquareMeters.of(6.883);


    public static final class CANIDs {
        public static final class Drive {
            public static final int IMU = 0;

            public static final class FL {
                public static final int DRIVE = 10;
                public static final int STEER = 11;
                public static final int ENCODER = 12;
            }

            public static final class FR {
                public static final int DRIVE = 7;
                public static final int STEER = 8;
                public static final int ENCODER = 9;
            }

            public static final class BL {
                public static final int DRIVE = 4;
                public static final int STEER = 5;
                public static final int ENCODER = 6;
            }

            public static final class BR {
                public static final int DRIVE = 1;
                public static final int STEER = 2;
                public static final int ENCODER = 3;
            }
        }

        public static final int PDH = 13;
    }
}
