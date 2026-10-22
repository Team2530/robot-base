package frc.robot.util.constants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.function.BooleanSupplier;


/*
 * this file contains information more relavant to the game and context beyond
 * the robot, like alliance and game piece values
 */
public final class MetaConstants {
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get()
            : Alliance.Blue;
    }

    public static BooleanSupplier isRed = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return getAlliance() == Alliance.Red;
        }
    };

    public static final class Controllers {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class Logging {
        public static final boolean LOG_INTO_FILE_ENABLED = true;
    }

    public static final class Field {
        public static final LinearAcceleration GRAVITY =
            MetersPerSecondPerSecond.of(9.81);

        public static final Distance LENGTH = Inches.of(651.22);
        public static final Distance WIDTH = Inches.of(317.69);
    }

    public static final class Game {
        /*
         * I used this for season specific constants, so, of course, its
         * empty now.
         */
    }
}
