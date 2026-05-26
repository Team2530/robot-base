package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient to hold robot-wide numerical or
 * boolean constants. This class should not be used for any other purpose. All
 * constants should be declared globally (i.e. public static). Avoid putting
 * anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class MetaConstants {
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

    public static final class RobotConstants {
        public static final Distance WIDTH = Inches.of(25);
        public static final Distance LENGTH = Inches.of(30);

        public static final Mass TOTAL_MASS = Kilogram.of(107);
        public static final MomentOfInertia MOMENT_OF_INERTIA =
            KilogramSquareMeters.of(6.883);
    }

    public static final class DriveConstants {
        public static final LinearVelocity MAX_ROBOT_VELOCITY =
            MetersPerSecond.of(4.2);
        public static final AngularVelocity MAX_ROBOT_RAD_VELOCITY =
            RotationsPerSecond.of(1);

        public static final class Control {
            public static final Dimensionless REGULAR_DRIVE_MULT =
                Percent.of(100);
            public static final Dimensionless TURTLE_DRIVE_MULT =
                Percent.of(25);
            public static final Dimensionless ROCK_DRIVE_MULT =
                Percent.of(15);
            public static final Frequency MULTIPLIER_SLEW_RATE =
                Percent
                .of(25)
                .per(Second);

            public static final class Deadband {
                public static final double X = 0.1;
                public static final double Y = 0.1;
                public static final double Z = 0.08;

                // the radius beyond the center of the joystick (from 0 to
                // 1) after which the angle-based heading control activates
                public static final double HEADING = 0;
            };

            // for angle-based heading control
            public static final class HeadingPID {
                public static final double P = 0.01;
                public static final double I = 0;
                public static final double D = 0;
                public static final double F = 0;
                public static final double IZ = 0;
            }

            // multiply the output of the drive motor by cos(angular_error)
            public static final boolean USE_COSINE_COMPENSATION = true;

            // correction for heading skew when rotating
            // (see)[https://yet-another-software-suite.github.io/YAGSL/javadocs/swervelib/SwerveDrive.html#setAngularVelocityCompensation(boolean,boolean,double)]
            public static final class AngularCompensation {
                public static final boolean ENABLE_IN_TELEOP = false;
                public static final boolean ENABLE_IN_AUTO = false;

                // expected values are between -0.15 and 0.15
                public static final double COMPENSATION_COEFFICIENT = 0.1;
            }
        }

        public static final class IMU {
            public static final boolean INVERTED = false;
        }

        public static final class Modules {
            public static final Distance WHEEL_DIAMETER = Inches.of(4);
            public static final double WHEEL_FRICTION_COEFFICIENT = 2.255;

            public static final Voltage OPTIMAL_VOLTAGE = Volts.of(12);
            public static final Current DRIVE_CURRENT_LIMIT = Amps.of(120);
            public static final Current STEER_CURRENT_LIMIT = Amps.of(120);

            // the minimum number of seconds it takes the motor to go from 0 to
            // full throttle
            public static final Time DRIVE_RAMP = Seconds.of(0);
            public static final Time STEER_RAMP = Seconds.of(0);

            // the minimum voltage it takes for the given motor to move
            public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.23);
            public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.19);

            public static final class Gearing {
                public static final class Drive{
                    public static final double LIGHT = 7.03;
                    public static final double RIDICULUS = 6.03;
                    public static final double LUDICRUS = 5.27;
                }

                public static final double ANGLE = 26.09;
            }


            public static final class Offsets {
                // distance left of the center of the robot
                private static final Distance BASE_X = Inches.of(9.75);
                // distance forward of the center of the robot
                private static final Distance BASE_Y = Inches.of(12.263);


                // im making the assumption that the swerve module placement is
                // symmetrical on both axes
                public static final class FL {
                    public static final Distance X = BASE_X;
                    public static final Distance Y = BASE_Y;
                    // the absolute encoder offset
                    public static final Angle ANGLE = Rotations.of(0.073975);
                    public static final boolean ENCODER_INVERTED = false;
                    public static final boolean DRIVE_INVERTED = false;
                    public static final boolean ANGLE_INVERTED = true;
                }
                public static final class FR {
                    public static final Distance X = BASE_X;
                    public static final Distance Y = BASE_Y.times(-1);
                    public static final Angle ANGLE = Rotations.of(0.918457);
                    public static final boolean ENCODER_INVERTED = false;
                    public static final boolean DRIVE_INVERTED = false;
                    public static final boolean ANGLE_INVERTED = true;
                }
                public static final class BL {
                    public static final Distance X = BASE_X.times(-1);
                    public static final Distance Y = BASE_Y;
                    public static final Angle ANGLE = Rotations.of(0.653564);
                    public static final boolean ENCODER_INVERTED = false;
                    public static final boolean DRIVE_INVERTED = false;
                    public static final boolean ANGLE_INVERTED = true;
                }
                public static final class BR {
                    public static final Distance X = BASE_X.times(-1);
                    public static final Distance Y = BASE_Y.times(-1);
                    public static final Angle ANGLE = Rotations.of(0.246826);
                    public static final boolean ENCODER_INVERTED = false;
                    public static final boolean DRIVE_INVERTED = false;
                    public static final boolean ANGLE_INVERTED = true;
                }
            }
            public static final class PID {
                // for the drive motors on the modules
                public static final class Drive {
                    public static final double P = 6.5;
                    public static final double I = 0;
                    public static final double D = 0.03;
                    public static final double F = 0.2;
                    public static final double IZ = 0;
                }

                // for the steer motors on the modules
                public static final class Angle {
                    public static final double P = 80;
                    public static final double I = 0;
                    public static final double D = 0;
                    public static final double F = 0;
                    public static final double IZ = 0;
                }
            }
        }

        public static class CANIDs {
            public static final int IMU = 0;

            public static class Modules {
                public static final class FL {
                    public static final int DRIVE = 10;
                    public static final int STEER = 11;
                    public static final int CANCODER = 12;
                }

                public static final class FR {
                    public static final int DRIVE = 7;
                    public static final int STEER = 8;
                    public static final int CANCODER = 9;
                }

                public static final class BL {
                    public static final int DRIVE = 4;
                    public static final int STEER = 5;
                    public static final int CANCODER = 6;
                }

                public static final class BR {
                    public static final int DRIVE = 1;
                    public static final int STEER = 2;
                    public static final int CANCODER = 3;
                }
            }
        }
    }

    public static final class LimelightConstants {
        private static final AprilTagFieldLayout TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public final static HashMap<Integer, Pose2d> TAG_POSES =
            new HashMap<Integer, Pose2d>() {{
                for (int i = 0; i < TAG_LAYOUT.getTags().size(); ++i) {
                    if (TAG_LAYOUT.getTagPose(i + 1).isPresent())
                        put(i, TAG_LAYOUT.getTagPose(i + 1).get().toPose2d());
                }
            }};

        public static final int[] VALID_TAGS = {
                1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
                19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
                35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48
            };
    }

    public static final class ChoreoConstants {
        public static final class PID {
            public static final class Translation {
                public static final double P = 7.0;
                public static final double I = 0.0;
                public static final double D = 0.0;
            }

            public static final class Heading {
                public static final double P = 7.0;
                public static final double I = 0.0;
                public static final double D = 0.02;
            }
        }
    }
}
