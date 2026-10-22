package frc.robot.util.constants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public final class DriveConstants {
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
}
