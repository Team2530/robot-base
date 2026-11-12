package frc.robot.util.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.AngularAcceleration;


public class ManipulatorConstants {
    public static final class ElevatorConstants {

        public static final class PortMotor {
            public static final int CAN_ID = 15;
            public static final boolean INVERTED = false;
        }

        public static final class StarboardMotor {
            public static final int CAN_ID = 15;
            public static final boolean INVERTED = false;
        }

        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.1;
        public static final double kV = 0.106;
        public static final double kA = 0.001;
        public static final double kG = 0.15;

        public static final int CASCADE_FACTOR = 2; // TODO: Get This
        public static final Distance PITCH_DIAMETER = Meters.of(0.0381); // TODO: Get This
        public static final int GEARBOX_REDUCTION = 9;

        public static final double MOTOR_ROTATIONS_PER_METER = (
            GEARBOX_REDUCTION / (CASCADE_FACTOR * Math.PI * PITCH_DIAMETER.in(Meters)) 
        ); // someone check this math please

        public static final class MaxKinematicDerivitives {
            public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(25);
            public static final AngularAcceleration MAX_ACCERLATION = RotationsPerSecondPerSecond.of(10);
        }

    }
}


