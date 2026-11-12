package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.util.constants.ManipulatorConstants.ElevatorConstants;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkFlex m_PortMotor;
    private final SparkFlex m_StarboardMotor;
    private final SparkClosedLoopController elevatorController; 

    private Angle targetPosition;

    public ElevatorSubsystem() {
        m_PortMotor = new SparkFlex(
            ElevatorConstants.PortMotor.CAN_ID,
            MotorType.kBrushless
        );

        m_StarboardMotor = new SparkFlex(
            ElevatorConstants.StarboardMotor.CAN_ID,
            MotorType.kBrushless
        );

        elevatorController = m_PortMotor.getClosedLoopController();

        SparkFlexConfig portConfig = new SparkFlexConfig();
        portConfig
            .closedLoop
                .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
            .feedForward
                .svag(ElevatorConstants.kS, ElevatorConstants.kV, ElevatorConstants.kA, ElevatorConstants.kG);
        portConfig
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake);

        SparkFlexConfig starboardConfig = new SparkFlexConfig();
        starboardConfig
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .follow(m_PortMotor, ElevatorConstants.StarboardMotor.INVERTED);
            
        m_PortMotor.configure(
            portConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        m_StarboardMotor.configure(
            starboardConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );


    }

    @Override
    public void periodic() {
        // TODO: Rotaiton to Height Math
        Angle setPosition;
        setPosition = targetPosition;

        elevatorController.setSetpoint(setPosition.in(Rotations), ControlType.kPosition);

        // TODO: Log more stuff
        SmartDashboard.putNumber(
            "Vortex/targetVelocity",
            setPosition.in(Rotations)
        );
        SmartDashboard.putNumber(
            "Vortex/velocity",
            m_PortMotor.getEncoder().getVelocity()
        );
        SmartDashboard.putNumber(
            "Vortex/position",
            m_PortMotor.getEncoder().getPosition()
        );

    }


    public void setTargetPosition(double position) {
        targetPosition = Rotations.of(position);
    }

}