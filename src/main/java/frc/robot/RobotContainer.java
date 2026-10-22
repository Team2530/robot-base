package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.MetaConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.control.DriveCommand;
import frc.robot.commands.util.MatchtimeStatusCommand;
import frc.robot.commands.util.VoltageStatusCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Elastic;
import frc.robot.subsystems.util.PDHSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {

    // @Logged
    public static final CommandXboxController driverXbox =
        new CommandXboxController(MetaConstants.Controllers.DRIVER_PORT);
    // @Logged
    public static final CommandXboxController operatorXbox =
        new CommandXboxController(MetaConstants.Controllers.OPERATOR_PORT);

    public static final SendableChooser<Command> autoChooser =
        new SendableChooser<>();

    @Logged
    public static final SwerveSubsystem swerveDriveSubsystem =
        new SwerveSubsystem();

    @Logged
    public static final PDHSubsystem pdhSubsystem = new PDHSubsystem();

    @Logged
    public static final DriveCommand normalDrive = new DriveCommand(
            swerveDriveSubsystem,
            driverXbox.getHID()
        );
    public static final LimelightSubsystem limelightSubsystem =
         new LimelightSubsystem(
        /*
            // These are initating the individual Limlight(s). The name should
            // match the limelight internal names.
            new Limelight(
                "limelight-fl",
                new Pose3d(
                    new Translation3d(
                        Meters.of(-0.31513),
                        Meters.of(-0.25669),
                        Meters.of(0.261315)
                    ),
                    new Rotation3d(
                        Degrees.of(-1.6),
                        Degrees.of(-14.8),
                        Degrees.of(55)
                    )
                )
            ),

        ... so on and so forth
        */
        );

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    static {
        // Configure the trigger bindings
        configureBindings();

        configureAutos();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveDriveSubsystem.setDefaultCommand(normalDrive);
        //turretSubsystem.setDefaultCommand(new TurretCommand(turretSubsystem));

        // i'm running out of names
        configureCommands();
    }

    /*
     * This method schedules and configures all miscellaneous commands / actions
     */
    private static void configureCommands() {
        SmartDashboard.putString(
            "Meta/Match_Type",
            DriverStation.getMatchType().toString()
        );

        RobotModeTriggers.autonomous()
            .onTrue(
                new ParallelCommandGroup(
                    new InstantCommand(() -> {
                        Elastic.selectTab("Autonomous");
                    }),

                    new VoltageStatusCommand(),
                    new MatchtimeStatusCommand()
                )
            );

        RobotModeTriggers.teleop()
            .onTrue(
                new ParallelCommandGroup(
                    new InstantCommand(() -> {
                        Elastic.selectTab("Teleoperated");
                    })
                )
            );

        RobotModeTriggers.disabled()
            .onTrue(
                new InstantCommand(() -> {
                    limelightSubsystem.setIMUModes(1);
                })
            );
    }
    


    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
     * with an arbitrary predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s
     * subclasses for {@link CommandXboxController Xbox}
     * / {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private static void configureBindings() {
        driverXbox.start()
            .onTrue(
                new InstantCommand(() -> {
                    swerveDriveSubsystem.snapToVision();
                    normalDrive.resetHeading();
                })
            );
        driverXbox.leftBumper()
            .whileTrue(
                new Command() {
                    @Override
                    public void execute() {
                        swerveDriveSubsystem.snapToVision();
                    }

                    @Override
                    public boolean isFinished() {
                        return false;
                    }
                }
            );

        operatorXbox.rightStick()
            .whileTrue(
                    new Command() {
                        @Override
                        public void execute() {
                            
                            swerveDriveSubsystem.snapToVision();
                        }                        

                        @Override
                        public boolean isFinished() {
                            return false;
                        }
                    }
            );
    }


    private static class Flagpole {
        private ArrayList<Command> flags = new ArrayList<>();

        public InstantCommand raiseFlaggedCommand(Supplier<Command> supplier) {
            return new InstantCommand(() -> {
                flags.add(
                    supplier.get() 
                );
            });
        }
        
        
        public Command catchFlags() {
            Command command = new ParallelRaceGroup(
                    flags.toArray(
                        new Command[flags.size()]
                    )
                );

            flags.clear();

            return command;
        }
    }

    private static void configureAutos() {
        AutoBuilder.configure(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::resetOdometry,
                swerveDriveSubsystem::getRobotRelativeVelocity,
                swerveDriveSubsystem::driveRobotRelative,
                new SwerveSubsystem.AutonomousController(),
                new RobotConfig(
                    RobotConstants.TOTAL_MASS,
                    RobotConstants.MOMENT_OF_INERTIA,
                    new ModuleConfig(
                        DriveConstants.Modules.WHEEL_DIAMETER.div(2),
                        DriveConstants.MAX_ROBOT_VELOCITY,
                        DriveConstants.Modules.WHEEL_FRICTION_COEFFICIENT,
                        DCMotor.getKrakenX60Foc(1),
                        swerveDriveSubsystem.getDriveGearRatio(),
                        DriveConstants.Modules.DRIVE_CURRENT_LIMIT,
                        1
                    ),
                    new Translation2d(
                        DriveConstants.Modules.Offsets.FL.X,
                        DriveConstants.Modules.Offsets.FL.Y
                    ),
                    new Translation2d(
                        DriveConstants.Modules.Offsets.FR.X,
                        DriveConstants.Modules.Offsets.FR.Y
                    ),
                    new Translation2d(
                        DriveConstants.Modules.Offsets.BL.X,
                        DriveConstants.Modules.Offsets.BL.Y
                    ),
                    new Translation2d(
                        DriveConstants.Modules.Offsets.BR.X,
                        DriveConstants.Modules.Offsets.BR.Y
                    )
                ),
                MetaConstants.isRed,
                swerveDriveSubsystem
            );

        // add named commands for the paths
        Flagpole flagpole = new Flagpole();
        Map<String, Command> namedCommands = new HashMap<>() {{
            put(
                "xStance",
                new InstantCommand(() -> {
                    swerveDriveSubsystem.xStance();
                })
            );
            put(
                "Raise Wait",
                flagpole.raiseFlaggedCommand(
                    () -> new WaitCommand(Seconds.of(4))
                )
            );
        }};
        for (Entry<String, Command> pair : namedCommands.entrySet()) {
            NamedCommands.registerCommand(
                pair.getKey(),
                pair.getValue()
            );

            SmartDashboard.putBoolean(
                "Auto/Event Bindings/" + pair.getKey() + "/available",
                true
            );
        }

        autoChooser.onChange(new Consumer<Command>() {
            @Override
            public void accept(Command command) {
                if (command instanceof PathPlannerAuto) {
                    swerveDriveSubsystem.resetOdometry(
                        ((PathPlannerAuto) command).getStartingPose()
                    );
                }
            }
        });

        // WARNING: as specified by the [pathplanner documentation]
        // (https://pathplanner.dev/pplib-named-commands.html),
        // named commands must be registerd before paths are created / loaded
        //
        // load paths
        for (String trajectoryName : Choreo.availableTrajectories()) {
            try {
                SequentialCommandGroup routine = new SequentialCommandGroup();

                // find event markers that are placed at a trajectory split,
                // remove them from the path,
                Trajectory choreoTrajectory =
                    Choreo.loadTrajectory(trajectoryName).get();
                List<Integer> splitIndices = choreoTrajectory.splits();
                for (int i = 0; i < splitIndices.size(); i++) {
                    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(
                            trajectoryName,
                            i
                        );
                    
                    if (i == 0) {
                        routine.addCommands(
                            new InstantCommand(() -> {
                                swerveDriveSubsystem.resetOdometry(
                                    AllianceFlipUtil.apply(
                                        path.getStartingHolonomicPose().get()
                                    )
                                );
                            })
                        );
                    }

                    routine.addCommands(
                        AutoBuilder.followPath(path),
                        new DeferredCommand(
                            flagpole::catchFlags,
                            new HashSet<>() 
                        )
                    );
                }

                autoChooser.addOption(
                        trajectoryName,
                        routine
                );
            } catch (Exception e) {
                System.out.print(
                    "Caught exception during autochooser configuration: "
                    + e
                );
            }

            // extra option
            autoChooser.addOption(
                "Do nothing",
                null
            );
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveDriveSubsystem;
    }

    public CommandXboxController getDriverXbox() {
        return driverXbox;
    }

    public CommandXboxController getOperatorXbox() {
        return operatorXbox;
    }
}
