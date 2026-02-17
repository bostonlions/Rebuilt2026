package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Robot;

public final class Drive implements Subsystem {
    public static final class Drivetrain extends SwerveConstants.TunerSwerveDrivetrain implements Subsystem {
        private static final double kSimLoopPeriod = 0.004; // 4 ms
        private Notifier m_simNotifier = null;
        private double m_lastSimTime;

        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
        /* Keep track if we've ever applied the operator perspective before or not */
        private boolean m_hasAppliedOperatorPerspective = false;

        /* Swerve requests to apply during SysId characterization */
        private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
        // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
        // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

        public static Drivetrain getInstance() {
            return new Drivetrain(
                SwerveConstants.DrivetrainConstants, SwerveConstants.FrontLeft,
                SwerveConstants.FrontRight, SwerveConstants.BackLeft, SwerveConstants.BackRight
            );
        }

        /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
        private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                null,        // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> setControl(m_translationCharacterization.withVolts(output)),
                null,
                this
            )
        );

        /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
        // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        //     new SysIdRoutine.Config(
        //         null,        // Use default ramp rate (1 V/s)
        //         Volts.of(7), // Use dynamic voltage of 7 V
        //         null,        // Use default timeout (10 s)
        //         // Log state with SignalLogger class
        //         state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        //     ),
        //     new SysIdRoutine.Mechanism(
        //         volts -> setControl(m_steerCharacterization.withVolts(volts)),
        //         null,
        //         this
        //     )
        // );

        /*
        * SysId routine for characterizing rotation.
        * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
        */
        // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        //     new SysIdRoutine.Config(
        //         /* This is in radians per second², but SysId only supports "volts per second" */
        //         Volts.of(Math.PI / 6).per(Second),
        //         /* This is in radians per second, but SysId only supports "volts" */
        //         Volts.of(Math.PI),
        //         null, // Use default timeout (10 s)
        //         // Log state with SignalLogger class
        //         state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        //     ),
        //     new SysIdRoutine.Mechanism(
        //         output -> {
        //             /* output is actually radians per second, but SysId only supports "volts" */
        //             setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
        //             /* also log the requested output for SysId */
        //             SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
        //         },
        //         null,
        //         this
        //     )
        // );

        /* The SysId routine to test */
        private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        private Drivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(drivetrainConstants, modules);
            if (Utils.isSimulation()) {
                startSimThread();
            }
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        private Drivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(drivetrainConstants, odometryUpdateFrequency, modules);
            if (Utils.isSimulation()) {
                startSimThread();
            }
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        private Drivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
            if (Utils.isSimulation()) {
                startSimThread();
            }
        }

        /**
         * Returns a command that applies the specified control request to this swerve drivetrain.
         *
         * @param request Function returning the request to apply
         * @return Command to run
         */
        public Command applyRequest(Supplier<SwerveRequest> request) {
            return run(() -> this.setControl(request.get()));
        }

        /**
         * Runs the SysId Quasistatic test in the given direction for the routine
         * specified by {@link #m_sysIdRoutineToApply}.
         *
         * @param direction Direction of the SysId Quasistatic test
         * @return Command to run
         */
        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
            return m_sysIdRoutineToApply.quasistatic(direction);
        }

        /**
         * Runs the SysId Dynamic test in the given direction for the routine
         * specified by {@link #m_sysIdRoutineToApply}.
         *
         * @param direction Direction of the SysId Dynamic test
         * @return Command to run
         */
        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return m_sysIdRoutineToApply.dynamic(direction);
        }

        @Override
        public void periodic() {
            /*
            * Periodically try to apply the operator perspective.
            * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
            * This allows us to correct the perspective in case the robot code restarts mid-match.
            * Otherwise, only check and apply the operator perspective if the DS is disabled.
            * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
            */
            if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
                DriverStation.getAlliance().ifPresent(allianceColor -> {
                    setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                            ? kRedAlliancePerspectiveRotation
                            : kBlueAlliancePerspectiveRotation
                    );
                    m_hasAppliedOperatorPerspective = true;
                });
            }
        }

        private void startSimThread() {
            m_lastSimTime = Utils.getCurrentTimeSeconds();

            /* Run simulation at a faster rate so PID gains behave more reasonably */
            m_simNotifier = new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - m_lastSimTime;
                m_lastSimTime = currentTime;

                /* use the measured time delta, get battery voltage from WPILib */
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
            m_simNotifier.startPeriodic(kSimLoopPeriod);
        }

        /**
         * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
         * while still accounting for measurement noise.
         *
         * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
         * @param timestampSeconds The timestamp of the vision measurement in seconds.
         */
        @Override
        public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
            super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
        }

        /**
         * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
         * while still accounting for measurement noise.
         * <p>
         * Note that the vision measurement standard deviations passed into this method
         * will continue to apply to future measurements until a subsequent call to
         * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
         *
         * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
         * @param timestampSeconds The timestamp of the vision measurement in seconds.
         * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
         *     in the form [x, y, theta]ᵀ, with units in meters and radians.
         */
        @Override
        public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
        ) {
            super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
        }

        /**
         * Return the pose at a given timestamp, if the buffer is not empty.
         *
         * @param timestampSeconds The timestamp of the pose in seconds.
         * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
         */
        @Override
        public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
            return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
        }
    }

    public static final class SwerveConstants {
        public static final SwerveRequest idle = new SwerveRequest.Idle();

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(2.66).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(120);

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

        public static final double MaxAngularRate = RotationsPerSecond.of(0.3333).in(RadiansPerSecond);
        // Theoretical free speed (m/s) at 12 V applied output; this needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.04);

        private static final double kCoupleRatio = 3.145;
        private static final double kDriveGearRatio = 5.36;
        private static final double kSteerGearRatio = 18.75;

        private static final Distance kWheelRadius = Inches.of(2);

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(Robot.kCANBus.getName())
                .withPigeon2Id(Robot.Ports.PIGEON)
                .withPigeon2Configs(new Pigeon2Configuration());

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

        // Front Left
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(1);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;
        private static final Distance kFrontLeftXPos = Inches.of(12);
        private static final Distance kFrontLeftYPos = Inches.of(12);

        // Front Right
        private static final Angle kFrontRightEncoderOffset = Rotations.of(1);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;
        private static final Distance kFrontRightXPos = Inches.of(12);
        private static final Distance kFrontRightYPos = Inches.of(-12);

        // Back Left
        private static final Angle kBackLeftEncoderOffset = Rotations.of(1);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;
        private static final Distance kBackLeftXPos = Inches.of(-12);
        private static final Distance kBackLeftYPos = Inches.of(12);

        // Back Right
        private static final Angle kBackRightEncoderOffset = Rotations.of(1);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;
        private static final Distance kBackRightXPos = Inches.of(-12);
        private static final Distance kBackRightYPos = Inches.of(-12);

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                Robot.Ports.FL_ROTATION, Robot.Ports.FL_DRIVE, Robot.Ports.FL_CANCODER, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                Robot.Ports.FR_ROTATION, Robot.Ports.FR_DRIVE, Robot.Ports.FR_CANCODER, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                Robot.Ports.BL_ROTATION, Robot.Ports.BL_DRIVE, Robot.Ports.BL_CANCODER, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreator.createModuleConstants(
                Robot.Ports.BR_ROTATION, Robot.Ports.BR_DRIVE, Robot.Ports.BR_CANCODER, kBackRightEncoderOffset,
                kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
            );

        /**
         * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
         */
        public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
             * @param modules               Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, modules
                );
            }

            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency The frequency to run the odometry loop. If
             *                                unspecified or set to 0 Hz, this is 250 Hz on
             *                                CAN FD, and 100 Hz on CAN 2.0.
             * @param modules                 Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules
                );
            }

            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
             *                                  unspecified or set to 0 Hz, this is 250 Hz on
             *                                  CAN FD, and 100 Hz on CAN 2.0.
             * @param odometryStandardDeviation The standard deviation for odometry calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param visionStandardDeviation   The standard deviation for vision calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param modules                   Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency,
                    odometryStandardDeviation, visionStandardDeviation, modules
                );
            }
        }
    }
}
