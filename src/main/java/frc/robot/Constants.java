package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

public final class Constants {
    public static final class ControllerConstants {
        public static final double kInputClipping = 1; // Trimming joystick input by a percent; set to 1 for 100% of
        public static final double kTriggerThreshold = 0.2; //                                         joystick range

        public static final double stickDeadband = 0.05;
        public static final int leftXAxis = 0;
        public static final int leftYAxis = 1;
        public static final int rightXAxis = 3;
        public static final int rightYAxis = 4;

        public static final boolean invertYAxis = false;
        public static final boolean invertRAxis = false;
        public static final boolean invertXAxis = false;

        // Mambo controller doesn't need any of the calibrations below
        public static final boolean isMambo = true;

        // if not Mambo there are 2 controllers with the same mechanics,
        // but different calibrations
        public static final boolean isC1 = true;

        // Controller 1 left side:
        public static final double C1LeftThrottleZero = -0.125;
        public static final double C1LeftYawZero = 0.039370;

        public static final double C1LeftThrottleHigh = 0.787402;
        public static final double C1LeftThrottleLow = 0.968750;

        public static final double C1LeftYawHigh = 0.86612;
        public static final double C1LeftYawLow = 0.77338;

        // Controller 1 right side:
        public static final double C1RightThrottleZero = 0.055118;
        public static final double C1RightYawZero = 0.055118;

        public static final double C1RightYawHigh = 0.866142;
        public static final double C1RightYawLow = 0.765625;

        public static final double C1RightThrottleHigh = 0.732283;
        public static final double C1RightThrottleLow = 0.601563;

        // Controller 2 left side:
        public static final double C2LeftThrottleZero = -0.023438;
        public static final double C2LeftYawZero = -0.078125;

        public static final double C2LeftThrottleHigh = 0.834646;
        public static final double C2LeftThrottleLow = 0.867188;

        public static final double C2LeftYawHigh = 0.748031;
        public static final double C2LeftYawLow = 0.890625;

        // Controller 2 right side:
        public static final double C2RightThrottleZero = -0.054688; // high 0.007874
        public static final double C2RightYawZero = 0.062992;

        public static final double C2RightYawHigh = 0.866142;
        public static final double C2RightYawLow = 0.664063;

        public static final double C2RightThrottleHigh = 0.669291;
        public static final double C2RightThrottleLow = 0.664063;

        // Controller left side:
        public static final double LeftThrottleZero = isC1 ? C1LeftThrottleZero : C2LeftThrottleZero;
        public static final double LeftYawZero = isC1 ? C1LeftYawZero : C2LeftYawZero;

        public static final double LeftThrottleHigh = isC1 ? C1LeftThrottleHigh : C2LeftThrottleHigh;
        public static final double LeftThrottleLow = isC1 ? C1LeftThrottleLow : C2LeftThrottleLow;

        public static final double LeftYawHigh = isC1 ? C1LeftYawHigh : C2LeftYawHigh;
        public static final double LeftYawLow = isC1 ? C1LeftYawLow : C2LeftYawLow;

        // Controller right side:
        public static final double RightThrottleZero = isC1 ? C1RightThrottleZero : C2RightThrottleZero;
        public static final double RightYawZero = isC1 ? C1RightYawZero : C2RightYawZero;

        public static final double RightYawHigh = isC1 ? C1RightYawHigh : C2RightYawHigh;
        public static final double RightYawLow = isC1 ? C1RightYawLow : C2RightYawLow;

        public static final double RightThrottleHigh = isC1 ? C1RightThrottleHigh : C2RightThrottleHigh;
        public static final double RightThrottleLow = isC1 ? C1RightThrottleLow : C2RightThrottleLow;
    }

    public static final class Ports {
        public static final String CANBUS_DRIVE = "Big Justice";
        public static final String CANBUS_OPS = "Baby Gronk";

        public static final int DRIVER_CONTROL = 0;
        public static final int OPERATOR_CONTROL = 1;

        public static final int PIGEON = 13;
        public static final int CANRANGE = 0;
        public static final int LAUNCHER = 14;

        public static final int FL_DRIVE = 5;
        public static final int FL_ROTATION = 8;
        public static final int FL_CANCODER = 1;

        public static final int FR_DRIVE = 7;
        public static final int FR_ROTATION = 10;
        public static final int FR_CANCODER = 3;

        public static final int BL_DRIVE = 9;
        public static final int BL_ROTATION = 12;
        public static final int BL_CANCODER = 2;

        public static final int BR_DRIVE = 6;
        public static final int BR_ROTATION = 11;
        public static final int BR_CANCODER = 4;
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

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus(Ports.CANBUS_DRIVE, "./logs/example.hoot");

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
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(Ports.PIGEON)
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
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;
        private static final Distance kFrontLeftXPos = Inches.of(12);
        private static final Distance kFrontLeftYPos = Inches.of(12);

        // Front Right
        private static final Angle kFrontRightEncoderOffset = Rotations.of(0);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;
        private static final Distance kFrontRightXPos = Inches.of(12);
        private static final Distance kFrontRightYPos = Inches.of(-12);

        // Back Left
        private static final Angle kBackLeftEncoderOffset = Rotations.of(0);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;
        private static final Distance kBackLeftXPos = Inches.of(-12);
        private static final Distance kBackLeftYPos = Inches.of(12);

        // Back Right
        private static final Angle kBackRightEncoderOffset = Rotations.of(0);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;
        private static final Distance kBackRightXPos = Inches.of(-12);
        private static final Distance kBackRightYPos = Inches.of(-12);

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                Ports.FL_ROTATION, Ports.FL_DRIVE, Ports.FL_CANCODER, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                Ports.FR_ROTATION, Ports.FR_DRIVE, Ports.FR_CANCODER, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                Ports.BL_ROTATION, Ports.BL_DRIVE, Ports.BL_CANCODER, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreator.createModuleConstants(
                Ports.BR_ROTATION, Ports.BR_DRIVE, Ports.BR_CANCODER, kBackRightEncoderOffset,
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

    public static final class LauncherConstants {
        public static final TalonFX motor = new TalonFX(Ports.LAUNCHER, new CANBus(Ports.CANBUS_DRIVE));
        public static final VelocityVoltage velocityVoltage = new VelocityVoltage(30).withSlot(0);
        public static final StaticBrake brake = new StaticBrake();
        public static final TalonFXConfiguration configs = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(0.1).withKV(0.12).withKP(0.11))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8)))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40)));
        {   // Retry config apply up to 3 times, report if failure:
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 3; ++i) {
                status = motor.getConfigurator().apply(configs);
                if (status.isOK()) break;
            }
            if (!status.isOK()) System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }
}