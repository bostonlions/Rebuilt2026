// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.Telemetry;
import frc.robot.Robot.Ports;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.subsystems.Drive.SwerveConstants;

public final class RobotContainer {
    private final CANrange canRange = new CANrange(Robot.Ports.CANRANGE, Robot.kCANBus);
    private final Telemetry logger = new Telemetry(SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond));
    private final ControlBoard controller = ControlBoard.getInstance();
    public final Drivetrain drivetrain = Drivetrain.getInstance();

    public RobotContainer() {
        canRange.getConfigurator().apply(new CANrangeConfiguration());
        Robot.pigeon.getConfigurator().apply(new Pigeon2Configuration());
        DriverStation.getAlliance().ifPresentOrElse(color -> Robot.pigeon.setYaw(color == Alliance.Blue ? 0 : 180), () -> {
            throw new IllegalArgumentException("Is this code happening too early and the alliance color isn't available yet?");
        });

        drivetrain.setDefaultCommand( // X is defined as forward according to WPILib convention, and Y is defined as to the left
            drivetrain.applyRequest(() -> // ...but our controller convention is y as forward, and x as to the right
                SwerveConstants.drive.withVelocityX(controller.getSwerveTranslation().getY())
                    .withVelocityY(-controller.getSwerveTranslation().getX())
                    .withRotationalRate(-controller.getSwerveRotation())
            )
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> SwerveConstants.idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.B)).onTrue(new InstantCommand(
            () -> System.out.println("CANrange distance: " + canRange.getDistance().getValueAsDouble())
        ));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton for an example:
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                SwerveConstants.drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> SwerveConstants.idle)
        );
    }

    private static final class ControlBoard {
        private static ControlBoard mInstance = null;
        private final CustomXboxController operator;
        private final GenericHID driver;
        private final double speedFactor;
        private final double kSwerveDeadband;

        private static ControlBoard getInstance() {
            if (mInstance == null) mInstance = new ControlBoard();
            return mInstance;
        }

        private ControlBoard() {
            driver = new GenericHID(Ports.DRIVER_CONTROL);
            operator = new CustomXboxController(Ports.OPERATOR_CONTROL);
            speedFactor = ControllerConstants.kInputClipping;
            kSwerveDeadband = ControllerConstants.stickDeadband;
        }

        private Translation2d getSwerveTranslation() {
            double forwardAxis = 0;
            double strafeAxis = 0;

            if (ControllerConstants.isMambo) {
                forwardAxis = driver.getRawAxis(2);
                strafeAxis = driver.getRawAxis(1);
                double mag = Math.pow(forwardAxis * forwardAxis + strafeAxis * strafeAxis, 0.5);
                double curveFactor = Math.pow(mag, 0.25);
                forwardAxis = forwardAxis * curveFactor;
                strafeAxis = strafeAxis * curveFactor;
            } else {
                forwardAxis = getRightThrottle();
                strafeAxis = getRightYaw();
            }

            forwardAxis *= speedFactor;
            strafeAxis *= speedFactor;

            SmartDashboard.putNumber("Raw Y", forwardAxis);
            SmartDashboard.putNumber("Raw X", strafeAxis);

            forwardAxis = ControllerConstants.invertYAxis ? forwardAxis : -forwardAxis;
            strafeAxis = ControllerConstants.invertXAxis ? strafeAxis : -strafeAxis;

            Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

            if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) return new Translation2d(); else {
                Rotation2d deadband_direction = new Rotation2d(tAxes.getX(), tAxes.getY());
                Translation2d deadband_vector = new Translation2d(kSwerveDeadband, deadband_direction);

                double scaled_x = MathUtil.applyDeadband(forwardAxis, Math.abs(deadband_vector.getX()));
                double scaled_y = MathUtil.applyDeadband(strafeAxis, Math.abs(deadband_vector.getY()));
                return new Translation2d(scaled_x, scaled_y).times(Drive.SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond) / 7);
            }
        }

        private double getSwerveRotation() {
            double rotAxis = ControllerConstants.isMambo ? driver.getRawAxis(3) : getLeftYaw();
            rotAxis = ControllerConstants.invertRAxis ? rotAxis : -rotAxis;
            rotAxis *= speedFactor;

            if (Math.abs(rotAxis) < kSwerveDeadband) return 0.0;
            return Drive.SwerveConstants.MaxAngularRate *
                (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
        }

        /** Non-mambo controller. Returns positions from -1 to 1 */
        private double getLeftYaw() {
            double leftYaw = driver.getRawAxis(ControllerConstants.leftXAxis);

            if (leftYaw != 0) leftYaw -= ControllerConstants.LeftYawZero;

            if (leftYaw > kSwerveDeadband) leftYaw /= (ControllerConstants.LeftYawHigh +
                (ControllerConstants.isC1 ? -ControllerConstants.LeftYawZero : ControllerConstants.LeftYawZero));
            else if (leftYaw < -kSwerveDeadband) leftYaw /= (ControllerConstants.LeftYawLow +
                ControllerConstants.LeftYawZero);
            return MathUtil.clamp(leftYaw, -1, 1);
        }

        /** Non-mambo controller. Returns positions from -1 to 1 */
        private double getRightThrottle() {
            double rightThrottle = driver.getRawAxis(ControllerConstants.rightYAxis);

            if (rightThrottle != 0) rightThrottle = rightThrottle - ControllerConstants.RightThrottleZero;

            if (rightThrottle > (ControllerConstants.isC1 ? kSwerveDeadband : 0.102))
                rightThrottle /= (ControllerConstants.RightThrottleHigh + (ControllerConstants.isC1 ?
                    -ControllerConstants.RightThrottleZero : ControllerConstants.RightThrottleZero));
            else if (rightThrottle < -kSwerveDeadband) rightThrottle /= (ControllerConstants.RightThrottleLow
                + ControllerConstants.RightThrottleZero);
            return MathUtil.clamp(rightThrottle, -1, 1);
        }

        /** Non-mambo controller. Returns positions from -1 to 1 */
        private double getRightYaw() {
            double rightYaw = driver.getRawAxis(ControllerConstants.rightXAxis);

            if (rightYaw != 0) rightYaw -= ControllerConstants.RightYawZero;

            if (rightYaw > kSwerveDeadband) rightYaw /= (ControllerConstants.RightYawHigh -
                ControllerConstants.RightYawZero);
            else if (rightYaw < -kSwerveDeadband) rightYaw /= (ControllerConstants.RightYawLow +
                ControllerConstants.RightYawZero);
            return MathUtil.clamp(rightYaw, -1, 1);
        }

        private static final class CustomXboxController {
            private final XboxController mController;
            // public enum Side { LEFT, RIGHT }
            // public enum Axis { X, Y }
            private enum Button {
                A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);
                public final int id;
                Button(int id) { this.id = id; }
            }

            private CustomXboxController(int port) {
                mController = new XboxController(port);
            }

            // public double getAxis(Side side, Axis axis) {
            //     boolean left = side == Side.LEFT;
            //     boolean y = axis == Axis.Y;
            //     return mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0));
            // }

            // public boolean getTrigger(Side side) {
            //     return mController.getRawAxis(side == Side.LEFT ? 2 : 3) > ControllerConstants.kTriggerThreshold;
            // }

            private boolean getButton(Button button) {
                return mController.getRawButton(button.id);
            }

            // public XboxController getController() {
            //     return mController;
            // }
        }

        private static final class ControllerConstants {
            private static final double kInputClipping = 1; // Trimming joystick input by a percent; set to 1 for 100% of
            // public static final double kTriggerThreshold = 0.2; //                                         joystick range

            private static final double stickDeadband = 0.05;
            private static final int leftXAxis = 0;
            // public static final int leftYAxis = 1;
            private static final int rightXAxis = 3;
            private static final int rightYAxis = 4;

            private static final boolean invertYAxis = false;
            private static final boolean invertRAxis = false;
            private static final boolean invertXAxis = false;

            // Mambo controller doesn't need any of the calibrations below
            private static final boolean isMambo = true;

            // if not Mambo there are 2 controllers with the same mechanics,
            // but different calibrations
            private static final boolean isC1 = true;

            // Controller 1 left side:
            // public static final double C1LeftThrottleZero = -0.125;
            private static final double C1LeftYawZero = 0.039370;

            // public static final double C1LeftThrottleHigh = 0.787402;
            // public static final double C1LeftThrottleLow = 0.968750;

            private static final double C1LeftYawHigh = 0.86612;
            private static final double C1LeftYawLow = 0.77338;

            // Controller 1 right side:
            private static final double C1RightThrottleZero = 0.055118;
            private static final double C1RightYawZero = 0.055118;

            private static final double C1RightYawHigh = 0.866142;
            private static final double C1RightYawLow = 0.765625;

            private static final double C1RightThrottleHigh = 0.732283;
            private static final double C1RightThrottleLow = 0.601563;

            // Controller 2 left side:
            // public static final double C2LeftThrottleZero = -0.023438;
            private static final double C2LeftYawZero = -0.078125;

            // public static final double C2LeftThrottleHigh = 0.834646;
            // public static final double C2LeftThrottleLow = 0.867188;

            private static final double C2LeftYawHigh = 0.748031;
            private static final double C2LeftYawLow = 0.890625;

            // Controller 2 right side:
            private static final double C2RightThrottleZero = -0.054688; // high 0.007874
            private static final double C2RightYawZero = 0.062992;

            private static final double C2RightYawHigh = 0.866142;
            private static final double C2RightYawLow = 0.664063;

            private static final double C2RightThrottleHigh = 0.669291;
            private static final double C2RightThrottleLow = 0.664063;

            // Controller left side:
            // public static final double LeftThrottleZero = isC1 ? C1LeftThrottleZero : C2LeftThrottleZero;
            private static final double LeftYawZero = isC1 ? C1LeftYawZero : C2LeftYawZero;

            // public static final double LeftThrottleHigh = isC1 ? C1LeftThrottleHigh : C2LeftThrottleHigh;
            // public static final double LeftThrottleLow = isC1 ? C1LeftThrottleLow : C2LeftThrottleLow;

            private static final double LeftYawHigh = isC1 ? C1LeftYawHigh : C2LeftYawHigh;
            private static final double LeftYawLow = isC1 ? C1LeftYawLow : C2LeftYawLow;

            // Controller right side:
            private static final double RightThrottleZero = isC1 ? C1RightThrottleZero : C2RightThrottleZero;
            private static final double RightYawZero = isC1 ? C1RightYawZero : C2RightYawZero;

            private static final double RightYawHigh = isC1 ? C1RightYawHigh : C2RightYawHigh;
            private static final double RightYawLow = isC1 ? C1RightYawLow : C2RightYawLow;

            private static final double RightThrottleHigh = isC1 ? C1RightThrottleHigh : C2RightThrottleHigh;
            private static final double RightThrottleLow = isC1 ? C1RightThrottleLow : C2RightThrottleLow;
        }
    }
}
