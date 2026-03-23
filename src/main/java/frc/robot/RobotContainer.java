// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.Telemetry;
import frc.robot.Robot.Ports;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.subsystems.Drive.SwerveConstants;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.Trimmer;

public final class RobotContainer {
    private final Telemetry logger = new Telemetry(SwerveConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond));
    public static final ControlBoard controller = ControlBoard.getInstance();
    public final Drivetrain drivetrain = Drivetrain.getInstance();
    public final Climber climber;
    private final Intake intake;
    private final Launcher launcher;
    private final Trimmer trimmer = Trimmer.getInstance();
    private final AutoFactory autoFactory;
    

    // // Test toggles
    // private boolean launcherTestEnabled = false;
    // private boolean feederRollerTestEnabled = false;

    private void zeroGyro() {
        drivetrain.resetRotation(new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI : 0));
    }

    public RobotContainer() {
        Robot.pigeon.getConfigurator().apply(new com.ctre.phoenix6.configs.Pigeon2Configuration());
        zeroGyro();
        drivetrain.setDefaultCommand( // X is defined as forward according to WPILib convention, and Y is defined as to the left
            drivetrain.applyRequest(() -> // origin is right corner of blue alliance driver station
                SwerveConstants.drive.withVelocityX(controller.getSwerveTranslation().getX())
                    .withVelocityY(controller.getSwerveTranslation().getY())
                    .withRotationalRate(controller.getSwerveRotation())
            )
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> SwerveConstants.idle).ignoringDisable(true)
        );

        new Trigger(() -> controller.driver.getRawButton(2))
            .onTrue(new InstantCommand(() -> zeroGyro()).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);

        intake = Intake.getInstance();
        SmartDashboard.putData(intake);
        // TODO: ensure climber is stowed before we extend the intake
        new Trigger(() -> controller.operator.getTrigger(ControlBoard.CustomXboxController.Side.RIGHT))
            .onTrue(new InstantCommand(() -> intake.setExtension(true), intake));
        new Trigger(() -> controller.operator.getTrigger(ControlBoard.CustomXboxController.Side.LEFT))
            .onTrue(new InstantCommand(() -> intake.setExtension(false), intake));
        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.LB))
            .onTrue(new InstantCommand(() -> intake.toggleSpin(), intake));
        

        launcher = Launcher.getInstance();
        SmartDashboard.putData(launcher);
        // --- SHOOTING ---
        // Controlls:
        // X: Simple toggle shooter / Shoot
        // RB: Simple Hurl / Spin Up
        // To shoot, hold down RB and press X when ready to start shooting

        // Simple shooting mode: X toggles everything on/off.
        // RPM target is adjustable at runtime via Trimmer ("Simple Launch RPM +- 100").
        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.X))
            .onTrue(new InstantCommand(() -> launcher.simpleToggle()));
        // Hurling mode: Right Bumper toggles everything on/off.
        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.RB))
            .onTrue(new InstantCommand(() -> launcher.simpleToggle(370/*0*/, 31, Double.NaN))); // Hurl shot
        
        // UNCOMMENT WHEN READY FOR SHOOTNIG WITH POSE2D
        // 1. Spin up / Aim (STANDBY)
        // new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.RB))
        //     .onTrue(new InstantCommand(() -> launcher.setMode(Launcher.Mode.STANDBY)))
        //     .onFalse(new InstantCommand(() -> launcher.setMode(Launcher.Mode.OFF)));

        // // 2. Fire (Requires right bumper to be held, or you can make it standalone)
        // new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.X))
        //     .onTrue(new InstantCommand(() -> launcher.setMode(Launcher.Mode.FIRE)))
        //     .onFalse(new InstantCommand(() -> {
        //         // When you let go of the fire button, drop back to STANDBY if the spin-up 
        //         // button is still held, otherwise turn OFF.
        //         if (controller.operator.getButton(ControlBoard.CustomXboxController.Button.RB)) {
        //             launcher.setMode(Launcher.Mode.STANDBY);
        //         } else {
        //             launcher.setMode(Launcher.Mode.OFF);
        //         }
        //     }));



        climber = Climber.getInstance();
        SmartDashboard.putData(climber);

        // // Start button re-zeros
        // new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.START))
        //     .onTrue(new InstantCommand(() -> climber.forceStow()));
        // Operator Xbox A button: lower hooks to Clinch CANCoder position
        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.A))
            .onTrue(new InstantCommand(() -> climber.setLowerHooks(Climber.Position.Clinch), climber));
        // Operator Xbox B button: lower hooks to Grab CANCoder position
        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.B))
            .onTrue(new InstantCommand(() -> climber.setLowerHooks(Climber.Position.Grab), climber));
        // Operator Xbox Y button: lower hooks to Prepare CANCoder position
        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.Y))
            .onTrue(new InstantCommand(() -> climber.setLowerHooks(Climber.Position.Prepare), climber));
        // Right stick click: prepare to climb TODO ensure intake is stowed first
        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.R_JOYSTICK))
            .onTrue(new InstantCommand(() -> climber.prepToClimb(), climber));
        // Screenshot/Share button (button 7): stow the climber
        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.BACK))
            .onTrue(new InstantCommand(() -> climber.stow(), climber));
        // Left stick click: lower hooks to Prepare
        new Trigger(() -> controller.operator.getButton(ControlBoard.CustomXboxController.Button.L_JOYSTICK))
            .onTrue(new InstantCommand(() -> climber.setLowerHooks(Climber.Position.Prepare), climber));


        autoFactory = new AutoFactory(
            drivetrain::getPose, // A function that returns the current robot pose
            drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drivetrain // The drive subsystem
        );


        
        /*
         * TRIMMER - all subsystems can add items to be adjusted.
         * These commands run in disabled mode (ignoringDisable), so you can
         * tweak parameters and choose Auto commands before the match.
         *
         * To select autonomous: D-pad LEFT/RIGHT = cycle subsystems (e.g. "Autonomous"),
         * D-pad RIGHT = cycle items within subsystem, D-pad UP/DOWN = cycle Auto routine.
         */
        SmartDashboard.putData(Auton.getInstance());
        SmartDashboard.putData(trimmer);
        new Trigger(() -> (controller.operator.getController().getPOV() == 270)).onTrue(trimmer.nextSubsystemCommand());
        new Trigger(() -> (controller.operator.getController().getPOV() == 90)).onTrue(trimmer.nextItemCommand());
        new Trigger(() -> (controller.operator.getController().getPOV() == 0)).onTrue(trimmer.incrementItemCommand());
        new Trigger(() -> (controller.operator.getController().getPOV() == 180)).onTrue(trimmer.decrementItemCommand());
    }

    public Command getAutonomousCommand() {
        return Auton.getInstance().getCommand();
    }

    

    public static final class ControlBoard {
        private static ControlBoard mInstance = null;
        public final CustomXboxController operator;
        private final GenericHID driver;
        private final double speedFactor;
        private final double kSwerveDeadband;

        /** Ethnix right switch (Axis 6): 0 = fast, >0 = 25% speed */
        private static final int kSlowDriveAxis = 6;
        private static final double kSlowDriveScale = 0.25;
        private static final double kSlowDriveThreshold = 0.05;

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

        public boolean getFeederSpinnerTestButton() {
            // Operator Xbox controller A button
            return operator.getButton(CustomXboxController.Button.A);
        }

        public boolean getFeederRollerToggleButton() {
            // Operator Xbox controller X button
            return operator.getButton(CustomXboxController.Button.X);
        }

        public boolean getLauncherToggleButton() {
            // Operator Xbox controller Y button
            return operator.getButton(CustomXboxController.Button.Y);
        }

        private Translation2d getSwerveTranslation() {
            double forwardAxis = 0;
            double strafeAxis = 0;

            if (ControllerConstants.isMambo) {
                forwardAxis = driver.getRawAxis(2);
                strafeAxis = driver.getRawAxis(1);
                double mag = Math.hypot(forwardAxis, strafeAxis);
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
                double driveScale = driver.getRawAxis(kSlowDriveAxis) > kSlowDriveThreshold ? kSlowDriveScale : 1.0;

                // Apply additional slowdown if the launcher is preparing to shoot
                if (Launcher.getInstance().isAimingOrShooting()) {
                    // Adjust this 0.3 (30% speed) value to whatever feels best
                    driveScale *= 0.3; // TODO: CHANGE to whatever speed you want while shooting
                }

                return new Translation2d(scaled_x, scaled_y).times(SwerveConstants.kSpeedAt12Volts
                    .in(edu.wpi.first.units.Units.MetersPerSecond) * driveScale);
            }
        }

        private double getSwerveRotation() {
            double rotAxis = ControllerConstants.isMambo ? driver.getRawAxis(3) : getLeftYaw();
            rotAxis = ControllerConstants.invertRAxis ? rotAxis : -rotAxis;
            rotAxis *= speedFactor;

            if (Math.abs(rotAxis) < kSwerveDeadband) return 0.0;
            double driveScale = driver.getRawAxis(kSlowDriveAxis) > kSlowDriveThreshold ? kSlowDriveScale : 1.0;
            return SwerveConstants.MaxAngularRate *
                (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband) * driveScale;
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

        public static final class CustomXboxController {
            private final XboxController mController;
            public enum Side { LEFT, RIGHT }
            public enum Axis { X, Y }
            private enum Button {
                A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);
                public final int id;
                Button(int id) { this.id = id; }
            }

            public XboxController getController() {
                return mController;
            }

            private CustomXboxController(int port) {
                mController = new XboxController(port);
            }

            public double getAxis(Side side, Axis axis) {
                boolean left = side == Side.LEFT;
                boolean y = axis == Axis.Y;
                return mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0));
            }

            public boolean getTrigger(Side side) {
                return mController.getRawAxis(side == Side.LEFT ? 2 : 3) > ControllerConstants.kTriggerThreshold;
            }

            public boolean getButton(Button button) {
                return mController.getRawButton(button.id);
            }

            public static final double scaleWithDeadband(final double joyValue, final double deadband) {
                return MathUtil.isNear(0, joyValue, deadband) ? 0 :
                    (joyValue < 0 ? joyValue + deadband : joyValue - deadband) / (1 - deadband);
            }
        }

        private static final class ControllerConstants {
            private static final double kInputClipping = 1; // Trimming joystick input by a percent; set to 1 for 100% of
            public static final double kTriggerThreshold = 0.2; //                                         joystick range

            private static final double stickDeadband = 0.02;
            private static final int leftXAxis = 0;
            // public static final int leftYAxis = 1;
            private static final int rightXAxis = 3;
            private static final int rightYAxis = 4;

            private static final boolean invertYAxis = false;
            private static final boolean invertRAxis = false;
            private static final boolean invertXAxis = true;

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
