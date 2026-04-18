package frc.robot.subsystems.launcher;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class LauncherConstants {
        // --- Turret & Yaw Constants ---
        // new wire mgmt & turret: with no guard attached to the launch wheel, [-135 -> 225] prevents
        // the wire harness from touching the tread (which would grab it!)
        public static final Pair<Double, Double> yawBounds = new Pair<Double, Double>(-125.0, 235.0);
        public static final double c11Offset = 0.6926; // 11-tooth cancoder value at 0 degrees
        public static final double c12Offset = 0.7485; // 12-tooth cancoder value at 0 degrees
        public static final double yawGearRatio = 35; // rotations of yaw motor to give a full rotation of turret
        public static final double launchGearRatio = 0.66666667;
        /** When near a soft limit, allow this much past the limit without re-homing via wrap (hold position). */
        public static final double kTurretLimitPastHoldDeg = 5.0;
        /** Scale on yaw Motion Magic cruise / accel / jerk when the turret must take the long wrap path (80% less → 0.2). */
        public static final double kYawLongPathMotionMagicScale = 0.50;
        /** End latch (return to full Motion Magic limits) when within this many degrees of the wrapped setpoint. */
        public static final double kYawLongPathArriveEpsilonDeg = 3.0;
        public static final double kYawMotionMagicCruiseVelocity = 100;
        public static final double kYawMotionMagicAcceleration = 400;
        public static final double kYawMotionMagicJerk = 1500;

        // --- Pitch (Hood) Constants ---
        private static final double minHoodAngle = 20.0; // hood angle relative to horizontal
        private static final double maxHoodAngle = 55.0;
        public static final double minPitch = 90 - maxHoodAngle; // Ball exit angle relative to horizontal
        public static final double maxPitch = 90 - minHoodAngle; // 0 meaning a shot paralell to the floor and 90 meaning vertical shot
        public static final Pair<Double, Double> pitchBounds = new Pair<Double, Double>(minPitch, maxPitch); // min, max pitch -- in degrees
        public static final double pitchGearRatio = (1 / 53.454545) * 360; // In pitch degrees per rotation because hood has a 1:53.454545 ratio.
        public static final double pitchForceTorque = -5;
        public static final double pitchForceVelocityLimit = -0.1;
        public static final double pitchLimitRotations = -0.25;

        // --- Default PID & Control Tuning ---
        public static final double kDefaultLaunchP = 0.02;
        public static final double kDefaultLaunchI = 0.0;

        public static final double kDefaultYawP = 0.4;
        public static final double kDefaultYawI = 0.01;
        public static final double kDefaultYawD = 0.022;

        // --- Yaw Control ---
        public static final double cableTensionFeedforwardMagnitude = 0.01; // was 0.04
        public static final double yawFricationFeedforwardMagnitude = 0.025; // was 0.1

        // --- Motor Duty Cycles ---
        public static final double kFeederSpinnerMotionDuty = 0.8; // for washing machine speed
        public static final double kFeederRollerMotionDuty = 0.5; // for feeder roller speed
        /** In autonomous, wait this many seconds before starting feeder spinner + roller */
        public static final double kFireFeederStartDelayAutonomous = 0.5;
        /** When turret needs to swivel 360 degrees, wait this long before starting feeder spinner + roller. */
        public static final double turretSpinFeederDelaySeconds = 0.2;

        // --- Shooter Kinematics & Polynomials ---
        public static final double kPitchOffset = 0; // degrees to offset pitch
        public static final double kRPMScaleFactor = 1.425; // scale to offset velocity
        public static final double projectionTime = 0.07; // seconds into the future to predict position
        public static final double fudgeFactorLinearVelocityProjection = 3;
        public static final double fudgeFactorYawAdjustment = 1;
        public static final double yawOffset = 0; // degrees
        public static final double kRPMTolerance = 0.05; // tolerance when checking flywheel speed is within range
        public static final double maxYawAdjustment = 10.0; // degrees to adjust yaw to compensate tangential velocity

        // Hood Angle Coefficients (p00, p10, p01, p20, p11, p02, p30, p21, p12, p03)
        public static final double[] HOOD_COEFFS = { 88.67, -8.535, 9.511, 0.8345, -1.303, 0.5606, -0.03808,
                0.08488, -0.08481, 0.009661 };
        // Exit Velocity (RPM) Coefficients (p00, p10, p01, p20, p11, p02, p30, p21, p12, p03)
        public static final double[] VELO_COEFFS = { 6.515, 0.4246, -0.2237, 0.01338, -0.0556, 0.02836, -0.0003368, // -0.003136
                0.0007127, 0.0003001, -0.001331 };

        // --- Geometry Constants ---
        public static final double wheelDiameter = 0.137; // in meters
        /** Coordinates are in meters following wpiblue coordinate convention */
        public static final Translation2d turretPosRobotRel = new Translation2d(-0.033147, 0.1394968);
}
