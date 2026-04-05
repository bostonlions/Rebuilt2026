package frc.robot.subsystems.launcher;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class LauncherConstants {
        // --- Turret & Yaw Constants ---
        public static final Pair<Double, Double> yawBounds = new Pair<Double, Double>(-90.0, 230.0); // in degrees, maximum is like -100 - 260 but I make it less bc the wire management was breaking
        public static final double c11Offset = 0.01977; // 11-tooth cancoder value at 0 degrees
        public static final double c12Offset = 0.0302; // 12-tooth cancoder value at 0 degrees
        public static final double yawGearRatio = 21; // rotations of yaw motor to give a full rotation of turret
        public static final double launchGearRatio = 0.6666666667;
        /** When near a soft limit, allow this much past the limit without re-homing via wrap (hold position). */
        public static final double kTurretLimitPastHoldDeg = 5.0;
        /** Scale on yaw Motion Magic cruise / accel / jerk when the turret must take the long wrap path (80% less → 0.2). */
        public static final double kYawLongPathMotionMagicScale = 0.90;
        /** End latch (return to full Motion Magic limits) when within this many degrees of the wrapped setpoint. */
        public static final double kYawLongPathArriveEpsilonDeg = 3.0;
        public static final double kYawMotionMagicCruiseVelocity = 50; // was 600
        public static final double kYawMotionMagicAcceleration = 250; // TODO: check this
        public static final double kYawMotionMagicJerk = 1000; // 10000 works

        // --- Pitch (Hood) Constants ---
        private static final double minHoodAngle = 20.0; // hood angle relative to horizontal
        private static final double maxHoodAngle = 55.0;
        public static final double minPitch = 90 - maxHoodAngle; // Ball exit angle relative to horizontal
        public static final double maxPitch = 90 - minHoodAngle; // 0 meaning a shot paralell to the floor and 90 meaning vertical shot
        public static final Pair<Double, Double> pitchBounds = new Pair<Double, Double>(minPitch, maxPitch); // min, max pitch -- in degrees
        public static final double pitchGearRatio = (1 / 53.4545) * 360; // In pitch degrees per rotation because hood has a 1:53.4545 ratio. 
        public static final double pitchForceTorque = -5;
        public static final double pitchForceVelocityLimit = -0.1;
        public static final double pitchLimitRotations = -0.25;

        // --- Default PID & Control Tuning ---
        public static final double kDefaultLaunchP = 0.02;
        public static final double kDefaultLaunchI = 0.0;
        public static final double kDefaultLaunchD = 0.0;

        public static final double kDefaultYawP = 0.4;
        public static final double kDefaultYawI = 0.00;
        public static final double kDefaultYawD = 0.022;

        // --- Motor Duty Cycles ---
        public static final double kFeederSpinnerMotionDuty = 0.7; // for washing machine speed (can be .5)
        public static final double kFeederSpinnerWithIntakeRequestDuty = 0.1; // for washing machine speed when intaking
        public static final double kFeederRollerMotionDuty = 0.9; // for feeder roller speed

        // --- Shooter Kinematics & Polynomials ---
        public static final double kPitchOffset = 0.0; // (deg) to offset pitch if shooting based on distance is missing
        public static final double kRPMScaleFactor = 1.37; // (m/s) scale to offset velocity if shooting based on distance is missing
        public static final double projectionTime = 0.1; // seconds into the future to predict position
        public static final double kRPMTolerance = 0.05; // tolerance when checking flywheel speed is within range
        public static final double maxYawAdjustment = 10.0; // degrees to adjust yaw to compensate tangential velocity

        // Hood Angle Coefficients (p00, p10, p01, p20, p11, p02, p30, p21, p12, p03)
        public static final double[] HOOD_COEFFS = { 90.26, -13.17, 7.925, 1.086, 0.009438, 0.04452, -0.03133,
                -0.02169, 0.08575, -0.08523 };
        // Exit Velocity (RPM) Coefficients (p00, p10, p01, p20, p11, p02, p30, p21, p12, p03)
        public static final double[] VELO_COEFFS = { 5.505, 0.455, -0.2255, 0.05223, -0.1248, 0.04985, -0.003136, // -0.003136
                0.005095, 0.000165, 0.004498 };

        // --- Geometry Constants ---
        // public static final double ballExitHeight = 0.391; // meters = 15.376 in 
        public static final Translation2d turretPosRobotRel = new Translation2d(-0.033147, 0.1394968); // coordinates are in meters following wpiblue coordinate convention
        // public static final double blueTrenchLocation = 4.625594;
        // public static final double redTrenchLocation = 11.915394; // meters
        // public static final double trenchTolerance = 0.61; // how close the robot has to be to the trench for the hood to go down
}
