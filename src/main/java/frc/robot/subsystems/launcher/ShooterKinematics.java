package frc.robot.subsystems.launcher;
// import frc.robot.subsystems.launcher.LauncheConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Drive.Drivetrain;

public class ShooterKinematics {
    /**
     * Calculates the result of a Poly33 surface fit
     *
     * @param x Distance to target (m)
     * @param y Radial velocity of robot (m/s)
     * @param p Coefficients array
     */
    private double calculatePoly33(double x, double y, double[] p) {
        return p[0] + p[1] * x + p[2] * y + p[3] * Math.pow(x, 2) + p[4] * x * y + p[5] * Math.pow(y, 2)
                + p[6] * Math.pow(x, 3) + p[7] * Math.pow(x, 2) * y + p[8] * x * Math.pow(y, 2) + p[9] * Math.pow(y, 3);
    }

    public double getTargetHoodAngle(double distance, double radialVelo) {
        return lib.Util.clamp(calculatePoly33(distance, radialVelo, LauncherConstants.HOOD_COEFFS) + LauncherConstants.kPitchOffset, LauncherConstants.minPitch, LauncherConstants.maxPitch);
    }

    public double getExitVeloMetersPerSec(double distance, double radialVelo) {
        return calculatePoly33(distance, radialVelo, LauncherConstants.VELO_COEFFS);
    }

    public double getTargetFlywheelRPM(double exitVeloMetersPerSec) {
        double wheelSurfaceVelo = exitVeloMetersPerSec * 2;
        double wheelCircumference = LauncherConstants.wheelDiameter * Math.PI;
        double rpm = (wheelSurfaceVelo / wheelCircumference) * 60;

        return lib.Util.clamp(rpm * LauncherConstants.kRPMScaleFactor, 0, 6065);
    }

    public double getAdjustedFlywheelRPM(double targetRPM) {
        return targetRPM * 1; // TODO: fit a feedforward graph to this
    }

    public double getCableTensionFeedforward(Rotation2d theta) {
        // this is the desmos form of the equasion \frac{-2h}{1+e^{-k\left(x-60\right)}}+h

        return (-(LauncherConstants.cableTensionFeedforwardMagnitude * 2) /
        (1 + (Math.exp(-0.09 * (theta.getDegrees() - 60))))) + LauncherConstants.cableTensionFeedforwardMagnitude;
    }

    public double getYawFrictionFeedworward() {
        double omega = Drivetrain.getInstance().getState().Speeds.omegaRadiansPerSecond;
        if (omega > 0.05) {
            return omega * LauncherConstants.yawFricationFeedforwardMagnitude;
        } else if (omega < -0.05) {
            return omega * LauncherConstants.yawFricationFeedforwardMagnitude;
        }
        return 0;
    }
}