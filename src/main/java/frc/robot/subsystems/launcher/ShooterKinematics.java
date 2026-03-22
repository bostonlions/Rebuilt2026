package frc.robot.subsystems.launcher;
// import frc.robot.subsystems.launcher.LauncheConstants;

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

    public double getTargetFlywheelRPM(double distance, double radialVelo) {
        double exitVeloMetersPerSec = calculatePoly33(distance, radialVelo, LauncherConstants.VELO_COEFFS);

        // 1. Convert Ball Exit Velo to Wheel Surface Speed (V_wheel = 2 * V_ball)
        double wheelSurfaceVelo = exitVeloMetersPerSec * 2.0;

        // 2. Convert to RPM for 5" Wheel
        // Circumference = 5 * pi * 0.0254 meters
        double wheelCircumference = 5.0 * 0.0254 * Math.PI;
        double rpm = (wheelSurfaceVelo / wheelCircumference)*60;

        return lib.Util.clamp(rpm * LauncherConstants.kRPMScaleFactor, 0, 6065);
    }
}