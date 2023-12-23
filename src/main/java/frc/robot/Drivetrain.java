package frc.robot;

public interface Drivetrain {
    public double getLeftDistanceInch();
    public double getRightDistanceInch();
    public double getAngleZDegrees();

    void arcadeDrive(double forwardSpeed, double rotationAngle);
}
