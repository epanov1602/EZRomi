package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class SimDrivetrain implements Drivetrain {
    private double m_leftDistance = 0, m_rightDistance = 0;

    private double m_time = -1; // seconds, -1 will be a special value

    public void update(double leftSpeed, double rightSpeed) {
        double time = Timer.getFPGATimestamp();
        if (m_time == -1) {
            m_time = time;
        } else {
            double timePassed = time - m_time;
            m_time = time;
            m_leftDistance = m_leftDistance + timePassed * leftSpeed;
            m_rightDistance = m_rightDistance + timePassed * rightSpeed;
        }
    }

    public double getLeftDistanceInch() {
        return m_leftDistance;
    }

    public double getRightDistanceInch() {
        return m_rightDistance;
    }

    public double getAngleZDegrees() {
        double radians = (m_leftDistance - m_rightDistance) / 14.0; // assuming distance between wheels is 14 inches
        return 57.29 * radians;
    }
}
