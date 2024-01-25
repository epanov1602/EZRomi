package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.sensors.RomiLimelight;



/* Timed robot with joystick and a two-motor drivetrain (differential drive) */

public class Robot extends TimedRobot {
  private final Joystick m_joystick = new Joystick(0);
  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);
  private final RomiLimelight m_camera = new RomiLimelight();

  private final NetworkTable m_reportedOdometry = NetworkTableInstance.getDefault().getTable("odometry");



  /** This function is called once when teleop mode is enabled. */
  @Override
  public void autonomousInit() {
    m_camera.setPipeline(3);
  }

  /** This function is called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {
    double targetX = m_camera.getX();
    double targetSize = m_camera.getA();

    if (targetX == 0) {
      System.out.println("I don't see the target anymore, not moving");
      m_drivetrain.m_leftMotor.set(0);
      m_drivetrain.m_rightMotor.set(0);
    }
    else if (targetSize > 6) {
      // we are pretty close
      System.out.println("not moving anymore, because the target is too big: size=" + targetSize);
      m_drivetrain.m_leftMotor.set(0);
      m_drivetrain.m_rightMotor.set(0);
    }
    else if (targetX > 2.0) {
      System.out.println("targetX=" + targetX);
      // to do: the target is to the right of us -- we must move forward while turning left
    }
    else if (targetX < -2.0) {
      System.out.println("targetX=" + targetX);
      // to do: the target is to the left of us -- we must move forward while turning left
    }
    else {
      System.out.println("nothing to do, so stopping: targetX=" + targetX + ", targetSize=" + targetSize);
      m_drivetrain.m_leftMotor.set(0);
      m_drivetrain.m_rightMotor.set(0);
    }
  }



  /** This function is called once when teleop mode is enabled. */
  @Override
  public void teleopInit() {
    m_camera.setPipeline(0);
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {  
    // here we will not use buttons, we will use stick input, also known as "axis" input
    double forwardSpeed = -m_joystick.getRawAxis(1); // 1.0 means "full forward", -1.0 means "full reverse"
    double rotationSpeed = m_joystick.getRawAxis(4);

    double leftMotorSpeed = forwardSpeed + 0.5 * rotationSpeed; // if we are turning with positive rotation speed, left motor spins a bit faster
    double rightMotorSpeed = forwardSpeed - 0.5* rotationSpeed; // if we are turning with positive rotation speed, right motor spins a bit slower

    // the highest positive motor speed is 1.0 (which means 100%), make sure we do not break this speed limit
    if (leftMotorSpeed > 1) leftMotorSpeed = 1;
    if (rightMotorSpeed > 1) rightMotorSpeed = 1;
  
    // the most negative motor speed is -1.0 (which means -100%), make sure we do not break this speed limit
    if (leftMotorSpeed < -1) leftMotorSpeed = -1;
    if (rightMotorSpeed < -1) rightMotorSpeed = -1;
  
    // set those speeds on the motors
    m_drivetrain.m_leftMotor.set(leftMotorSpeed);
    m_drivetrain.m_rightMotor.set(rightMotorSpeed);
  }


  /** This function is called periodically during test mode */
  @Override
  public void testPeriodic() {
    // depending on which button is pressed, do something
    if (m_joystick.getRawButton(2)) {
      System.out.println("button 2 is pressed => turning left");
      m_drivetrain.m_leftMotor.set(0.5);
      m_drivetrain.m_rightMotor.set(0);
    }
    else if (m_joystick.getRawButton(3)) {
      System.out.println("button 3 pressed => turning right");
      m_drivetrain.m_leftMotor.set(0);
      m_drivetrain.m_rightMotor.set(0.5);
    }
    else if (m_joystick.getRawButton(4)) {
      System.out.println("button 4 pressed => going forward at 80% speed");
      m_drivetrain.m_leftMotor.set(0.8);
      m_drivetrain.m_rightMotor.set(0.8);
    }
    else if (m_joystick.getRawButton(1)) {
      System.out.println("button 4 pressed => going backward at 80% speed");
      m_drivetrain.m_leftMotor.set(-0.5);
      m_drivetrain.m_rightMotor.set(-0.5);
    } else {
      System.out.println("none of those buttons pressed => not moving");
      m_drivetrain.m_leftMotor.set(0);
      m_drivetrain.m_rightMotor.set(0);
    }
  }


  /** This function is called every 20 ms, no matter the mode. */
  @Override
  public void robotPeriodic() {
    // update the odometry
    Drivetrain sensors = m_drivetrain;

    if (UseSimulatedDriveTrain) {
      double leftSpeed = 10 * m_drivetrain.m_leftMotor.get();
      double rightSpeed = 10 * m_drivetrain.m_rightMotor.get();
      m_simulatedDrivetrain.update(leftSpeed, rightSpeed);
      sensors = m_simulatedDrivetrain;
    }

    m_odometry.update(Rotation2d.fromDegrees(sensors.getAngleZDegrees()), sensors.getLeftDistanceInch(), sensors.getRightDistanceInch());

    Pose2d position = m_odometry.getPoseMeters();
    m_reportedOdometry.getEntry("X").setDouble(position.getX());
    m_reportedOdometry.getEntry("Y").setDouble(position.getY());
    m_reportedOdometry.getEntry("heading").setDouble(position.getRotation().getDegrees());
  }

  /** This function is run when the robot is first started up. */
  @Override
  public void robotInit() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when robot is disabled. */
  @Override
  public void disabledPeriodic() {}

  private final SimDrivetrain m_simulatedDrivetrain = new SimDrivetrain();
  private static final boolean UseSimulatedDriveTrain = false;
}
