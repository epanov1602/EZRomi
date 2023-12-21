package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;



/* Timed robot with joystick and a two-motor drivetrain (differential drive) */

public class Robot extends TimedRobot {
  private final Joystick m_joystick = new Joystick(0);
  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);

  private final NetworkTable m_reportedOdometry = NetworkTableInstance.getDefault().getTable("odometry");


  /** This function is called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {
    // what does odometry say? (where are we now and where are we heading)
    Pose2d position = m_odometry.getPoseMeters();
    double currentHeading = position.getRotation().getDegrees();
    double currentX = position.getX();
    double currentY = position.getY();

    if (currentY < Constants.AutonomousTargetY) {
      // step 1: if our Y is under TargetY, aim for heading around 80-90 degrees (go East) until currentY > TargetY

      if (currentHeading < 90) {
        // if our heading is not yet 90 degrees, keep turning right until that heading angle reaches 90 degrees
        m_drivetrain.m_leftMotor.set(0.2);
        m_drivetrain.m_rightMotor.set(-0.2);
      } else {
        // otherwise, clear to go forward
        m_drivetrain.m_leftMotor.set(0.5);
        m_drivetrain.m_rightMotor.set(0.5);
      }

    }
    else if (currentX < Constants.AutonomousTargetX) {

      // case 2: we have reached our target Y, but not X: what should I do here? please help!
      // (for now, set motor speed to zero)
      m_drivetrain.m_leftMotor.set(0);
      m_drivetrain.m_rightMotor.set(0);

    }
    else {

      // case 3: we have reached or overshot, stop
      m_drivetrain.m_leftMotor.set(0);
      m_drivetrain.m_rightMotor.set(0);

    }
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

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
  }

  /** This function is called once when teleop mode is enabled. */
  @Override
  public void teleopInit() {
  }

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
