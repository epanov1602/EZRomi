package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.trajectories.CheckersTrajectoryGenerator;



/* Timed robot with joystick and a two-motor drivetrain (differential drive) */

public class Robot extends TimedRobot {
  private final Joystick m_joystick = new Joystick(0);
  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);

  private final NetworkTable m_reportedOdometry = NetworkTableInstance.getDefault().getTable("odometry");
  private final Stopwatch m_stopwatch = new Stopwatch();

  private final CheckersTrajectoryGenerator m_trajectoryGenerator = new CheckersTrajectoryGenerator();


  // at the start we assume that we have not reached targets yet
  private boolean reachedTarget0 = false;
  private boolean reachedTarget1 = false;
  private boolean reachedTarget2 = false;


  /** This function is called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {
    // if we have not reached target0, work on that
    if (reachedTarget0 == false) {
      boolean gotThere = getToTarget(Constants.AutonomousTargetX0, Constants.AutonomousTargetY0);
      if (gotThere == true) {
        System.out.println("Target 0 reached");
        reachedTarget0 = true;
      }
    }
    
    // otherwise if we have not reached target1, work on that
    else if (reachedTarget1 == false) {
      boolean gotThere = getToTarget(Constants.AutonomousTargetX1, Constants.AutonomousTargetY1);
      if (gotThere) {
        reachedTarget1 = true;
        System.out.println("Target 1 reached");
      }
    }

    // otherwise if we have not reached target2, work on that
    else if (reachedTarget2 == false) {
      boolean gotThere = getToTarget(Constants.AutonomousTargetX2, Constants.AutonomousTargetY2);
      if (gotThere == true) {
        reachedTarget2 = true;
        System.out.println("Target 2 reached");
      }
    }

    // else looks like we reached all the targets, hit the stopwatch
    else {
      m_stopwatch.onFinished();
    }
  }


  public boolean getToTarget(double targetX, double targetY) {
    // -- where are we now?
    Pose2d position = m_odometry.getPoseMeters();
    double currentX = position.getX();
    double currentY = position.getY();

    // -- maybe we have finished?
    double distanceToTarget = Math.sqrt( (currentX - targetX) * (currentX - targetX) + (currentY - targetY) * (currentY - targetY) );
    if (distanceToTarget <= Constants.AutonomousTargetRadius) {
      m_drivetrain.m_leftMotor.set(0);
      m_drivetrain.m_rightMotor.set(0);
      return true; // this means set gotThere=true, because we got there
    }

    // -- not finished yet, make the next step
    m_trajectoryGenerator.getNextStep(position, targetX, targetY, m_drivetrain, m_stopwatch);
    return false; // this means set gotThere=false, because we did not get there yet
  }



  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    m_stopwatch.onStartedRacing();
    m_odometry.resetPosition(new Rotation2d(0), 0, 0, new Pose2d());
    m_drivetrain.resetEncoders();
  }



  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // here we will not use buttons, we will use stick input, also known as "axis" input
    double forwardSpeed = -m_joystick.getRawAxis(5); // 1.0 means "full forward", -1.0 means "full reverse"
    double rotationSpeed = m_joystick.getRawAxis(0);

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
    teleopPeriodic(); // just do the same thing as what you do in teleop mode
  }


  /** This function is called every 20 ms, no matter the mode. */
  @Override
  public void robotPeriodic() {
    // update the odometry
    Drivetrain sensors = m_drivetrain;

    if (Constants.UseSimulatedDriveTrain) {
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

  RobotHttpResponder m_http = new RobotHttpResponder(8080);

  /** This function is run when the robot is first started up. */
  @Override
  public void robotInit() {
    m_http.start();    
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

  private final SimulatedDrivetrain m_simulatedDrivetrain = new SimulatedDrivetrain();
}
