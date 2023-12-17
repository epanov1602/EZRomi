package frc.robot;

import java.io.Console;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.sensors.RomiLimelight;


/* Timed robot with joystick and a two-motor drivetrain (differential drive) */

public class Robot extends TimedRobot {
  private final Joystick m_joystick = new Joystick(0);
  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  private final RomiLimelight m_camera = new RomiLimelight();





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
      m_drivetrain.m_leftMotor.set(0.2);
      m_drivetrain.m_rightMotor.set(0.2);
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





  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // here we will not use buttons, we will use stick input, also known as "axis" input
    double forwardSpeed = -m_joystick.getRawAxis(1); // 1.0 means "full forward", -1.0 means "full reverse"
    double rotationSpeed = m_joystick.getRawAxis(4);

    double leftMotorSpeed = forwardSpeed + 0.3 * rotationSpeed; // if we are turning with positive rotation speed, left motor spins a bit faster
    double rightMotorSpeed = forwardSpeed - 0.3* rotationSpeed; // if we are turning with positive rotation speed, right motor spins a bit slower

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



  /** This function is called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {
    double targetX = m_camera.getX(); // do we see the target? what is its X?
    if (targetX > 7) {
      System.out.println("target is to our right => turning right: x=" + targetX);
      m_drivetrain.m_leftMotor.set(0.1);
      m_drivetrain.m_rightMotor.set(-0.1);
    }
    else if (targetX < -7) {
      System.out.println("target is to our left => turning left: x=" + targetX);
      m_drivetrain.m_leftMotor.set(-0.1);
      m_drivetrain.m_rightMotor.set(0.1);
    }
    else if (targetX != 0) {
      // we see the target, but don't need to turn left or right; maybe change the code to chase it?
      m_drivetrain.m_leftMotor.set(0.8);
      m_drivetrain.m_rightMotor.set(0.8);
    }
    else {
      // we do not see the target => stop
      m_drivetrain.m_leftMotor.set(0);
      m_drivetrain.m_rightMotor.set(0);     
    }
  }

  /** This function is called every 20 ms, no matter the mode. */
  @Override
  public void robotPeriodic() {}

  /** This function is run when the robot is first started up. */
  @Override
  public void robotInit() {}

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    m_camera.setPipeline(2); // camera pipeline 2 is for recognizing game pieces
  }

  /** This function is called once when teleop mode is enabled. */
  @Override
  public void teleopInit() {
    m_camera.setPipeline(0); // camera pipeline 0 is for just regular camera view
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_camera.setPipeline(0); // camera pipeline 0 is for just regular camera view
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when robot is disabled. */
  @Override
  public void disabledPeriodic() {}
}
