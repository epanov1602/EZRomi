// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Drivetrain;
import frc.robot.Stopwatch;

/** generates jagged trajectory to the target step by step, similar to how pieces move in checkers */
public class CheckersTrajectoryGenerator {
    private final static double ForwardSpeed = 0.5; // 40% speed
    private final static double RotationSpeed = 0.33; // 33% speed
    private final static double AngleTolerance = 15; // plus minus 15 degrees is ok when heading in certain direction

    public void getNextStep(Pose2d currentPose, double targetX, double targetY, Drivetrain drivetrain, Stopwatch stopwatch) {

        if (isTargetNorthEastOfUs(currentPose, targetX, targetY)) {
            stopwatch.setStatusText("target is north east of us");
            boolean correctHeading = ensureHeadingAngle(45, currentPose, drivetrain);
            if (correctHeading)
                drivetrain.arcadeDrive(ForwardSpeed, 0);
            return;
        }
    
        if (isTargetNorthWestOfUs(currentPose, targetX, targetY)) {
            stopwatch.setStatusText("target is north west of us");
            boolean correctHeading = ensureHeadingAngle(-45, currentPose, drivetrain);
            if (correctHeading)
                drivetrain.arcadeDrive(ForwardSpeed, 0);
            return;
        }

        if (isTargetSouthEastOfUs(currentPose, targetX, targetY)) {
            stopwatch.setStatusText("target is south east of us");
            boolean correctHeading = ensureHeadingAngle(135, currentPose, drivetrain);
            if (correctHeading)
                drivetrain.arcadeDrive(ForwardSpeed, 0);
            return;
        }

        if (isTargetSouthWestOfUs(currentPose, targetX, targetY)) {
            stopwatch.setStatusText("target is south west of us");
            boolean correctHeading = ensureHeadingAngle(-135, currentPose, drivetrain);
            if (correctHeading)
                drivetrain.arcadeDrive(ForwardSpeed, 0);
            return;
        }

        stopwatch.setStatusText("ERROR: cannot determine trajectory");
        drivetrain.arcadeDrive(0, 0); // safer to stop and sit
    }

    boolean isTargetNorthEastOfUs(Pose2d position, double targetX, double targetY) {
        if (targetX >= position.getX() && targetY >= position.getY())
            return true;
        else
            return false;
    }

    boolean isTargetNorthWestOfUs(Pose2d position, double targetX, double targetY) {
        if (targetX >= position.getX() && targetY <= position.getY())
            return true;
        else
            return false;
    }

    boolean isTargetSouthEastOfUs(Pose2d position, double targetX, double targetY) {
        if (targetX <= position.getX() && targetY >= position.getY())
            return true;
        else
            return false;
    }

    boolean isTargetSouthWestOfUs(Pose2d position, double targetX, double targetY) {
        if (targetX <= position.getX() && targetY <= position.getY())
            return true;
        else
            return false;
    }

    private boolean ensureHeadingAngle(double targetAngle, Pose2d currentPose, Drivetrain drivetrain) {
        double currentAngle = currentPose.getRotation().getDegrees();

        // do we need to turn further right?
        if (currentAngle < targetAngle - AngleTolerance) {
            drivetrain.arcadeDrive(0, -RotationSpeed);
            return false; // have to keep turning, please set correctHeading=false, since we aren't heading the right way yet
        }

        // do we need to turn further left?
        if (currentAngle > targetAngle + AngleTolerance) {
            drivetrain.arcadeDrive(0, +RotationSpeed);
            return false; // have to keep turning, please set correctHeading=false, since we aren't heading the right way yet
        }

        // we don't need to turn further left or right, our heading is approximately correct
        return true; // pleass set correctHeading=true, so we can proceed with driving forward
    }


    // unused methods, but could be handy for smoother trajectories?

    boolean isTargetStrictlyNorthOfUs(Pose2d position, double targetX, double targetY) {
        if (targetX > position.getX() && Math.abs(targetY - position.getY()) <= 0.5 * Math.abs(targetX - position.getX()))
            return true;
        else
            return false;
    }

    boolean isTargetStrictlySouthOfUs(Pose2d position, double targetX, double targetY) {
        if (targetX < position.getX() && Math.abs(targetY - position.getY()) <= 0.5 * Math.abs(targetX - position.getX()))
            return true;
        else
            return false;
    }

    boolean isTargetStrictlyEastOfUs(Pose2d position, double targetX, double targetY) {
        if (targetY > position.getY() && Math.abs(targetX - position.getX()) <= 0.5 * Math.abs(targetY - position.getY()))
            return true;
        else
            return false;
    }

    boolean isTargetStrictlyWestOfUs(Pose2d position, double targetX, double targetY) {
        if (targetY < position.getY() && Math.abs(targetX - position.getX()) <= 0.5 * Math.abs(targetY - position.getY()))
            return true;
        else
            return false;
    }
}
