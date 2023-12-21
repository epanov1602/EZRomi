// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/** Stopwatch for the race: if still racing, call onRacing() otherwise call onFinished() */
public class Stopwatch {
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("stopwatch");
    private final NetworkTableEntry m_status = m_table.getEntry("status");
    private final NetworkTableEntry m_raceTime = m_table.getEntry("raceTime");

    private double m_startTime = -1; /* -1 will serve as a special value which means "unset" */
    private double m_finishTime = -1; /* -1 will serve as a special value which means "unset" */
    private String m_lastStatusText = "";

    public void setStatusText(String statusText) {
        if (m_finishTime != -1)
            return; // finished already
        m_raceTime.setDouble(Timer.getFPGATimestamp() - m_startTime);
        if (statusText != m_lastStatusText) {
            m_status.setString(statusText);
            m_lastStatusText = statusText;
        }
    }

    public void onStartedRacing() {
        if (m_startTime == -1) {
            m_startTime = Timer.getFPGATimestamp();
            setStatusText("started");
        }
    }

    public void onFinished() {
        if (m_finishTime == -1) {
            setStatusText("finished");
            m_finishTime = Timer.getFPGATimestamp();
            m_raceTime.setDouble(m_finishTime - m_startTime);
        }
    }

    public void reset() {
        m_finishTime = -1;
        m_startTime = -1;
        m_lastStatusText = "";
        m_raceTime.setDouble(0);
        m_status.setString("");
    }
}
