package org.team1912.pyrogen.pyrolib.ftclib.command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.team1912.pyrogen.pyrolib.ftclib.drivebase.DifferentialDrive;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.kinematics.DifferentialOdometry;
import org.team1912.pyrogen.pyrolib.ftclib.purepursuit.DifferentialPath;
import org.team1912.pyrogen.pyrolib.ftclib.purepursuit.Waypoint;

/**
 * @author Jackson
 * @see DifferentialPath
 */
public class DifferentialPPCommand extends CommandBase {

    private DifferentialDrive m_drive;
    private DifferentialOdometry m_odometry;
    private DifferentialPath m_path;
    private Telemetry m_telemetry;

    public DifferentialPPCommand(Telemetry telemetry, DifferentialDrive drive, DifferentialOdometry odometry, Waypoint... waypoints) {
        m_path = new DifferentialPath(waypoints);
        m_drive = drive;
        m_odometry = odometry;
        m_telemetry = telemetry;
    }

    @Override
    public void initialize() {
        m_path.init();
    }

    public void addWaypoint(Waypoint waypoint) {
        m_path.add(waypoint);
    }

    public void addWaypoints(Waypoint... waypoints) {
        for (Waypoint waypoint : waypoints) this.addWaypoint(waypoint);
    }

    public void removeWaypointAtIndex(int index) {
        m_path.remove(index);
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        Pose2d robotPose = m_odometry.getPose();
        double[] motorSpeeds = m_path.loop_dbg(
                m_telemetry,
                robotPose.getTranslation().getX(),
                robotPose.getTranslation().getY(),
                robotPose.getHeading());
        m_telemetry.addData("position >", "%f %f %f",
                robotPose.getX(),
                robotPose.getY(),
                robotPose.getHeading());
        m_telemetry.addData("translation >", "%f %f %f",
                robotPose.getTranslation().getX(),
                robotPose.getTranslation().getY(),
                robotPose.getHeading());
        m_telemetry.addData("motorSpeeds >", "%f %f",
                motorSpeeds[0],
                motorSpeeds[1]);
        m_telemetry.update();
        m_drive.arcadeDrive(motorSpeeds[0], motorSpeeds[1]);
        m_odometry.updatePose();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return m_path.isFinished();
    }

}
