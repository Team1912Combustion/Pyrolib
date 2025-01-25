package org.team1912.pyrogen.pyrolib.ftclib.kinematics;

import org.team1912.pyrogen.pyrolib.GoBildaPinpoint.PinpointSensor;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Pose2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Rotation2d;
import org.team1912.pyrogen.pyrolib.ftclib.geometry.Twist2d;

public class GoBildaOdometry extends Odometry {

    private double prevX, prevY;
    private Rotation2d prevAngle;

    private double m_xOffset, m_yOffset;

    // the suppliers
    public PinpointSensor m_pinpoint;

    public GoBildaOdometry(PinpointSensor pinpoint, Pose2d initialPose, double xOffset, double yOffset) {
        super(initialPose);
        m_pinpoint = pinpoint;
        m_pinpoint.initialize();
        prevX = initialPose.getX(); prevY = initialPose.getY();
        prevAngle = initialPose.getRotation();
    }

    public GoBildaOdometry(PinpointSensor pinpoint, Pose2d initialPose) {
        super(initialPose);
        m_pinpoint = pinpoint;
        m_pinpoint.initialize();
        prevX = initialPose.getX(); prevY = initialPose.getY();
        prevAngle = initialPose.getRotation();
    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        update(m_pinpoint.getPose2d());
    }

    @Override
    public void updatePose(Pose2d pose) {
        m_pinpoint.setPose2d(pose);
        prevX = pose.getX();
        prevY = pose.getY();
        prevAngle = pose.getRotation();
        robotPose = pose;
    }

    public Pose2d update(Pose2d new_Pose2d) {

        double X = new_Pose2d.getX();
        double Y = new_Pose2d.getY();
        Rotation2d angle = new_Pose2d.getRotation();

        double dx = X - prevX;
        double dy = Y - prevY;
        double dw = (angle.minus(prevAngle).getRadians());
        Twist2d twist2d = new Twist2d(dx, dy, dw);
        Pose2d newPose = robotPose.exp(twist2d);

        prevX = X;
        prevY = Y;
        prevAngle = angle;
        robotPose = new Pose2d(newPose.getTranslation(), angle);
        return robotPose;
    }

    public Pose2d getSensorPose() {
        return m_pinpoint.getPose2d();
    }
}
