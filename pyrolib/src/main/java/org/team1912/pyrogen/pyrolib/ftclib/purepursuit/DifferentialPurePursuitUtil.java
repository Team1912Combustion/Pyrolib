package org.team1912.pyrogen.pyrolib.ftclib.purepursuit;

/**
 * This class contains various static methods that are used in the pure pursuit algorithm.
 *
 * @author Michael Baljet, Team 14470
 * @version 1.0
 */
public final class DifferentialPurePursuitUtil {

    // This class only has static items so the constructor is hidden.
    private DifferentialPurePursuitUtil() {
    }

    /**
     * Takes the robot's current position and rotation and calculates the motor powers for the robot to move to the target position.
     *
     * @param cx       Robot's current X position.
     * @param cy       Robot's current Y position.
     * @param ca       Robot's current rotation (angle).
     * @param tx       Target X position.
     * @param ty       Target Y position.
     * @param ta       Target rotation (angle).
     * @param turnOnly True if the robot should only turn.
     * @return A double array containing raw motor powers. a[0] is strafe power, a[1] is vertical power and a[2] is turn power.
     */
    public static double[] moveToPosition(double cx, double cy, double ca, double tx, double ty, double ta, boolean turnOnly) {

        double[] rawMotorPowers;

        if (turnOnly)
            // If turnOnly is true, only return a turn power.
            return new double[]{0,  PurePursuitUtil.angleWrap(ta - ca) / Math.PI};

        double absoluteXToPosition = tx - cx;
        double absoluteYToPosition = ty - cy;
        double absoluteAngleToPosition = Math.atan2(absoluteYToPosition, absoluteXToPosition);

        double powerForward = Math.hypot(absoluteXToPosition, absoluteYToPosition) /
		              (Math.abs(absoluteXToPosition) + Math.abs(absoluteYToPosition));
        double powerTurn = PurePursuitUtil.angleWrap(absoluteAngleToPosition - ca) / Math.PI;

        rawMotorPowers = new double[2];

        // The positive rotation counter-clockwise is a negative turn power
        rawMotorPowers[0] = powerForward;
        rawMotorPowers[1] = -powerTurn;

        return rawMotorPowers;
    }
}
