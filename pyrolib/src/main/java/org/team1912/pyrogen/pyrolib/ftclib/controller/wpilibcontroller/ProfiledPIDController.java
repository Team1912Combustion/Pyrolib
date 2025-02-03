/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team1912.pyrogen.pyrolib.ftclib.controller.wpilibcontroller;

import org.team1912.pyrogen.pyrolib.ftclib.controller.PIDController;
import org.team1912.pyrogen.pyrolib.ftclib.trajectory.TrapezoidProfile;
import org.team1912.pyrogen.pyrolib.ftclib.util.MathUtil;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid
 * profile.
 */
@SuppressWarnings("PMD.TooManyMethods")
public class ProfiledPIDController {
    private PIDController m_controller;

    private double m_minimumInput;
    private double m_maximumInput;

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.Constraints m_constraints;

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
     * Kd.
     *
     * @param Kp          The proportional coefficient.
     * @param Ki          The integral coefficient.
     * @param Kd          The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    @SuppressWarnings("ParameterName")
    public ProfiledPIDController(double Kp, double Ki, double Kd,
                                 TrapezoidProfile.Constraints constraints) {
        m_controller = new PIDController(Kp, Ki, Kd);
        m_constraints = constraints;
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Sets the proportional, integral, and differential coefficients.
     *
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Differential coefficient
     */
    @SuppressWarnings("ParameterName")
    public void setPID(double Kp, double Ki, double Kd) {
        m_controller.setPID(Kp, Ki, Kd);
    }

    /**
     * Sets the proportional coefficient of the PID controller gain.
     *
     * @param Kp proportional coefficient
     */
    @SuppressWarnings("ParameterName")
    public void setP(double Kp) {
        m_controller.setP(Kp);
    }

    /**
     * Sets the integral coefficient of the PID controller gain.
     *
     * @param Ki integral coefficient
     */
    @SuppressWarnings("ParameterName")
    public void setI(double Ki) {
        m_controller.setI(Ki);
    }

    /**
     * Sets the differential coefficient of the PID controller gain.
     *
     * @param Kd differential coefficient
     */
    @SuppressWarnings("ParameterName")
    public void setD(double Kd) {
        m_controller.setD(Kd);
    }

    /**
     * Gets the proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return m_controller.getP();
    }

    /**
     * Gets the integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return m_controller.getI();
    }

    /**
     * Gets the differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return m_controller.getD();
    }

    /**
     * Gets the period of this controller.
     *
     * @return The period of the controller.
     */
    public double getPeriod() {
        return m_controller.getPeriod();
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal state.
     */
    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     */
    public void setGoal(double goal) {
        m_goal = new TrapezoidProfile.State(goal, 0);
    }

    /**
     * Gets the goal for the ProfiledPIDController.
     */
    public TrapezoidProfile.State getGoal() {
        return m_goal;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     */
    public boolean atGoal() {
        return atSetpoint() && m_goal.equals(m_setpoint);
    }

    /**
     * Set velocity and acceleration constraints for goal.
     *
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        m_constraints = constraints;
    }

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     *
     * @return The current setpoint.
     */
    public TrapezoidProfile.State getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     */
    public boolean atSetpoint() {
        return m_controller.atSetPoint();
    }


    /**
     * Enables continuous input.
     *
     * <p>Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_controller.enableContinuousInput(minimumInput, maximumInput);
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        m_controller.disableContinuousInput();
    }

    /**
     * Sets the minimum and maximum contributions of the integral term.
     *
     * <p>The internal integrator is clamped so that the integral term's contribution to the output
     * stays between minimumIntegral and maximumIntegral. This prevents integral windup.
     *
     * @param minimumIntegral The minimum contribution of the integral term.
     * @param maximumIntegral The maximum contribution of the integral term.
     */

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_controller.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return m_controller.getPositionError();
    }

    /**
     * Returns the change in error per second.
     */
    public double getVelocityError() {
        return m_controller.getVelocityError();
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     */
    public double calculate(double measurement) {
        if (m_controller.isContinuousInputEnabled()) {
            // Get error which is the smallest distance between goal and measurement
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            double goalMinDistance =
                    MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
            double setpointMinDistance =
                    MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);

            // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
            // may be outside the input range after this operation, but that's OK because the controller
            // will still go there and report an error of zero. In other words, the setpoint only needs to
            // be offset from the measurement by the input range modulus; they don't need to be equal.
            m_goal.position = goalMinDistance + measurement;
            m_setpoint.position = setpointMinDistance + measurement;
        }

        TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(getPeriod());
        return m_controller.calculate(measurement, m_setpoint.position);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     */
    public double calculate(double measurement, TrapezoidProfile.State goal) {
        setGoal(goal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PIDController.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     */
    public double calculate(double measurement, double goal) {
        setGoal(goal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public double calculate(double measurement, TrapezoidProfile.State goal,
                            TrapezoidProfile.Constraints constraints) {
        setConstraints(constraints);
        return calculate(measurement, goal);
    }

    /**
     * Reset the previous error, the integral term, and disable the controller.
     */
    public void reset() {
        m_controller.reset();
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     */
    public void reset(TrapezoidProfile.State measurement) {
        m_controller.reset();
        m_setpoint = measurement;
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system.
     * @param measuredVelocity The current measured velocity of the system.
     */
    public void reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system. The velocity is
     *                         assumed to be zero.
     */
    public void reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
    }

}
