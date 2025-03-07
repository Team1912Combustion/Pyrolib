/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team1912.pyrogen.pyrolib.ftclib.command;

import java.util.function.BooleanSupplier;

/**
 * A command that runs a Runnable continuously.  Has no end condition as-is;
 * either subclass it or use {@link Command#withTimeout(long)} or
 * {@link Command#interruptOn(BooleanSupplier)} to give it one.  If you only wish
 * to execute a Runnable once, use {@link InstantCommand}.
 */
public class RunCommand extends CommandBase {

    protected final Runnable m_toRun;

    /**
     * Creates a new RunCommand.  The Runnable will be run continuously until the command
     * ends.  Does not run when disabled.
     *
     * @param toRun        the Runnable to run
     * @param requirements the subsystems to require
     */
    public RunCommand(Runnable toRun, Subsystem... requirements) {
        m_toRun = toRun;

        addRequirements(requirements);
    }

    @Override
    public void execute() {
        m_toRun.run();
    }

}
