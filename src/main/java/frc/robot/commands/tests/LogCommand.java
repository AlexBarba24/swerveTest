// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/**
 * Writes one line to the console and the dashboard and finishes immediately.
 *
 * <p>Requires no subsystem, so it can separate steps inside a test sequence or report a value read
 * at that point in the sequence.
 */
public class LogCommand extends Command {
  private final Supplier<String> m_message;

  /**
   * Logs a fixed message.
   *
   * @param message the line to publish
   */
  public LogCommand(String message) {
    this(() -> message);
  }

  /**
   * Logs a message built when the command runs, for reporting state that is only known by then.
   *
   * @param message supplies the line to publish
   */
  public LogCommand(Supplier<String> message) {
    m_message = message;
    setName("Log");
  }

  @Override
  public void initialize() {
    TimedTestCommand.log(m_message.get());
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
