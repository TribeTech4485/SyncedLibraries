package frc.robot.SyncedLibraries;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

public class ManipulatorBaseSysID {
  SysIdRoutine sysIdRoutine;
  ManipulatorBase manipulatorBase;

  /**
   * @param manipulatorBase The manipulator base to be used.
   * @param rampRate        The ramp rate in volts per second.
   * @param stepVoltage     The step voltage in volts.
   * @param timeout         The timeout duration in seconds.
   */
  public ManipulatorBaseSysID(ManipulatorBase manipulatorBase, double rampRate, double stepVoltage, double timeout) {
    this.manipulatorBase = manipulatorBase;
    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(rampRate).per(Second),
            Volts.of(stepVoltage), Seconds.of(timeout)),
        new SysIdRoutine.Mechanism(this::setVoltage, null, manipulatorBase));
  }

  /**
   * Sets the ramp rate to 1v/s, step voltage to 7v, and timeout to 10s.
   * 
   * @param manipulatorBase The manipulator base to be used.
   */
  public ManipulatorBaseSysID(ManipulatorBase manipulator) {
    this(manipulator, 1, 7, 10);
  }

  private void setVoltage(Voltage voltage) {
    manipulatorBase.setVoltage(voltage.magnitude(), false);
  }

  /** Voltage steps, check accel */
  public Command quasistaticForwards() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  /** Voltage steps, check accel */
  public Command quasistaticReverse() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  /** Voltage ramp, no accel */
  public Command dynamicForwards() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  /** Voltage ramp, no accel */
  public Command dynamicReverse() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }
}
