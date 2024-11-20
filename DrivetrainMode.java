package frc.robot.SyncedLibraries;

public enum DrivetrainMode {
  /** Full movement */
  NORMAL,

  /** AKA X-lock.  */
  LOCKED,

  /** Movement locked, only rotate, used for aiming (field relative reccomended) */
  ROTATE
}