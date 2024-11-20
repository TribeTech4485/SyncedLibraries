package frc.robot.SyncedLibraries;

import java.util.ArrayList;

import frc.robot.SyncedLibraries.SystemBases.ControllerBase;

/**
 * This class is used to select the controller that is actively being used.
 * The first controller in the list that is being touched is the one that is
 * used.
 * <p>
 * If no controllers are being touched, the nullController is used.
 * <p>
 * ONLY CHECKS IF JOYSTICK ARE BEING TOUCHED, NOT THE BUTTONS
 */
public class AutoControllerSelector {

  /**
   * Further items take lower precedence.
   */
  ArrayList<ControllerBase> controllers = new ArrayList<>();
  ArrayList<Integer> ports = new ArrayList<>();
  ControllerBase ghostController;
  Controllers controllersClass;

  public AutoControllerSelector(Controllers controllersClass) {
    this.ghostController = controllersClass.ghostController;
    this.controllersClass = controllersClass;
  }

  /**
   * <h4>NEVER ATTEMPT TO USE THE BUTTONS FROM THIS CONTROLLER</h4>
   * <p>
   * Using any .get() method from this controller will cause a crash within 2
   * minutes
   * due to <b>extreme</b> memory usage.
   * <p>
   * Recommended to treat as if this method is private, but it is public for
   * potential future use.
   */
  public ControllerBase getController() {
    // System.out.println("Getting controller");
    if (controllers == null) {
      return ghostController;
    }

    for (int port : ports) {
      ControllerBase controller = controllersClass.getPort(port);
      if (controller == null) {
        continue;
      }
      if (!controller.isPluggedIn()) {
        continue;
      }
      if (controller.isJoysticksBeingTouched()) {
        return controller;
      } else {
        // System.out.println("Controller on port " + controller.port + " not plugged
        // in");
      }
    }
    return ghostController;
  }

  public AutoControllerSelector addController(Integer... port) {
    for (Integer i : port) {
      ports.add(i);
    }
    return this;
  }

  public double getLeftY() {
    return getController().getLeftY();
  }

  public double getRightY() {
    return getController().getRightY();
  }

  public double getLeftX() {
    return getController().getLeftX();
  }

  public double getRightX() {
    return getController().getRightX();
  }

  public double getLeftTrigger() {
    return getController().getLeftTrigger();
  }

  public double getRightTrigger() {
    return getController().getRightTrigger();
  }
}
