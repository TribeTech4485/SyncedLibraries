package frc.robot.SyncedLibraries;

import java.util.ArrayList;

import frc.robot.SyncedLibraries.Controllers.ControllerBase;

/**
 * This class is used to select the controller that is actively being used.
 * The first controller in the list that is being touched is the one that is used.
 * If no controllers are being touched, the nullController is used.
 */
public class AutoControllerSelector {

  /**
   * Further items take lower precedence.
   */
  ArrayList<ControllerBase> controllers = new ArrayList<>();
  ControllerBase nullController;

  public AutoControllerSelector(ControllerBase nullController) {
    this.nullController = nullController;
  }

  public ControllerBase getController() {
    if (controllers == null) {
      return nullController;
    }

    for (ControllerBase controller : controllers) {
      if (controller == null) {
        continue;
      }
      if (!controller.isPluggedIn()) {
        continue;
      }
      if (controller.isBeingTouched()) {
        System.out.println(getClass().getName() + " returned controller on port " + controller.port);
        return controller;
      }
    }
    return nullController;
  }

  public void addController(ControllerBase... controller) {
    for (ControllerBase c : controller) {
      controllers.add(c);
    }
  }
}
