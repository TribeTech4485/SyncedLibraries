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
  ControllerBase ghostController;

  public AutoControllerSelector(ControllerBase ghostController) {
    this.ghostController = ghostController;
  }

  public ControllerBase getController() {
    if (controllers == null) {
      return ghostController;
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
    return ghostController;
  }

  public void addController(ControllerBase... controller) {
    for (ControllerBase c : controller) {
      controllers.add(c);
    }
  }
}
