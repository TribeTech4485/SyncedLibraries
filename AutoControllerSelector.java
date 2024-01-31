package frc.robot.SyncedLibraries;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.SyncedLibraries.Controllers.ControllerBase;

/**
 * This class is used to select the controller that is actively being used.
 * The first controller in the list that is being touched is the one that is
 * used.
 * If no controllers are being touched, the nullController is used.
 * <p>
 * @deprecated Use {@link Controllers} instead
 */
@Deprecated
public class AutoControllerSelector {

  /**
   * Further items take lower precedence.
   */
  ArrayList<ControllerBase> controllers = new ArrayList<>();
  ArrayList<Integer> ports = new ArrayList<>();
  ControllerBase ghostController;
  Controllers controllersClass;
  MotorControllerGroup motorCotrollerGroup;

  public AutoControllerSelector(ControllerBase ghostController, Controllers controllersClass) {
    this.ghostController = ghostController;
    this.controllersClass = controllersClass;
  }

  public ControllerBase getController() {
    System.out.println("Getting controller");
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
      if (controller.isBeingTouched()) {
        return controller;
      } else {
        System.out.println("Controller on port " + controller.port + " not plugged in");
      }
    }
    return ghostController;
  }

  public void addController(Integer... port) {
    for (Integer i : port) {
      ports.add(i);
    }
  }
}
