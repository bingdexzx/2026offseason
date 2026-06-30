package frc.robot.subsystems;

public enum WantedStates {
  START(new WantedStatesData(0.0, 0.0, 0.3, 0.0, 0.0, 0.0)),
  INTAKE(new WantedStatesData(0.0, 0.0, 0.0, 6.0, 3.0, 0.0));

  private final WantedStatesData data;

  private WantedStates(WantedStatesData data) {
    this.data = data;
  }
  ;
}
