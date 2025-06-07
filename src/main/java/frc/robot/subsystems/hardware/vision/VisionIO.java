package frc.robot.subsystems.hardware.vision;

import java.util.Optional;

public interface VisionIO {
  Optional<Double> getTX(int targetID);
  Optional<Double> getTY(int targetID);
}
