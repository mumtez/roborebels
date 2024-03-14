package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LVMaxSonarEZ implements DistanceSensor {

  private final AnalogInput sensor;

  public LVMaxSonarEZ(AnalogInput analogInput) {
    this.sensor = analogInput;
  }

  @Override
  public double getDistance(DistanceUnit distanceUnit) {
    // Assumes sensor is running at 3.3v max voltage (plugged directly to REV Hub)
    double ultrasonicLevel = this.sensor.getVoltage() * 1024.0 / sensor.getMaxVoltage();
    return distanceUnit.fromInches(ultrasonicLevel);
  }

  @Override
  public Manufacturer getManufacturer() {
    return sensor.getManufacturer();
  }

  @Override
  public String getDeviceName() {
    return sensor.getDeviceName();
  }

  @Override
  public String getConnectionInfo() {
    return sensor.getConnectionInfo();
  }

  @Override
  public int getVersion() {
    return 0;
  }

  @Override
  public void resetDeviceConfigurationForOpMode() {
    sensor.resetDeviceConfigurationForOpMode();
  }

  @Override
  public void close() {
    sensor.close();
  }
}