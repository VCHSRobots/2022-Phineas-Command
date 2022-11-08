// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class distanceRPMPoint {
    public double distance = 0;
    public double topRPM = 0;
    public double botRPM = 0;

    public distanceRPMPoint(double distance, double topRPM, double botRPM){
        this.distance = distance;
        this.topRPM = topRPM;
        this.botRPM = botRPM;
    }
}