// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class AllianceHelpers {
  private AllianceHelpers() {
  }

  // System (FMS) and store variable in network tables as hex, so it can be
  // displayed on the Elastic dashboard using a Color View widget
  public static void setAllianceColor() {
    NetworkTableEntry allianceColorHexEntry = NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("AllianceColorHex");

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // System.out.println("Red Alliance");
        allianceColorHexEntry.setString("#FF0000");
      } else if (ally.get() == Alliance.Blue) {
        // System.out.println("Blue Alliance");
        allianceColorHexEntry.setString("#0000FF");
      }
    } else {
      System.out.println("No Alliance Color Yet");
      allianceColorHexEntry.setString("#808080"); // Default to gray if no alliance
    }
  }

  public static String getAllianceColor() {
    return NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("AllianceColorHex").getString("#808080");
  }

  public static Boolean isInactiveFirst() {
    String gameData;
    String currentAllianceColorHex = getAllianceColor();// #FF0000 red, #0000FF Blue
    char currentAllianceColorChar;

    gameData = DriverStation.getGameSpecificMessage();
    if (currentAllianceColorHex.equals("#FF0000")) {
      currentAllianceColorChar = 'R';
    } else if (currentAllianceColorHex.equals("#0000FF")) {
      currentAllianceColorChar = 'B';
    } else {
      currentAllianceColorChar = 'G';
    }

    if (gameData.length() > 0 && currentAllianceColorChar != 'G') {
      char inactiveFirstChar = gameData.charAt(0);
      return currentAllianceColorChar == inactiveFirstChar;
    } else {
      return null;
    }
  }

  public static void updateHubStatus(Boolean isInactiveFirst) {
    // TELEOP TRANSITION  10 Seconds  2:30 – 2:20  (both alliances active)
    // SHIFT 1            25 Seconds  2:20 – 1:45
    // SHIFT 2            25 Seconds  1:45 – 1:20
    // SHIFT 3            25 Seconds  1:20 – 0:55
    // SHIFT 4            25 Seconds  0:55 – 0:30
    // END GAME           30 Seconds  0:30 – 0:00  (hub inactive)

    double timeRemaining = DriverStation.getMatchTime();
    boolean isHubActive;
    double timeToInactive = 0;
    double timeToActive = 0;

    boolean isInTransition = timeRemaining > 130;
    boolean isInShift1     = timeRemaining > 105 && timeRemaining <= 130;
    boolean isInShift2     = timeRemaining > 80  && timeRemaining <= 105;
    boolean isInShift3     = timeRemaining > 55  && timeRemaining <= 80;
    boolean isInShift4     = timeRemaining > 30  && timeRemaining <= 55;

    if (isInactiveFirst == null) {
      // Alliance assignment not yet known — assume hub active
      isHubActive = true;
    } else if (isInactiveFirst) {
      // This alliance is inactive in shifts 1 & 3, active in shifts 2 & 4
      if (isInTransition) {
        isHubActive = true;
        timeToInactive = timeRemaining - 130; // shift 1 starts, hub goes inactive
      } else if (isInShift1) {
        isHubActive = false;
        timeToActive = timeRemaining - 105;   // shift 2 starts, hub goes active
      } else if (isInShift2) {
        isHubActive = true;
        timeToInactive = timeRemaining - 80;  // shift 3 starts, hub goes inactive
      } else if (isInShift3) {
        isHubActive = false;
        timeToActive = timeRemaining - 55;    // shift 4 starts, hub goes active
      } else if (isInShift4) {
        isHubActive = true;
        timeToInactive = timeRemaining - 30;  // end game starts, hub goes inactive
      } else {
        isHubActive = false; // end game
      }
    } else {
      // This alliance is active in shifts 1 & 3, inactive in shifts 2 & 4
      if (isInTransition || isInShift1) {
        isHubActive = true;
        timeToInactive = timeRemaining - 105; // shift 2 starts, hub goes inactive
      } else if (isInShift2) {
        isHubActive = false;
        timeToActive = timeRemaining - 80;    // shift 3 starts, hub goes active
      } else if (isInShift3) {
        isHubActive = true;
        timeToInactive = timeRemaining - 55;  // shift 4 starts, hub goes inactive
      } else if (isInShift4) {
        isHubActive = false;
        timeToActive = timeRemaining - 30;    // end game (still inactive)
      } else {
        isHubActive = false; // end game
      }
    }

    NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("hubStatus").setBoolean(isHubActive);
    NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("timeToActive").setDouble(timeToActive);
    NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("timeToInactive").setDouble(timeToInactive);
  }
}
