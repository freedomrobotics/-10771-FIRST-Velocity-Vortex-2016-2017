package org.fhs.robotics.ftcteam10771.lepamplemousse.core;

/**
 * Created by Adam Li on 1/11/2017.
 */

public enum Alliance {
    UNKNOWN ("unknown", "unknown"),
    RED_ALLIANCE ("red", "general"),
    RED_ALLIANCE_INSIDE ("red", "inside"),
    RED_ALLIANCE_OUTSIDE ("red", "outside"),
    BLUE_ALLIANCE ("blue", "general"),
    BLUE_ALLIANCE_INSIDE ("blue", "inside"),
    BLUE_ALLIANCE_OUTSIDE ("blue", "outside");


    private final String alliance;
    private final String position;

    private Alliance(String alliance, String position) {
        this.alliance = alliance;
        this.position = position;
    }

    public String getAlliance() {
        // (otherName == null) check is not needed because name.equals(null) returns false
        return alliance;
    }

    public String getPosition() {
        // (otherName == null) check is not needed because name.equals(null) returns false
        return position;
    }

    public String toString() {
        return this.name();
    }

    public boolean equalsAlliance(Alliance alliance){
        return alliance.getAlliance().equalsIgnoreCase(this.alliance);
    }

    public boolean equalsPosition(Alliance alliance) {
        return alliance.getPosition().equalsIgnoreCase(this.position);
    }
}
