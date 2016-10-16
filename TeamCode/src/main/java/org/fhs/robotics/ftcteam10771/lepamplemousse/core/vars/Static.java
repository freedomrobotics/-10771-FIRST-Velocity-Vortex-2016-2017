package org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * These are variables that have to be hardcoded. They can't be changed via a config file.
 * Well, in the future, they could be put in a preferences file... but for now, this is good enough
 */
public class Static {
    //region Static Variables/Configurations for the config package

    // This is assuming the SD card path is already given.
    // Because this is going to be appended, add "/" in front
    /**
     * Configuration Directory Path relative to /sdcard
     */
    public static final String configPath = "/GrapefruitConfigAtomTest";

    /**
     * Configuration File Suffixes.
     */
    public static final String configFileSufffix = ".yml";

    /**
     * Filename for the config file that config.Variables loads
     */
    public static final String configVarFileName = "settings";

    /**
     * Filename for the config file that config.Components loads
     */
    public static final String configCompFileName = "config";

    /**
     * Filename for the config file that config.Controllers loads
     */
    public static final String configControlFileName = "keymapping";

    /**
      * A constant to convert nanoseconds to seconds.
      */
    public static final float nanoSecondsToSeconds = 1.0f / 1000000000.0f;
    //endregion

    //region Maps

    public static final String mapsPath = "/GrapefruitMapsAtomTest";

    public static final String mapsFileSuffix = ".yml";

    public static final String mapsFieldFileName = "fieldmap";

    public static final String mapsRedMontFileName = "redmountain";

    public static final String mapsBlueMontFileName = "bluemountain";

    public static final List<String> mapNames = new ArrayList<String>(Arrays.asList(mapsFieldFileName, mapsRedMontFileName, mapsBlueMontFileName));

    //endregion

    //region Static Variables/Configurations globally

    // Will be moved to a config file later.
    public static final boolean Debug = true;

    //endregion
}