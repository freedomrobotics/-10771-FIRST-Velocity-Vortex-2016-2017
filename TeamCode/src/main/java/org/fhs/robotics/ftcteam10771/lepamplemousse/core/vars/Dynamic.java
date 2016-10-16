package org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars;

import android.content.res.AssetManager;

/**
 * Variables that change throughout operation.
 * Some examples are Time and Distance.
 */
public class Dynamic {
    // TODO: 11/27/2015 Improve
    public static boolean reset = false;

    /**
     * Flag for whether the config file folder exists or not
     */
    // TODO: 11/27/2015 Move to dynamic
    public static boolean configDirExists = true;

    /**
     * AssetManager Reference
     */
    public static AssetManager globalAssets = null;

}
