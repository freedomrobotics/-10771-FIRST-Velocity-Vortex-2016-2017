package org.fhs.robotics.ftcteam10771.lepamplemousse.config;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Dynamic;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.ReturnValues;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * Abstract class to use to define what each other class should do
 * or to share a value across them such as the filepath (which is not right)
 */
public abstract class ConfigOld {

    /**
     * The location of the coded location of the config files
     */
    protected File configDirectory = new File(Environment.getExternalStorageDirectory().toString() + Static.configPath);
    protected boolean writable = false;
    // flag
    private boolean configDirCheck = false;

    /**
     * Debuggable Constructor
     *
     * @param telemetry Telemetry output for Debug
     */
    public ConfigOld(Telemetry telemetry) {
        writable = Environment.getExternalStorageDirectory().canWrite();
        // If this hasn't been done yet, or reset has been called, run the folder check
        if (!configDirCheck || Dynamic.reset) {
            Dynamic.configDirExists = true;
            //region Debug block
            // If debug is enabled, verbose output
            if (Static.Debug) {
                if (writable) {
                    telemetry.addData("FS-Write", "enabled");
                } else {
                    telemetry.addData("FS-Write", "not enabled");
                }
                if (configDirectory.exists()) {
                    telemetry.addData("ConfigDir", "exists");
                } else {
                    telemetry.addData("ConfigDir", "does not exist... creating...");
                    if (configDirectory.mkdirs()) {
                        telemetry.addData("ConfigDir", "created successfully");
                    } else {
                        telemetry.addData("ConfigDir", "failed to create... using defaults");
                        Dynamic.configDirExists = false;
                    }
                }
            }
            // endregion
            // Runs if debug is not enabled
            else dirCheck();
        }
        //Small flag to prevent debug spamming and prevent redundant checks
        configDirCheck = true;
    }

    /**
     * Non-debuggable constructor
     */
    public ConfigOld() {
        writable = Environment.getExternalStorageDirectory().canWrite();
        // If this hasn't been done yet, or reset has been called, run the folder check
        if (!configDirCheck || Dynamic.reset) {
            Dynamic.configDirExists = true;
            dirCheck();
        }
        //Small flag to prevent debug spamming and prevent redundant checks
        configDirCheck = true;
    }

    //Just to save lines
    private void dirCheck() {
        // Shortened logic to check and create the folder. If it can write and the folder
        // doesn't exist, it will try to create it, marking it as not existing if it fails.
        // If it doesn't exist and write can't happen, it's marked as not existing.
        if (!configDirectory.exists() && writable) {
            if (!configDirectory.mkdirs()) {
                Dynamic.configDirExists = false;
            }
        } else if (!configDirectory.exists()) {
            Dynamic.configDirExists = false;
        }
    }

    /**
     * Simple function to copy a default file to the config directory
     *
     * @param fileName The name of the config file to copy.
     * @return Whether or not the file was created successfully.
     */
    protected boolean createDefaults(String fileName) {
        try {
            File configFile = new File(configDirectory, fileName);
            if (!configFile.isFile() && !configFile.createNewFile()) {
                return false;
            }
            InputStream in = Dynamic.globalAssets.open(fileName);
            OutputStream out = new FileOutputStream(configFile, false);

            byte[] buffer = new byte[1024];
            int readlen;
            while ((readlen = in.read(buffer)) != -1) {
                out.write(buffer, 0, readlen);
            }
            in.close();
            out.flush();
            out.close();

            return true;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return false;
    }

    // TODO: 11/28/2015 Change some of the compound functions to use enumerations instead of assuming true

    /**
     * Identifies and locates the file and such.
     * Run it within the constructor
     */
    protected abstract void init();

    /**
     * Read the values from the configuration file.
     * Should run and pass the value from verify and load.
     *
     * @return True if file loaded; false if defaults loaded.
     */
    public abstract boolean read();

    /**
     * Creates a configuration using the default values.
     * If a configuration file exists, it will be replaced.
     * Will not load the values after.
     *
     * @return Creation state.
     */
    public boolean create() {
        return create(true, false).equals(ReturnValues.SUCCESS);
    }

    /**
     * Creates a configuration using the default or stored values.
     * If a configuration file exists, it will be replaced.
     * Will not load the values after.
     *
     * @param useDefaults Whether or not to create the defaults
     * @return Creation state.
     */
    public boolean create(boolean useDefaults) {
        return create(useDefaults, false).equals(ReturnValues.SUCCESS);
    }

    /**
     * Creates a configuration using the default or stored values.
     * If a configuration file exists, it will be replaced.
     *
     * @param useDefaults Whether or not to load the defaults
     * @param loadAfter   Whether or not to verify and load the file after replacing
     * @return Success Value based on the ReturnValues enumeration
     */
    public abstract ReturnValues create(boolean useDefaults, boolean loadAfter);

    /**
     * Verify the retrieved values. True if verified, false if not.
     * Will replace bad values with defaults
     *
     * @return Verify state
     */
    public abstract boolean verify();

    /**
     * Loads the file.
     *
     * @return Loaded successfully or not
     */
    public boolean load() {
        return load(false);
    }

    /**
     * Loads values.
     *
     * @param loadDefault Whether to load from defaults or file.
     * @return Loaded successfully or not
     */
    public abstract boolean load(boolean loadDefault);

    /**
     * Retrieves the object associated with the string
     *
     * @param key The string to search and retrieve the object from
     * @return The object mapped to the string
     */
    public abstract Object retrieve(String key);

    /**
     * Stores or replaces the object associated with the string
     *
     * @param key    The string to search and store the object to
     * @param object The Object to store
     */
    public abstract boolean store(String key, Object object);
}
