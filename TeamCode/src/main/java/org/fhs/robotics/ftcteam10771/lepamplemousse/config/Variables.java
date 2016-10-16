package org.fhs.robotics.ftcteam10771.lepamplemousse.config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Dynamic;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.ReturnValues;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.yaml.snakeyaml.Yaml;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * Configuration fie accessor for a Variables Config File
 * todo simplify and make some more merges with config.java
 * todo improve
 */
public class Variables extends Config {

    // Filename with the suffix
    private String fileName = Static.configVarFileName + Static.configFileSufffix;
    // The actual file
    private File configFile = new File(configDirectory, fileName);
    // Does the file exist or no.
    private boolean fileExists = true;
    // variable for telemetry
    private Telemetry telemetry = null;
    // The YAML object.
    private Yaml yaml = new Yaml();
    // An object to string map.
    private Map<String, Object> data = null;

    /**
     * Debuggable Constructor
     *
     * @param telemetry Telemetry output for Debug
     */
    public Variables(Telemetry telemetry) {
        //Calling the superclass' (Config.class) constructor.
        super(telemetry);
        this.telemetry = telemetry;
        init();
    }

    /**
     * Non-debuggable constructor
     */
    public Variables() {
        //Calling the superclass' (Config.class) constructor.
        super();
        init();
    }

    /**
     * Identifies and locates the file and such.
     * Runs within the constructor
     */
    @Override
    protected void init() {
        // saves code by not typing true a billion times.
        fileExists = true;
        // if the configuration directory even exists, then check of the config exists
        if (Dynamic.configDirExists) {
            //region Debug block
            // If debug is enabled, verbose output
            if (Static.Debug && telemetry != null) {
                if (configFile.exists()) {
                    telemetry.addData("VarConfFile", "exists");
                } else {
                    telemetry.addData("VarConfFile", "does not exist... creating with defaults...");
                    if (createDefaults(fileName) && writable) {
                        telemetry.addData("VarConfFile", "created successfully");
                    } else {
                        telemetry.addData("VarConfFile", "failed to create... using defaults");
                        fileExists = false;
                    }
                }
            }
            //endregion
            else {
                // shortened logic
                if (!configFile.exists() && writable) {
                    if (!createDefaults(fileName)) {
                        fileExists = false;
                    }
                } else if (!configFile.exists()) {
                    fileExists = false;
                }
            }
        }
        // If config doesn't exists, obviously the file doesn't and the system can't be
        // writable, so just say it doesn't exist
        else {
            if (Static.Debug && telemetry != null)
                telemetry.addData("VarConfFile", "using defaults");
            fileExists = false;
        }
    }

    /**
     * Read the values from the configuration file.
     * Runs and passes the value from verify and load.
     *
     * @return True if file loaded; false if defaults loaded.
     */
    @Override
    public boolean read() {
        if (load()) {
            if (!verify()) {
                load(true);
                return false;
            }
            return true;
        }
        load(true);
        return false;
    }

    /**
     * Creates the configuration using the default or stored values.
     * If the configuration file exists, it will be replaced.
     *
     * @param useDefaults Whether or not to load the defaults
     * @param loadAfter   Whether or not to load the file and verify after replacing
     * @return Success Value based on the ReturnValues enumeration
     */
    @Override
    public ReturnValues create(boolean useDefaults, boolean loadAfter) {
        boolean result;
        if (useDefaults) {
            result = createDefaults(fileName);
            if (Static.Debug && telemetry != null) {
                if (result) {
                    telemetry.addData("CreatedVarConfFile", "default created");
                    fileExists = true;
                } else {
                    telemetry.addData("CreatedVarConfFile", "failed to create default");
                }
            }
        } else {
            try {
                FileWriter configWrite = new FileWriter(configFile);
                configWrite.write(yaml.dump(data));
                configWrite.flush();
                configWrite.close();
                if (Static.Debug && telemetry != null)
                    telemetry.addData("CreatedVarConfFile", "created successfully");
                result = fileExists = true;
            } catch (IOException e) {
                e.printStackTrace();
                if (Static.Debug && telemetry != null)
                    telemetry.addData("CreatedVarConfFile", "failed to create");
                result = false;
            }
        }
        if (!loadAfter) {
            return (result) ? ReturnValues.SUCCESS : ReturnValues.FAIL;     // Look up Ternary operator if you don't understand
        } else if (!result) {
            return ReturnValues.FAIL;
        } else {
            if (load()) {
                if (verify()) {
                    if (Static.Debug && telemetry != null)
                        telemetry.addData("LoadedVarConFile", "default loaded");
                    return ReturnValues.SUCCESS;
                } else {
                    if (Static.Debug && telemetry != null)
                        telemetry.addData("LoadedVarConFile", "failed to verify");
                    return ReturnValues.UNABLE_TO_VERIFY;
                }
            } else {
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadedVarConFile", "failed to load");
                return ReturnValues.UNABLE_TO_LOAD;
            }
        }
    }

    /**
     * Verify the loaded values. True if verified, false if not.
     * Will replace bad values with defaults
     *
     * @return Verify state
     */
    @Override
    public boolean verify() {
        // TODO: 11/28/2015 Add a range checker
        return true;
    }

    /**
     * Loads values.
     *
     * @param loadDefault Whether to load from defaults or file.
     * @return Loaded successfully or not
     */
    @Override
    public boolean load(boolean loadDefault) {
        InputStream config;
        data = null;
        if (loadDefault) {
            try {
                config = Dynamic.globalAssets.open(fileName);
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadVarConfFile", "default selected");
            } catch (IOException e) {
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadVarConfFile", "failed to load default");
                e.printStackTrace();
                return false;
            }
        } else if (fileExists) {
            try {
                config = new FileInputStream(configFile);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadVarConfFile", "file failed to load");
                return false;
            }
            if (Static.Debug && telemetry != null)
                telemetry.addData("LoadVarConfFile", "file selected");
        } else {
            return false;
        }
        if (config != null) {
            data = (Map<String, Object>) yaml.load(config);
            if (Static.Debug && telemetry != null)
                telemetry.addData("LoadVarConfFile", "failed to load");
            if (data != null) {
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadVarConfFile", "loaded");
                return true;
            }
        }
        return false;
    }

    //region Getters and Setters

    /**
     * Retrieves the object associated with the string
     *
     * @param key The string to search and retrieve the object from
     * @return The object mapped to the string
     */
    @Override
    public Object retrieve(String key) {
        if (data != null) {
            return data.get(key);
        }
        return null;
    }

    /**
     * Stores or replaces the object associated with the string
     *
     * @param key    The string to search and store the object to
     * @param object The Object to store
     */
    @Override
    public boolean store(String key, Object object) {
        if (data != null) {
            data.put(key, object);
        }
        return false;
    }
    //********************************NEW Untested Code**************************

    /**
     * Gets whether or not setting type exists
     *
     * @return Whether or not the setting type exists.
     */
    public boolean settingExists(String setting) {
        return data.containsKey(setting);
    }

    /**
     * Method for determining existence of a setting
     *
     * @param settingType the category of a setting
     * @param setting     the specific setting
     * @return whether or not it exists
     */
    public boolean settingExists(String settingType, String setting) {
        if (settingExists(settingType)) {
            return ((Map) data.get(settingType)).containsKey(setting);
        } else return false;
    }

    //checks if a device type is enabled
    public boolean enabled(String type) {
        if (((Map) data.get(type)).get("enabled").equals(true)) return true;
        else return false;
    }

    //checks if device(motor) is reversed
    //TODO: 12/14/2015 Improve
    public boolean reversed(String type, String device) {
        if (settingExists(type, device)) {
            if (((Map) ((Map) data.get(type)).get(device)).get("reversed").equals(true)) {
                return true;
            } else {
                return false;
            }
        } else return false;
    }

    //Method for obtaining offsets for a device
    public Integer obtainOffset(String type, String setting) {
        if (settingExists(type, setting)) {
            return Integer.parseInt(((Map) ((Map) data.get(type)).get(setting)).get("offset").toString());
        } else return null;
    }

    //Method for any quantity setting(especially winch)
    public Integer settingQuantity(String type, String quantity) {
        if (settingExists(type, quantity)) {
            return Integer.parseInt(((Map) data.get(type)).get(quantity).toString());
        } else return null;
    }

    //Overloaded quantity method
    public Integer settingQuantity(String type, String setting, String quantity) {
        if (settingExists(type, setting)) {
            return Integer.parseInt(((Map) ((Map) data.get(type)).get(setting)).get(quantity).toString());
        } else return null;
    }

    //
    public String deviceSettingKey(String type, String device) {
        if (settingExists(type, device)) {
            return (((Map) ((Map) data.get(type)).get(device)).get("map_name").toString());
        } else return null;
    }

    //***************************END New Untested Code*************************

    /**
     * Sets whether or not the drivetrain object in the configuration file exists.
     * Will clear the previous drivetrain object.
     *
     * @param exists The existance of the drivetrain object in the configuration file.
     */
    public void setDrivetrainExists(boolean exists) {
        data.remove("drivetrain");
        if (exists) {
            data.put("drivetrain", new Object());
        }
    }

    /**
     * Gets the value under the drivetrain object
     *
     * @param tag_name The name of the object being called on within drivetrain.
     * @return A java object that can be casted to the appropriate value.
     */
    public Object getDrivetrainObject(String tag_name) {
        if (settingExists("drivetrain")) {
            return ((Map) data.get("drivetrain")).get(tag_name);
        }
        return null;
    }


    // TODO: 12/19/2015 Match for the correct class.
    // TODO: 12/19/2015 Merge common functions into config.java

    /**
     * Returns a set for the inputs under a specific gamepad
     *
     * @return A set with an entryset of the nested map within a controller
     */
    public Set<Map.Entry<String, Object>> getEntrySet() {
        if (data != null) {
            return data.entrySet();
        }
        return new HashMap<String, Object>().entrySet();
    }

    //endregion
}
