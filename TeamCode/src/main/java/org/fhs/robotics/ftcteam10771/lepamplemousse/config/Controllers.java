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
 * Configuration class for controllers
 */
public class Controllers extends ConfigOld {

    // Filename with the suffix
    private String fileName = Static.configControlFileName + Static.configFileSufffix;
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
    public Controllers(Telemetry telemetry) {
        //Calling the superclass' (ConfigOld.class) constructor.
        super(telemetry);
        this.telemetry = telemetry;
        init();
    }

    /**
     * Non-debuggable constructor
     */
    public Controllers() {
        //Calling the superclass' (ConfigOld.class) constructor.
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
                    telemetry.addData("ControlConFile", "exists");
                } else {
                    telemetry.addData("ControlConFile", "does not exist... creating with defaults...");
                    if (createDefaults(fileName) && writable) {
                        telemetry.addData("ControlConFile", "created successfully");
                    } else {
                        telemetry.addData("ControlConFile", "failed to create... using defaults");
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
                telemetry.addData("ControlConFile", "using defaults");
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
                    telemetry.addData("CreatedControlConFile", "default created");
                    fileExists = true;
                } else {
                    telemetry.addData("CreatedControlConFile", "failed to create default");
                }
            }
        } else {
            try {
                FileWriter configWrite = new FileWriter(configFile);
                configWrite.write(yaml.dump(data));
                configWrite.flush();
                configWrite.close();
                if (Static.Debug && telemetry != null)
                    telemetry.addData("CreatedControlConFile", "created successfully");
                result = fileExists = true;
            } catch (IOException e) {
                e.printStackTrace();
                if (Static.Debug && telemetry != null)
                    telemetry.addData("CreatedControlConFile", "failed to create");
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
                        telemetry.addData("LoadedControlConFile", "default loaded");
                    return ReturnValues.SUCCESS;
                } else {
                    if (Static.Debug && telemetry != null)
                        telemetry.addData("LoadedControlConFile", "failed to verify");
                    return ReturnValues.UNABLE_TO_VERIFY;
                }
            } else {
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadedControlConFile", "failed to load");
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
                    telemetry.addData("LoadControlConFile", "default selected");
            } catch (IOException e) {
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadControlConFile", "failed to load default");
                e.printStackTrace();
                return false;
            }
        } else if (fileExists) {
            try {
                config = new FileInputStream(configFile);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadControlConfFile", "file failed to load");
                return false;
            }
            if (Static.Debug && telemetry != null)
                telemetry.addData("LoadControlConfFile", "file selected");
        } else {
            return false;
        }
        if (config != null) {
            data = (Map<String, Object>) yaml.load(config);
            if (Static.Debug && telemetry != null)
                telemetry.addData("LoadControlConfFile", "failed to load");
            if (data != null) {
                if (Static.Debug && telemetry != null)
                    telemetry.addData("LoadControlConfFile", "loaded");
                return true;
            }
        }
        return false;
    }

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

    //region GETTERS, SETTERS, AND CHECKERS (NEEDS CHECKING)
    //TODO: 12/16/2015 Add setters

    /**
     * Checks the existance of a gamepad
     *
     * @param id The name of hte overarching device
     * @return a boolean of the device's existence
     */
    public boolean gamepadExists(Integer id) {
        return data.containsKey("gamepad" + id);
    }

    /**
     * Getter for the controller
     *
     * @param id The id of the gamepad to retrieve
     * @return The map of the controller
     */
    public Map getGamepad(Integer id) {
        if (gamepadExists(id)) {
            return (Map) data.get("gamepad" + id);
        }
        return null;
    }

    /**
     * Getter for an input on a controller given the id
     *
     * @param id    Id of the controller
     * @param input The specified input as string
     * @return The mapping of the input
     */
    public Map getInput(Integer id, String input) {
        Map controller;
        if ((controller = getGamepad(id)) != null) {
            return (Map) controller.get(input);
        }
        return null;
    }

    /**
     * Checker for the input's existence in the mapping
     *
     * @param id    Id of the controller
     * @param input The input to be checked
     * @return whether or not the input exists
     */
    public boolean inputExists(Integer id, String input) {
        Map controller;
        if ((controller = getGamepad(id)) != null) {
            return controller.containsKey(input);
        }
        return false;
    }

    /**
     * Generic checker for setting
     *
     * @param id      id of the controller
     * @param input   The specified input
     * @param setting The setting to be checked as string
     * @return Boolean value of setting key
     */
    public boolean getBoolean(Integer id, String input, String setting) {
        Map controller;
        if ((controller = getInput(id, input)) != null) {
            if (controller.get(setting) != null) {
                return controller.get(setting).equals(true);
            }
        }
        return false;
    }

    /**
     * Check if digital is enabled
     *
     * @param id    Id of the controller
     * @param input the input to check
     * @return Whether or not the input is digital
     */
    public boolean digitalEnabled(Integer id, String input) {
        return getBoolean(id, input, "digital");
    }

    /**
     * Check if the input is inverted
     *
     * @param id    Id of the controller
     * @param input the input to check
     * @return Whether or not the input is inverted
     */
    public boolean invertedEnabled(Integer id, String input) {
        return getBoolean(id, input, "inverted");
    }

    /**
     * Retrieve the function of the input
     *
     * @param id    Id of the controller
     * @param input the input to check
     * @return The function of the input
     */
    public String getFunction(Integer id, String input) {
        Map controller;
        if ((controller = getInput(id, input)) != null) {
            if (controller.get("function") != null) {
                return controller.get("function").toString();
            }
        }
        return null;
    }

    /**
     * Returns a set for the inputs under a specific gamepad
     *
     * @param id id of the controller
     * @return A set with an entryset of the nested map within a controller
     */
    public Set<Map.Entry<String, Object>> getEntrySet(Integer id) {
        Map controller;
        if ((controller = getGamepad(id)) != null) {
            return controller.entrySet();
        }
        return new HashMap<String, Object>().entrySet();
    }
    //endregion
}

