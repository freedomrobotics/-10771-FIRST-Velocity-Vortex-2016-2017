package org.fhs.robotics.ftcteam10771.lepamplemousse.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Variables;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.ReturnValues;

import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Initial reset values. Should include the default/starting values loaded
 * from the config or other places and set all variables to such.
 */
public class StartValues {

    Variables variables = null;

    Map<String, Settings> objects = new HashMap<String, Settings>();

    Settings flags = new Settings();

    Telemetry telemetry;

    /**
     * The contrusctor for StartValues
     *
     * @param variables The Variables configuration object
     */
    public StartValues(Variables variables, Telemetry telemtry) {
        this.variables = variables;
        this.telemetry = telemtry;
    }

    /**
     * Initializes all of the starting values from the Variables config
     *
     * @return Whether successfully initialized or now
     */
    public ReturnValues initialize() {
        //Retrieve the iterator and the entry associated with the iteration
        for (Object o : variables.getEntrySet()) {
            Map.Entry settingObject = (Map.Entry) o;
            //Check to see if it's a flag or a configuration object
            Pattern p = Pattern.compile("[\\{\\}]");
            Matcher m = p.matcher(settingObject.getValue().toString());
            //If it's a flag, put it in the flag map
            if (!m.find()) {
                flags.addToMap(settingObject.getKey().toString(), settingObject.getValue());
                //and continue the loop
                continue;
            }
            //otherwise run it through the iterations and stor to the configuration objects
            objects.put(settingObject.getKey().toString(), (Settings) iterateOverValues(settingObject.getValue()));
        }
        return ReturnValues.SUCCESS;
    }

    /**
     * It's a repetative function that calls to itself in order to construct a Settings object
     *
     * @param object The object to check as a Map object to convert to settings
     * @return An object of either the settings (which has to be casted) or the origial object pased.
     */
    private Object iterateOverValues(Object object) {
        //Begin assembling the settings
        Settings settings = new Settings();
        //Regular Expressions to determine if the Object is a Map
        Pattern p = Pattern.compile("[\\{\\}]");
        Matcher m = p.matcher(object.toString());
        if (m.find()) {
            //Iterate over the Map if it is
            for (Object o : ((Map) object).entrySet()) {
                Map.Entry valuesObject = (Map.Entry) o;
                //And continue the iteration
                settings.addToMap(valuesObject.getKey().toString(), iterateOverValues(valuesObject.getValue()));
            }
            return settings;
        }
        //Otherwise return the object back
        return object;
    }

    /**
     * Retrieves the settings object of a supersetting in the config
     *
     * @param name The name of the setting
     * @return The setting object
     */
    public Settings settings(String name) {
        if (objects.containsKey(name)) {
            return objects.get(name);
        }
        return null;
    }

    /**
     * Retrieves the settings object of global flags in the config
     *
     * @return The Settings object of the global flags
     */
    public Settings flags() {
        return flags;
    }

    /**
     * The settings object which contains a HashMap and various functions to quickly retrieve settings
     */
    public class Settings {
        Map<String, Object> data = new HashMap<String, Object>();

        /**
         * Default constructor for the settings object which contains a HashMap and various functions to quickly retrieve settings
         */
        public Settings() {

        }

        /**
         * Cosntructor that copies an existing map to the Settings Object
         *
         * @param map The map to copy
         */
        public Settings(Map<String, Object> map) {
            if (map != null) {
                data.putAll(map);
            }
        }

        /**
         * Adds a value to the Settings object
         *
         * @param key    The key associated with the object
         * @param object The value to store
         */
        public void addToMap(String key, Object object) {
            data.remove(key);
            if (key != null) {
                data.put(key, object);
            }
        }

        /**
         * Copies an existing map to the Settings Object
         *
         * @param map The map to copy
         */
        public void addToMap(Map<String, Object> map) {
            if (map != null) {
                data.putAll(map);
            }
        }

        /**
         * Retrieves the object associated with the key
         *
         * @param key The key to retrieve the object from
         * @return The object associated with the key
         */
        public Object get(String key) {
            if (valueExists(key)) {
                return data.get(key);
            }
            return null;
        }

        /**
         * Removes the object associated with the key
         *
         * @param key The key to remove the object association
         */
        public void remove(String key) {
            data.remove(key);
        }

        /**
         * Checks to see if a key exists
         *
         * @param key The key to check for
         * @return A boolean stating whether the key exists or not
         */
        public boolean valueExists(String key) {
            return data.containsKey(key);
        }

        /**
         * Retrieves the object associated with the key
         *
         * @param key The key to retrieve the object from
         * @return The object associated with the key
         */
        public Object getObject(String key) {
            return get(key);
        }

        /**
         * Retrieves the boolean associated with the key
         *
         * @param key The key to retrieve the boolean from
         * @return The boolean associated with the key
         */
        public boolean getBool(String key) {
            return get(key).toString().equals("true");
        }

        /**
         * Retrieves the string associated with the key
         *
         * @param key The key to retrieve the string from
         * @return The string associated with the key
         */
        public String getString(String key) {
            return get(key).toString();
        }

        /**
         * Retrieves the integer associated with the key
         *
         * @param key The key to retrieve the integer from
         * @return The integer associated with the key
         */
        public int getInt(String key) {
            if (valueExists(key)) {
                Pattern p = Pattern.compile("[a-zA-Z]");
                Matcher m = p.matcher(data.get(key).toString());
                if (m.find()) {
                    return 0;
                }
                if (data.get(key).toString().contains(".")) {
                    return ((Double) data.get(key)).intValue();
                }
                return (Integer) data.get(key);
            }
            return 0;
        }

        /**
         * Retrieves the float associated with the key
         *
         * @param key The key to retrieve the float from
         * @return The float associated with the key
         */
        public float getFloat(String key) {
            if (valueExists(key)) {
                Pattern p = Pattern.compile("[a-zA-Z]");
                Matcher m = p.matcher(data.get(key).toString());
                if (m.find()) {
                    return 0;
                }
                if (!data.get(key).toString().contains(".")) {
                    return ((Integer) data.get(key)).floatValue();
                }
                return ((Double) data.get(key)).floatValue();
            }
            return 0;
        }

        /**
         * Retrieves the Settings object associated with the key
         *
         * @param key The key to retrieve the Settings object from
         * @return The Settings object associated with the key
         */
        public Settings getSettings(String key) {
            if (valueExists(key)) {
                Pattern p = Pattern.compile("[\\{\\}]");
                Matcher m = p.matcher(data.get(key).toString());
                if (m.find()) {
                    return (Settings) data.get(key);
                }
            }
            return null;
        }

        @Override
        public String toString() {
            return data.toString();
        }
    }
}
