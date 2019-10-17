## Introduction

The march_rqt_launch_menu is an rqt plugin that makes it easy to select how you want to run the march iv exoskeleton.

It is build with extensibility in mind and therefore it should be incredibly easy to add more launch parameters to the options

## Features
 - Parse Qt forms and upload these settings to the ros parameter server.
 - Basic sanity checks on which run configuration is selected.
 - Add new launch options without adding more code.
 - Ability to change default settings in the ui.
 
## Implementation
The goal of this launch menu was to make it as extensible as possible without having to add more code.


## Tutorials
### Getting started
It is highly advised to install Qt creator as it makes editing the .ui files very easy.
```
sudo apt-get install build-essential
sudo apt-get install qtcreator
sudo apt-get install qt5-default
```
In Qt-creator navigate to Design and press `Ctrl + O` to open the `launch_menu.ui` file.

You can now start editing the .ui file!

### Adding a new QCheckBox
You can add a new `QCheckBox` whenever you want to add a new option to the launch menu. It does not matter to which `QGroupBox` this checkbox belongs or where in the layout it's placed.

 1. Drag a Check Box button from the left banner to the `QGroupBox` where you want it to be located.
 2. Double-click the `QCheckBox` to change the text.
 3. In the property editor in the bottom right, locate the `toolTip` and the `statusTip`. 
 4. Change the `toolTip` to a description that fits your setting.
 5. Change the `statusTip` to the name of the parameter that will be uploaded to the parameter server starting with a `/` (e.g. `/ui/example_ui`).
 
 6. Run the launch menu and select the option. Accept by pressing 'OK' in the bottom right.
 7. Check if the parameter is set correctly in the parameter server (e.g. `rosparam get march/launch_menu/ui/example_ui`).
 
### Adding a new QGroupBox
You can add a new QGroupBox whenever you feel the need to group certain buttons together. It is advised to group settings together that are part of the same subtopic (e.g. `/ui/gazebo` and `/ui/rviz`).
 1. Drag the `QGroupBox` to the layout you want it to be a part of.
 2. Double-click the `QGroupBox` to change the title.
 3. Give the object a sensible name in the property editor.
 4. Add whichever checkboxes and radio buttons you want to the `QGroupBox`
 5. Right-click the new `QGroupBox` and change the layout to `Form Layout`.
 6. Optional: If this group will contain radio buttons, change the `statusTip` to the topic you want the setting to publish to (e.g. `/ui`).
 7. The new boxes and buttons will now be automatically loaded!

### Adding a new QRadioButton
It is important to note that only 1 `QRadioButton` can be active in a single `QGroupBox`. Therefore when you want multiple settings involving radio buttons you have to make multiple groups.

If you want to add an option to an existing group the process is quite simple.
 1. Drag the `QRadioButton` to the group.
 2. Give the object a sensible name in the property editor.
 3. Give it a sensible title, this is the value that will be uploaded to the parameter server with spaces removed and in lowercase.
 4. Double-check that the `QGroupBox` it belongs to has a statustip. This is the topic it will be published on.

### Adding a new run configuration
When a new run configuration arises it is relatively simple to add it to the launch menu.
 1. Navigate to `run_configuration_enum.py` and add your run configuration below the existing (e.g. `testjoint = "testjoint"`)
 2. Add a `QRadioButton` to the `RunConfigurationGroup` in Qt creator.
 3. Give it a name that matches the name of your run configuration. Spaces are stripped and it is case-insensitive (e.g. "Test Joint" and "TEs TJOin t" is fine, "test_joint" or "test-joint" is not.)
 4. Optionally give it a `toolTip` to give information to the user hovering over it.
 5. Add a new `QGroupBox` to the `RunConfigurationLayout`.
 6. Give this `QGroupBox` a sensible name in the property editor (e.g. `TestJointGroup`).
 7. Add whatever radio buttons and checkboxes you want.
 8. Right-click the new `QGroupBox` and change the layout to `Form Layout`.
 9. In `launch_menu.py` locate `run_configuration_changed()` and add your group to the if statement.
    ```
    ...
    elif enabled_configuration == run_configuration_enum.testjoint.name:
        self._widget.TestJointGroup.setEnabled(True)
    ...
    ```
10. Run the launch menu to test your new run configuration!
11. Validate it is set correctly with `rosparam get /march/launch_settings/run_configuration`

There are 2 errors that can be thrown when something goes wrong with this section.
 1. You did not add the run configuration to the enumerator, this throws a fatal error.
 2. It was added correctly, but not yet to the if statement. This throws a warning.
 
 
