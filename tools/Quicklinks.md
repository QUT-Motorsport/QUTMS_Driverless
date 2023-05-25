These are useful links and shortcuts to help you get started with the QUTMS Driverless project.

<br>

# Using the CLI tools
Build tools first with colcon
```
colcon build --symlink-install --packages-select qutms_cli_tools
```

Then source the workspace
```
source install/setup.bash
```

Tools are prefaced with `ws` for 'WorkSpace'
View tools available
```
ws
```
Will output:
```
 QUTMS CLI Tools 
 Usage: ws_<command> [<args>]
 Workspace Commands:
        ws_pull         Pull selected repos
        ws_build        Build selected packages
        ws_launch       Launch groups of ROS launch files
        ws_format       Pre-commit format in repo
        ws_record       Record topics to ROS2 bag
 Args:
        *Check help for each command for specific args*
```

To find out more about a tool, use the `-h` flag
