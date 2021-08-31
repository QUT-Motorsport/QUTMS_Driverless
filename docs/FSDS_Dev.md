# 2. FS Driverless Simulator development

## Set Up FSDS

Download FS Driverless Simulator from the Formula Student GitHub:

> <https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.0.0/fsds-v2.0.0-win.zip>

Extract the files to a folder of your choosing.

In the QUTMS Driverless repo folder `fs_sim_settings`, there is a folder titled: `Formula-Student-Driverless-Simulator`.
Copy the entire folder into your user directory.
Eg. `C:\Users\Username\`.

To test this was successful, run `FSDS.exe`.
You should be able to run the simulation and control the car with 'WASD' keys.

## Running Scripts

To run scripts, we must first build a development container with the Docker image, navigate to the same QUTMS Driverless directory where you built the image and type:

> `make run target=perception`

This will bring you into the virtual container made by the image. The command line will now look like this:

> `developer@docker-desktop:/home/developer/driverless_ws$`

To build the ros2 packages for the FS simulator, type:

> `colcon build`

First time building this may take up to 3 minutes, sequential builds inside the container may be quicker.

Now, to initialise scripts with the ros2 environment, type:

> `source install/setup.bash`

Open a new terminal tab and navigate to the same QUTMS Driverless directory where we have been working, or open the folder in terminal. Ensure XServer and the FSDS simulator is running. Type:

> `make run target=unreal_sim`

This will make a 'bridge' between the FS simulator and the ros2 node where scripts are running so data can be output and read.

Finally, to run the included script that has been developed, type:

> `ros2 run <PACKAGE> <SCRIPT>`

This would change based on your program.