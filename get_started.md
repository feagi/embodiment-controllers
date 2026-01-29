# Get started
1) Please see three examples. Each controller may handle and read data from the source differently, but the way you pass data to FEAGI should be exactly the same as the others.

Tello (Drone) example: https://github.com/feagi/controllers/blob/staging/embodiments/ryze_robotics/tello/controller.py#L245-L293

Cozmo (A toy) example: https://github.com/feagi/controllers/blob/staging/embodiments/digital_dream_labs/cozmo_1.0/controller.py

Mycobot (an arm) example: https://github.com/feagi/controllers/blob/staging/embodiments/elephant_robotics/pure_python_mycobot/controller.py#L105-L119

So, here is the template: https://github.com/feagi/controllers/blob/staging/embodiments/template/controller.py

2) The controller will need a capabilities file, which you can generate from here: https://github.com/feagi/controller_configurator/tree/main. This will generate the capabilities for you. You'll need to create a new file called `capabilities.json` and place it in the same folder as `controller.py`.

3) You will also need a `networking.json` file. We don't have a generator for that yet, but you can simply copy and paste it from here: https://github.com/feagi/controllers/blob/staging/embodiments/ufactory/lite_6/networking.json

4) There are various ways to connect a controller with FEAGI. You can test it based on your own preferences. If you want to use a website like https://brainsforrobots.com/, you can get a `magic_link` from an experiment inside NRS. If you prefer to run Docker, you can do that here: https://github.com/feagi/feagi/tree/staging/docker

To use `playground.yml` for Docker, run the following commands:
1) `docker compose -f playground.yml pull` # to fetch the latest images
2) `docker compose -f playground.yml up` # to start it
3) `docker compose -f playground.yml down` # to stop it

If you want to run FEAGI locally, you can clone the FEAGI repository: https://github.com/feagi/feagi/tree/staging

Navigate to `feagi/src/`, then run these commands:
python3 main.py

If you want to see red voxels, you will need to run the Godot bridge from here: https://github.com/feagi/feagi-connector/tree/staging/embodiments/godot-bridge
1) Navigate to `feagi-connector/embodiments/godot-bridge`
2) Run `python3 bridge_godot_python.py`

Make sure FEAGI is started before the Godot bridge, then you can use your own controller to test.

Again, it's entirely up to you how you choose to run it. Personally, I prefer running everything locally.

# Additional Information
For information about connectivity and creating your own controller, visit:
- [connectivity.md](connectivity.md)
- [create_controller.md](docs/create_controller.md)