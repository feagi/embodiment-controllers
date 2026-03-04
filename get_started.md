# Get started

**Prerequisite:** Start FEAGI and Brain Visualizer before launching any controller. See [FEAGI Installation](https://github.com/feagi/feagi-python-sdk/blob/main/DEPLOY.md).

1) Please see three examples. Each controller may handle and read data from the source differently, but the way you pass data to FEAGI should be exactly the same as the others.

- Tello (Drone): [embodiments/ryze_robotics/tello/](embodiments/ryze_robotics/tello/)
- Cozmo (Toy): [embodiments/digital_dream_labs/cozmo_1.0/](embodiments/digital_dream_labs/cozmo_1.0/)
- Mycobot (Arm): [embodiments/elephant_robotics/pure_python_mycobot/](embodiments/elephant_robotics/pure_python_mycobot/)

Template: [embodiments/template/](embodiments/template/)

2) The controller will need a capabilities file. You can generate it from [Controller Configurator](https://github.com/feagi/controller_configurator/tree/main). Create `capabilities.json` and place it in the same folder as `controller.py`.

3) You will also need a `networking.json` file. Copy from [embodiments/ufactory/lite_6/networking.json](embodiments/ufactory/lite_6/networking.json) as a starting point.

4) There are various ways to connect a controller with FEAGI. For [Neurorobotics Studio](https://studio.feagi.org), get a `magic_link` from an experiment. For Docker, see [feagi/docker](https://github.com/feagi/feagi/tree/staging/docker).

To use `playground.yml` for Docker:
- `docker compose -f playground.yml pull`
- `docker compose -f playground.yml up`
- `docker compose -f playground.yml down`

To run FEAGI locally (FEAGI 2.0):
```bash
pip install feagi
feagi start
feagi bv start
```

Make sure FEAGI is started before your controller.

# Additional Information
For information about connectivity and creating your own controller, visit:
- [connectivity.md](connectivity.md)
- [create_controller.md](docs/create_controller.md)