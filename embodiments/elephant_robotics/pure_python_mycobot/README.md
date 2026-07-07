# Connect with MyCobot
### IMPORTANT: Ensure you have already updated based on the MyCobot requirements. Also, make sure that you are already running FEAGI and the Brain Visualizer. If you are using the NRS studio, then don't worry about the previous warning.

1. `git clone https://github.com/feagi/embodiment-controllers.git`
2. `cd embodiment-controllers/embodiments/elephant_robotics/pure_python_mycobot`
3. `python3 -m venv venv`
4. `source venv/bin/activate`
5. Find the type of USB MyCobot uses.
   (See below to find out how to find the path for USB on Linux, Mac, or Windows.)

**Mac/Linux users:**
```bash
pip3 install -r requirements.txt
python3 controller.py --magic_link "<insert link here>"  
# Replace the bracket. This is for NRS studio. Not using NRS studio? Remove --magic_link
```

**Windows users:**
```bash
pip install -r requirements.txt
python controller.py --magic_link "<insert link here>"  # Replace the bracket.
# This is for NRS studio. Not using NRS studio? Remove --magic_link
```

---
# Extra flags
Example command: `python controller.py --help`
```commandline
optional arguments:
  -h, --help            Show this help message and exit.
  
  -magic_link MAGIC_LINK, --magic_link MAGIC_LINK
                        Use a magic link. You can find your magic link from NRS studio.
                        
  -magic-link MAGIC_LINK, --magic-link MAGIC_LINK
                        Use a magic link. You can find your magic link from NRS studio.
                        
  -magic MAGIC, --magic_link MAGIC
                        Use a magic link. You can find your magic link from NRS studio.
                        
  -ip IP, --ip IP       Specify the FEAGI IP address.
  
  -port PORT, --port PORT
                        Change the ZMQ port. Use 30000 for Docker and 3000 for localhost.

```

# How to Find and Update Your USB Path in the configuration.json File

## Windows
Check the small dialog box that appears in the right corner to see the port address. Use that name.

## Mac
Open the terminal and type `ls /dev/cu.`. Before pressing enter, be sure to press tab to autocomplete the path. This should be your path to MyCobot. If it shows multiple paths, check each one until you see your robot start to move.

## Terminal
Open the terminal, type `ls /dev/tty`, and then press tab to find your path to MyCobot.