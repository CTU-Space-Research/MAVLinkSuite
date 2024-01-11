
# 📖 MAVLink _mavgen_, Illustria 2 dialect, MAVLink python tester

> Example how to use pre-generated MAVLink files (with slight modifications) in a embedded STM32 project.

### Important files / folders:

- mavgenerate.py = _mavgen_ executable
- /message_definitions = XML dialects definitions
- /generated_libs = generated C and python files by the _mavgen_ app
- /mavlink_python_tester = my python app for simple protocol testing via USB serial port. Based on the generated py lib from dialect

<!-- GETTING STARTED -->
## 💻 Getting Started

To install the minimal MAVLink environment on Ubuntu LTS 20.04 or 22.04, enter the following on a terminal:

```bash
# Dependencies
sudo apt install python3-pip

python3 -m pip install tk #tkinter is required for the pymavlink to work and is not stated in the requirements file

python3 -m pip install -r pymavlink/requirements.txt #if you clone this repo and already has python 3, install the pymavlink dependencies
```


## 🛠 Generating C/Py libraries using _mavgen_ 

Opening the GUI:
```bash
# Dependencies
sudo python3 mavgenerate.py
```

Settings
- XLM = your dialect in *message_definitions*
- Out = set it as *generated_libs*
- Language = Python3 / C
- Protocol = 2

After the files are generated, move the in the generated_libs folder so that they correspond with the already created structure

You can find the block diagram how to use the generated libs in the *MS Teams -> CTU SR -> Avionics -> Knowledgebase -> Schematics -> MAVLink*.


## 🛠 Using Mavlink python tester app

Starting the app:

```bash
# Dependencies
cd mavlink_python_tester/

sudo sudo python3 async_mavlink_tool.py 
```


## Key Links
* [Documentation/Website](https://mavlink.io/en/) (mavlink.io/en/)
* [Discussion/Support](https://mavlink.io/en/#support)
* [Contributing](https://mavlink.io/en/contributing/contributing.html)
* [License](https://mavlink.io/en/#license)
