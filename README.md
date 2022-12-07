## Requirements

* Docker deamon
* Probably linux
* (optional) NVIDIA

## Build

```bash
./build.sh
```

## Run

```bash
./run.sh
```

## Run with nvidia

```bash
./run_with_nvidia.sh
```

## Interesing directories and files

* `config` - directory with 3d models, demo menu and launchers (tools and vehicle)
* `code_prelection/code_prelection/obstacles.py` - simulates obstacles on the map
* `code_prelection/code_prelection/vehicle.py` - simulates vehicle with steer, speed and close/open door
* `code_prelection/code_prelection/autonomy_controller.py` - simple detecting collision with controlling speed of vehicle
