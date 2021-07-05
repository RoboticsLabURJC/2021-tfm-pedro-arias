## Build plugin

```bash
cd build
rm -rf *
cmake ../
make
```

## Setup env vars
```bash
GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$PWD/build
GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/models
```
