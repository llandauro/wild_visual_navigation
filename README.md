# Vision pipeline
This repo is for the online vision pipeline that uses trained physical decoders to output dense prediction of the environments in the vision channel. The pipeline is based on the previous repo from "Fast Traversability Estimation for Wild Visual Navigation".

## Installation
**Attention**: Please follow the installation order exactly as below. Otherwise, you may encounter some errors.
### Install robostack ros first:
https://robostack.github.io/GettingStarted.html

### Install pytorch next:
(Here we use mamba for virtual environment management with python 3.9)
```bash
mamba install pytorch torchvision torchaudio pytorch-cuda=12.1 -c pytorch -c nvidia
```
### Install other dependencies:
```bash
pip install -r requirements.txt
```
If you encounter any errors, please follow the error message to install the missing dependencies.

### Install this repo:
```bash
pip install .
```
## Vision pipeline - offline training
All configs are set in `BaseWVN/config/wvn_config.py`, for all the training/testing, you should pay attention to path-related settings.
### Offline Dataset
It is generated from the online rosbag playing. By setting `label_ext_mode: bool=True` you can record the dataset. The corresponding settings and paths are in config file.
```bash
python src/wild_visual_navigation_ros/scripts/Phy_decoder_node.py  # start phy decoders
python src/wild_visual_navigation_ros/scripts/Main_process_node.py # start main process
```
`ctrl+c` to stop/finish the recording.

### Manual correction of GT masks
Beacause the automatically generated GT masks (from SAM or SEEM) are not perfect, we need to manually correct them with segments.ai . 

You can use the `BaseWVN/offline/seg_correction.py` to correct the masks. The detailed usage you can refer to the code.
### Running
For offline training/testing, you can switch the config and run the following command:
```bash
python BaseWVN/offline/offline_training_lightning.py
```

## Vision pipeline - online training

### Running
For different configs, please refer to the code and config file.
```bash
python src/wild_visual_navigation_ros/scripts/Phy_decoder_node.py  # start phy decoders
python src/wild_visual_navigation_ros/scripts/Main_process_node.py # start main process
```

