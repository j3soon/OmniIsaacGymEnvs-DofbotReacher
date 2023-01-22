# Dofbot Reacher Reinforcement Learning Sim2Real Environment for Omniverse Isaac Gym/Sim

This repository adds a DofbotReacher environment based on [OmniIsaacGymEnvs](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs) (commit [d0eaf2e](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/tree/d0eaf2e7f1e1e901d62e780392ca77843c08eb2c)), and includes Sim2Real code to control a real-world [Dofbot](https://category.yahboom.net/collections/r-robotics-arm/products/dofbot-jetson_nano) with the policy learned by reinforcement learning in Omniverse Isaac Gym/Sim.

We target Isaac Sim 2022.1.1 and test the RL code on Windows 10 and Ubuntu 18.04. The Sim2Real code is tested on Linux and a real Dofbot.

This repo is compatible with [OmniIsaacGymEnvs-UR10Reacher](https://github.com/j3soon/OmniIsaacGymEnvs-UR10Reacher).

## Preview

![](docs/media/DofbotReacher-Vectorized.gif)

![](docs/media/DofbotReacher-Sim2Real.gif)

## Installation

Prerequisites:
- [Install Omniverse Isaac Sim 2022.1.1](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html) (Must setup Cache and Nucleus)
- Your computer & GPU should be able to run the Cartpole example in [OmniIsaacGymEnvs](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs)
- (Optional) [Set up a Dofbot with Jetson Nano](http://www.yahboom.net/study/Dofbot-Jetson_nano) in the real world

We will use Anaconda to manage our virtual environment:

1. Clone this repository:
   - Linux
     ```sh
     cd ~
     git clone https://github.com/j3soon/OmniIsaacGymEnvs-DofbotReacher.git
     ```
   - Windows
     ```sh
     cd %USERPROFILE%
     git clone https://github.com/j3soon/OmniIsaacGymEnvs-DofbotReacher.git
     ```
2. Generate [instanceable](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gym_instanceable_assets.html) Dofbot assets for training:

   [Launch the Script Editor](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gui_interactive_scripting.html#script-editor) in Isaac Sim. Copy the content in `omniisaacgymenvs/utils/usd_utils/create_instanceable_dofbot.py` and execute it inside the Script Editor window. Wait until you see the text `Done!`.
3. [Download and Install Anaconda](https://www.anaconda.com/products/distribution#Downloads).
   ```sh
   # For 64-bit Linux (x86_64/x64/amd64/intel64)
   wget https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-x86_64.sh
   bash Anaconda3-2022.10-Linux-x86_64.sh
   ```
   For Windows users, make sure to use Anaconda Prompt instead of Command Prompt or Powershell for the following commands.
4. Patch Isaac Sim 2022.1.1
   - Linux
     ```sh
     export ISAAC_SIM="$HOME/.local/share/ov/pkg/isaac_sim-2022.1.1"
     cp $ISAAC_SIM/setup_python_env.sh $ISAAC_SIM/setup_python_env.sh.bak
     cp ~/OmniIsaacGymEnvs-DofbotReacher/isaac_sim-2022.1.1-patch/linux/setup_python_env.sh $ISAAC_SIM/setup_python_env.sh
     ```
   - Windows
     ```sh
     set ISAAC_SIM="%LOCALAPPDATA%\ov\pkg\isaac_sim-2022.1.1"
     copy %USERPROFILE%\OmniIsaacGymEnvs-DofbotReacher\isaac_sim-2022.1.1-patch\windows\setup_conda_env.bat %ISAAC_SIM%\setup_conda_env.bat
     ```
5. [Set up conda environment for Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_python.html#advanced-running-with-anaconda)
   - Linux
     ```sh
     # conda remove --name isaac-sim --all
     export ISAAC_SIM="$HOME/.local/share/ov/pkg/isaac_sim-2022.1.1"
     cd $ISAAC_SIM
     conda env create -f environment.yml
     conda activate isaac-sim
     cd ~/OmniIsaacGymEnvs-DofbotReacher
     pip install -e .
     ```
   - Windows
     ```sh
     # conda remove --name isaac-sim --all
     set ISAAC_SIM="%LOCALAPPDATA%\ov\pkg\isaac_sim-2022.1.1"
     cd %ISAAC_SIM%
     conda env create -f environment.yml
     conda activate isaac-sim
     :: Fix incorrect importlib-metadata version (isaac-sim 2022.1.1)
     pip install importlib-metadata==4.11.4
     cd %USERPROFILE%\OmniIsaacGymEnvs-DofbotReacher
     pip install -e .
     :: Fix incorrect torch version (isaac-sim 2022.1.1)
     conda install -y pytorch==1.11.0 torchvision==0.12.0 torchaudio==0.11.0 -c pytorch
     ```
6. Activate conda environment
   - Linux
     ```sh
     export ISAAC_SIM="$HOME/.local/share/ov/pkg/isaac_sim-2022.1.1"
     cd $ISAAC_SIM
     conda activate isaac-sim
     source setup_conda_env.sh
     ```
   - Windows
     ```sh
     set ISAAC_SIM="%LOCALAPPDATA%\ov\pkg\isaac_sim-2022.1.1"
     cd %ISAAC_SIM%
     conda activate isaac-sim
     call setup_conda_env.bat
     ```

Please note that you should execute the commands in Step 6 for every new shell.

For Windows users, replace `~` to `%USERPROFILE%` for all the following commands.

## Dummy Policy

This is a sample to make sure you have setup the environment correctly. You should see a single Dofbot in Isaac Sim.

```sh
cd ~/OmniIsaacGymEnvs-DofbotReacher
python omniisaacgymenvs/scripts/dummy_dofbot_policy.py task=DofbotReacher test=True num_envs=1
```

## Training

You can launch the training in `headless` mode as follows:

```sh
cd ~/OmniIsaacGymEnvs-DofbotReacher
python omniisaacgymenvs/scripts/rlgames_train.py task=DofbotReacher headless=True
```

The number of environments is set to 2048 by default. If your GPU has small memory, you can decrease the number of environments by changing the arguments `num_envs` as below:

```sh
cd ~/OmniIsaacGymEnvs-DofbotReacher
python omniisaacgymenvs/scripts/rlgames_train.py task=DofbotReacher headless=True num_envs=2048
```

You can also skip training by downloading the pre-trained model checkpoint by:

```sh
cd ~/OmniIsaacGymEnvs-DofbotReacher
wget https://github.com/j3soon/OmniIsaacGymEnvs-DofbotReacher/releases/download/v1.1.0/runs.zip
unzip runs.zip
```

The learning curve of the pre-trained model:

![](docs/media/DofbotReacher-Learning-Curve.png)

## Testing

Make sure you have model checkpoints at `~/OmniIsaacGymEnvs-DofbotReacher/runs`, you can check it with the following command:

```sh
ls ~/OmniIsaacGymEnvs-DofbotReacher/runs/DofbotReacher/nn/
```

Please note that you may not want to use the checkpoint `./runs/DofbotReacher/nn/DofbotReacher.pth` due to the randomness of the reward signal. Instead, use the latest checkpoint such as `./runs/DofbotReacher/nn/last_DofbotReacher_ep_1000_rew_XXX.pth`. You can replace `DofbotReacher.pth` with the latest checkpoint before following the steps below, or you can simply modify the commands to use the latest checkpoint.

You can visualize the learned policy by the following command:

```sh
cd ~/OmniIsaacGymEnvs-DofbotReacher
python omniisaacgymenvs/scripts/rlgames_train.py task=DofbotReacher test=True num_envs=512 checkpoint=./runs/DofbotReacher/nn/DofbotReacher.pth
```

Likewise, you can decrease the number of environments by modifying the parameter `num_envs=512`.

## Sim2Real

The learned policy has a very conservative constraint on the joint limits. Therefore, the gripper would not hit the ground under such limits. However, you should still make sure there are no other obstacles within Dofbot's workspace (reachable area). That being said, if things go wrong, press `Ctrl+C` twice in the terminal to kill the process.

> It would be possible to remove the conservative joint limit constraints by utilizing self-collision detection in Isaac Sim. We are currently investigating this feature.

For simplicity, we'll use TCP instead of ROS to control the real-world dofbot. Copy the server notebook file (`omniisaacgymenvs/sim2real/dofbot-server.ipynb`) to the Jetson Nano on your Dofbot. Launch a Jupyter Notebook on Jetson Nano and execute the server notebook file.

You should be able to reset the Dofbot's joints by the following script:

```sh
cd ~/OmniIsaacGymEnvs-DofbotReacher
python omniisaacgymenvs/sim2real/dofbot.py
```

Edit `omniisaacgymenvs/cfg/task/DofbotReacher.yaml`. Set `sim2real.enabled` to `True`, and set `sim2real.ip` to the IP of your Dofbot:

```yaml
sim2real:
  enabled: True
  fail_quietely: False
  verbose: False
  ip: <IP_OF_YOUR_DOFBOT>
  port: 65432
```

Now you can control the real-world Dofbot in real-time by the following command:

```sh
cd ~/OmniIsaacGymEnvs-DofbotReacher
python omniisaacgymenvs/scripts/rlgames_train.py task=DofbotReacher test=True num_envs=1 checkpoint=./runs/DofbotReacher/nn/DofbotReacher.pth
```

## Demo

We provide an interactable demo based on the `DofbotReacher` RL example. In this demo, you can click on any of
the Dofbot in the scene to manually control the robot with your keyboard as follows:

- `Q`/`A`: Control Joint 0.
- `W`/`S`: Control Joint 1.
- `E`/`D`: Control Joint 2.
- `R`/`F`: Control Joint 3.
- `T`/`G`: Control Joint 4.
- `Y`/`H`: Control Joint 5.
- `ESC`: Unselect a selected Dofbot and yields manual control

Launch this demo with the following command. Note that this demo limits the maximum number of Dofbot in the scene to 128.

```sh
cd ~/OmniIsaacGymEnvs-DofbotReacher
python omniisaacgymenvs/scripts/rlgames_play.py task=DofbotReacher num_envs=64
```

## Running in Docker

If you have a [NVIDIA Enterprise subscription](https://docs.omniverse.nvidia.com/prod_nucleus/prod_nucleus/enterprise/installation/planning.html), you can run all services with Docker Compose.

For users without a subscription, you can pull the [Isaac Docker image](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim), but should still install Omniverse Nucleus beforehand. (only Isaac itself is dockerized)

Follow [this tutorial](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_advanced_container_deployment.html) to generate your NGC API Key, and make sure you can access Isaac with Omniverse Streaming Client, WebRTC, or WebSocket. After that, exit the Docker container.

Please note that you should generate instanceable assets beforehand as mentioned in the [Installation](#installation) section.

We will now set up the environment inside Docker:

1. Launch an Isaac Container:
   ```sh
   docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
   -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
   -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
   -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
   -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
   -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
   -v ~/docker/isaac-sim/config:/root/.nvidia-omniverse/config:rw \
   -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
   -v ~/docker/isaac-sim/documents:/root/Documents:rw \
   nvcr.io/nvidia/isaac-sim:2022.1.1
   ```
2. Install common tools:
   ```sh
   apt update && apt install -y git wget vim
   ```
3. Clone this repository:
   ```sh
   cd ~
   git clone https://github.com/j3soon/OmniIsaacGymEnvs-DofbotReacher.git
   ```
4. [Download and Install Anaconda](https://www.anaconda.com/products/distribution#Downloads).
   ```sh
   # For 64-bit (x86_64/x64/amd64/intel64)
   wget https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-x86_64.sh
   bash Anaconda3-2022.10-Linux-x86_64.sh -b -p $HOME/anaconda3
   ```
5. Patch Isaac Sim 2022.1.1
   ```sh
   export ISAAC_SIM="/isaac-sim"
   cp $ISAAC_SIM/setup_python_env.sh $ISAAC_SIM/setup_python_env.sh.bak
   cp ~/OmniIsaacGymEnvs-DofbotReacher/isaac_sim-2022.1.1-patch/setup_python_env.sh $ISAAC_SIM/setup_python_env.sh
   ```
6. [Set up conda environment for Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_python.html#advanced-running-with-anaconda)
   ```sh
   source ~/anaconda3/etc/profile.d/conda.sh
   # conda remove --name isaac-sim --all
   export ISAAC_SIM="/isaac-sim"
   cd $ISAAC_SIM
   conda env create -f environment.yml
   conda activate isaac-sim
   cd ~/OmniIsaacGymEnvs-DofbotReacher
   pip install -e .
   ```
7. Activate conda environment
   ```sh
   source ~/anaconda3/etc/profile.d/conda.sh
   export ISAAC_SIM="/isaac-sim"
   cd $ISAAC_SIM
   conda activate isaac-sim
   source setup_conda_env.sh
   ./vulkan_check.sh
   ```

We can now train a RL policy in this container:

```sh
cd ~/OmniIsaacGymEnvs-DofbotReacher
python omniisaacgymenvs/scripts/rlgames_train.py task=DofbotReacher headless=True num_envs=2048
```

Make sure to copy the learned weights to a mounted volume before exiting the container, otherwise it will be deleted:

```sh
# In container
cp -r ~/OmniIsaacGymEnvs-DofbotReacher/runs ~/Documents/runs
# In host
ls ~/docker/isaac-sim/documents/
```

You can watch the training progress with:

```sh
docker ps # Observe Container ID
docker exec -it <CONTAINER_ID> /bin/bash
conda activate isaac-sim
cd ~/OmniIsaacGymEnvs-DofbotReacher
tensorboard --logdir=./runs
```

Currently we do not support running commands that requires visualization (Testing, Sim2Real, etc.) in Docker. Since I haven't figured out how to make Vulkan render a Isaac window inside a container yet. Alternatively, it may be possible to add `headless=True` and view them in Omniverse Streaming Client, WebRTC, or WebSocket, but I haven't tested this by myself.

> **Note**: below are the original README of [OmniIsaacGymEnvs](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs).

# Omniverse Isaac Gym Reinforcement Learning Environments for Isaac Sim

### About this repository

This repository contains Reinforcement Learning examples that can be run with the latest release of [Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html). RL examples are trained using PPO from [rl_games](https://github.com/Denys88/rl_games) library and examples are built on top of Isaac Sim's `omni.isaac.core` and `omni.isaac.gym` frameworks.

<img src="https://user-images.githubusercontent.com/34286328/171454189-6afafbff-bb61-4aac-b518-24646007cb9f.gif" width="300" height="150"/>&emsp;<img src="https://user-images.githubusercontent.com/34286328/184172037-cdad9ee8-f705-466f-bbde-3caa6c7dea37.gif" width="300" height="150"/>

<img src="https://user-images.githubusercontent.com/34286328/171454182-0be1b830-bceb-4cfd-93fb-e1eb8871ec68.gif" width="300" height="150"/>&emsp;<img src="https://user-images.githubusercontent.com/34286328/171454193-e027885d-1510-4ef4-b838-06b37f70c1c7.gif" width="300" height="150"/>

<img src="https://user-images.githubusercontent.com/34286328/184174894-03767aa0-936c-4bfe-bbe9-a6865f539bb4.gif" width="300" height="150"/>&emsp;<img src="https://user-images.githubusercontent.com/34286328/184168200-152567a8-3354-4947-9ae0-9443a56fee4c.gif" width="300" height="150"/>

<img src="https://user-images.githubusercontent.com/34286328/184176312-df7d2727-f043-46e3-b537-48a583d321b9.gif" width="300" height="150"/>&emsp;<img src="https://user-images.githubusercontent.com/34286328/184178817-9c4b6b3c-c8a2-41fb-94be-cfc8ece51d5d.gif" width="300" height="150"/>

<img src="https://user-images.githubusercontent.com/34286328/171454160-8cb6739d-162a-4c84-922d-cda04382633f.gif" width="300" height="150"/>&emsp;<img src="https://user-images.githubusercontent.com/34286328/171454176-ce08f6d0-3087-4ecc-9273-7d30d8f73f6d.gif" width="300" height="150"/>

<img src="https://user-images.githubusercontent.com/34286328/184170040-3f76f761-e748-452e-b8c8-3cc1c7c8cb98.gif" width="614" height="307"/>

### Installation

Follow the Isaac Sim [documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html) to install the latest Isaac Sim release.

*Examples in this repository rely on features from the most recent Isaac Sim release. Please make sure to update any existing Isaac Sim build to the latest release version, 2022.1.1, to ensure examples work as expected.*

Once installed, this repository can be used as a python module, `omniisaacgymenvs`, with the python executable provided in Isaac Sim.

To install `omniisaacgymenvs`, first clone this repository:

```bash
git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs.git
```

Once cloned, locate the [python executable in Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_python.html). By default, this should be `python.sh`. We will refer to this path as `PYTHON_PATH`.

To set a `PYTHON_PATH` variable in the terminal that links to the python executable, we can run a command that resembles the following. Make sure to update the paths to your local path.

```
For Linux: alias PYTHON_PATH=~/.local/share/ov/pkg/isaac_sim-*/python.sh
For Windows: doskey PYTHON_PATH=C:\Users\user\AppData\Local\ov\pkg\isaac_sim-*\python.bat $*
```

Install `omniisaacgymenvs` as a python module for `PYTHON_PATH`:

```bash
PYTHON_PATH -m pip install -e .
```


### Running the examples

*Note: All commands should be executed from `omniisaacgymenvs/omniisaacgymenvs`.*

To train your first policy, run:

```bash
PYTHON_PATH scripts/rlgames_train.py task=Cartpole
```

You should see an Isaac Sim window pop up. Once Isaac Sim initialization completes, the Cartpole scene will be constructed and simulation will start running automatically. The process will terminate once training finishes.


Here's another example - Ant locomotion - using the multi-threaded training script:

```bash
PYTHON_PATH scripts/rlgames_train_mt.py task=Ant
```

Note that by default, we show a Viewport window with rendering, which slows down training. You can choose to close the Viewport window during training for better performance. The Viewport window can be re-enabled by selecting `Window > Viewport` from the top menu bar.

To achieve maximum performance, you can launch training in `headless` mode as follows:

```bash
PYTHON_PATH scripts/rlgames_train.py task=Ant headless=True
```

#### A Note on the Startup Time of the Simulation

Some of the examples could take a few minutes to load because the startup time scales based on the number of environments. The startup time will continually
be optimized in future releases.


### Loading trained models // Checkpoints

Checkpoints are saved in the folder `runs/EXPERIMENT_NAME/nn` where `EXPERIMENT_NAME`
defaults to the task name, but can also be overridden via the `experiment` argument.

To load a trained checkpoint and continue training, use the `checkpoint` argument:

```bash
PYTHON_PATH scripts/rlgames_train.py task=Ant checkpoint=runs/Ant/nn/Ant.pth
```

To load a trained checkpoint and only perform inference (no training), pass `test=True`
as an argument, along with the checkpoint name. To avoid rendering overhead, you may
also want to run with fewer environments using `num_envs=64`:

```bash
PYTHON_PATH scripts/rlgames_train.py task=Ant checkpoint=runs/Ant/nn/Ant.pth test=True num_envs=64
```

Note that if there are special characters such as `[` or `=` in the checkpoint names,
you will need to escape them and put quotes around the string. For example,
`checkpoint="runs/Ant/nn/last_Antep\=501rew\[5981.31\].pth"`

We provide pre-trained checkpoints on the [Nucleus](https://docs.omniverse.nvidia.com/prod_nucleus/prod_nucleus/overview.html) server under `Assets/Isaac/2022.1/Isaac/Samples/OmniIsaacGymEnvs/Checkpoints`. Run the following command
to launch inference with pre-trained checkpoint:

Localhost (To set up localhost, please refer to the [Isaac Sim installation guide](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html)):

```bash
PYTHON_PATH scripts/rlgames_train.py task=Ant checkpoint=omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Samples/OmniIsaacGymEnvs/Checkpoints/ant.pth test=True num_envs=64
```

Production server:

```bash
PYTHON_PATH scripts/rlgames_train.py task=Ant checkpoint=http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.1/Isaac/Samples/OmniIsaacGymEnvs/Checkpoints/ant.pth test=True num_envs=64
```

When running with a pre-trained checkpoint for the first time, we will automatically download the checkpoint file to `omniisaacgymenvs/checkpoints`. For subsequent runs, we will re-use the file that has already been downloaded, and will not overwrite existing checkpoints with the same name in the `checkpoints` folder.

## Training Scripts

All scripts provided in `omniisaacgymenvs/scripts` can be launched directly with `PYTHON_PATH`.

To test out a task without RL in the loop, run the random policy script with:

```bash
PYTHON_PATH scripts/random_policy.py task=Cartpole
```

This script will sample random actions from the action space and apply these actions to your task without running any RL policies. Simulation should start automatically after launching the script, and will run indefinitely until terminated.


To run a simple form of PPO from `rl_games`, use the single-threaded training script:

```bash
PYTHON_PATH scripts/rlgames_train.py task=Cartpole
```

This script creates an instance of the PPO runner in `rl_games` and automatically launches training and simulation. Once training completes (the total number of iterations have been reached), the script will exit. If running inference with `test=True checkpoint=<path/to/checkpoint>`, the script will run indefinitely until terminated. Note that this script will have limitations on interaction with the UI.


Lastly, we provide a multi-threaded training script that executes the RL policy on a separate thread than the main thread used for simulation and rendering:

```bash
PYTHON_PATH scripts/rlgames_train_mt.py task=Cartpole
```

This script uses the same RL Games PPO policy as the above, but runs the RL loop on a new thread. Communication between the RL thread and the main thread happens on threaded Queues. Simulation will start automatically, but the script will **not** exit when training terminates, except when running in headless mode. Simulation will stop when training completes or can be stopped by clicking on the Stop button in the UI. Training can be launched again by clicking on the Play button. Similarly, if running inference with `test=True checkpoint=<path/to/checkpoint>`, simulation will run until the Stop button is clicked, or the script will run indefinitely until the process is terminated.


### Configuration and command line arguments

We use [Hydra](https://hydra.cc/docs/intro/) to manage the config.

Common arguments for the training scripts are:

* `task=TASK` - Selects which task to use. Any of `AllegroHand`, `Ant`, `Anymal`, `AnymalTerrain`, `BallBalance`, `Cartpole`, `Crazyflie`, `FrankaCabinet`, `Humanoid`, `Ingenuity`, `Quadcopter`, `ShadowHand`, `ShadowHandOpenAI_FF`, `ShadowHandOpenAI_LSTM` (these correspond to the config for each environment in the folder `omniisaacgymenvs/cfg/task`)
* `train=TRAIN` - Selects which training config to use. Will automatically default to the correct config for the environment (ie. `<TASK>PPO`).
* `num_envs=NUM_ENVS` - Selects the number of environments to use (overriding the default number of environments set in the task config).
* `seed=SEED` - Sets a seed value for randomization, and overrides the default seed in the task config
* `pipeline=PIPELINE` - Which API pipeline to use. Defaults to `gpu`, can also set to `cpu`. When using the `gpu` pipeline, all data stays on the GPU. When using the `cpu` pipeline, simulation can run on either CPU or GPU, depending on the `sim_device` setting, but a copy of the data is always made on the CPU at every step.
* `sim_device=SIM_DEVICE` - Device used for physics simulation. Set to `gpu` (default) to use GPU and to `cpu` for CPU.
* `device_id=DEVICE_ID` - Device ID for GPU to use for simulation and task. Defaults to `0`. This parameter will only be used if simulation runs on GPU.
* `rl_device=RL_DEVICE` - Which device / ID to use for the RL algorithm. Defaults to `cuda:0`, and follows PyTorch-like device syntax.
* `test=TEST`- If set to `True`, only runs inference on the policy and does not do any training.
* `checkpoint=CHECKPOINT_PATH` - Path to the checkpoint to load for training or testing.
* `headless=HEADLESS` - Whether to run in headless mode.
* `experiment=EXPERIMENT` - Sets the name of the experiment.
* `max_iterations=MAX_ITERATIONS` - Sets how many iterations to run for. Reasonable defaults are provided for the provided environments.

Hydra also allows setting variables inside config files directly as command line arguments. As an example, to set the minibatch size for a rl_games training run, you can use `train.params.config.minibatch_size=64`. Similarly, variables in task configs can also be set. For example, `task.env.episodeLength=100`.

#### Hydra Notes

Default values for each of these are found in the `omniisaacgymenvs/cfg/config.yaml` file.

The way that the `task` and `train` portions of the config works are through the use of config groups.
You can learn more about how these work [here](https://hydra.cc/docs/tutorials/structured_config/config_groups/)
The actual configs for `task` are in `omniisaacgymenvs/cfg/task/<TASK>.yaml` and for `train` in `omniisaacgymenvs/cfg/train/<TASK>PPO.yaml`.

In some places in the config you will find other variables referenced (for example,
 `num_actors: ${....task.env.numEnvs}`). Each `.` represents going one level up in the config hierarchy.
 This is documented fully [here](https://omegaconf.readthedocs.io/en/latest/usage.html#variable-interpolation).

### Tensorboard

Tensorboard can be launched during training via the following command:
```bash
PYTHON_PATH -m tensorboard.main --logdir runs/EXPERIMENT_NAME/summaries
```

## WandB support

You can run (WandB)[https://wandb.ai/] with OmniIsaacGymEnvs by setting `wandb_activate=True` flag from the command line. You can set the group, name, entity, and project for the run by setting the `wandb_group`, `wandb_name`, `wandb_entity` and `wandb_project` arguments. Make sure you have WandB installed in the Isaac Sim Python executable with `PYTHON_PATH -m pip install wandb` before activating.


## Tasks

Source code for tasks can be found in `omniisaacgymenvs/tasks`.

Each task follows the frameworks provided in `omni.isaac.core` and `omni.isaac.gym` in Isaac Sim.

Refer to [docs/framework.md](docs/framework.md) for how to create your own tasks.

Full details on each of the tasks available can be found in the [RL examples documentation](docs/rl_examples.md).


## Demo

We provide an interactable demo based on the `AnymalTerrain` RL example. In this demo, you can click on any of
the ANYmals in the scene to go into third-person mode and manually control the robot with your keyboard as follows:

- `Up Arrow`: Forward linear velocity command
- `Down Arrow`: Backward linear velocity command
- `Left Arrow`: Leftward linear velocity command
- `Right Arrow`: Rightward linear velocity command
- `Z`: Counterclockwise yaw angular velocity command
- `X`: Clockwise yaw angular velocity command
- `C`: Toggles camera view between third-person and scene view while maintaining manual control
- `ESC`: Unselect a selected ANYmal and yields manual control

Launch this demo with the following command. Note that this demo limits the maximum number of ANYmals in the scene to 128.

```
PYTHON_PATH scripts/rlgames_play.py task=AnymalTerrain num_envs=64 checkpoint=omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Samples/OmniIsaacGymEnvs/Checkpoints/anymal_terrain.pth
```

<img src="https://user-images.githubusercontent.com/34286328/184688654-6e7899b2-5847-4184-8944-2a96b129b1ff.gif" width="600" height="300"/>


## A note about Force Sensors

Force sensors are supported in Isaac Sim and OIGE via the `ArticulationView` class. Sensor readings can be retrieved using `get_force_sensor_forces()` API, as shown in the Ant/Humanoid Locomotion task, as well as in the Ball Balance task. Please note that there is currently a known bug regarding force sensors in Omniverse Physics. Transforms of force sensors (i.e. their local poses) are set in the actor space of the Articulation instead of the body space, which is the expected behaviour. We will be fixing this in the coming release.
