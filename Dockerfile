# Ref: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim
FROM nvcr.io/nvidia/isaac-sim:2023.1.0-hotfix.1

ENV ISAAC_SIM=/isaac-sim
WORKDIR /root
# Install common tools
RUN apt-get update && apt-get install -y git wget vim \
    && rm -rf /var/lib/apt/lists/*
# Download and Install Anaconda
# Ref: https://www.anaconda.com/products/distribution#Downloads
RUN wget -q https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-x86_64.sh \
    && bash Anaconda3-2022.10-Linux-x86_64.sh -b -p $HOME/anaconda3
# Patch Isaac Sim 2023.1.0
# Ref: https://github.com/j3soon/isaac-extended
RUN git clone https://github.com/j3soon/isaac-extended.git \
    && cp $ISAAC_SIM/setup_python_env.sh $ISAAC_SIM/setup_python_env.sh.bak \
    && cp ~/isaac-extended/isaac_sim-2023.1.0-patch/linux/setup_python_env.sh $ISAAC_SIM/setup_python_env.sh
# Set up conda environment for Isaac Sim
# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#advanced-running-with-anaconda
RUN . ~/anaconda3/etc/profile.d/conda.sh \
    # conda remove --name isaac-sim --all
    && cd $ISAAC_SIM \
    && conda env create -f environment.yml
# Activation commands for the conda environment
RUN echo ". ~/anaconda3/etc/profile.d/conda.sh" >> ~/.bashrc \
    && echo "conda activate isaac-sim" >> ~/.bashrc \
    && echo ". ${ISAAC_SIM}/setup_conda_env.sh" >> ~/.bashrc

WORKDIR /isaac-sim
