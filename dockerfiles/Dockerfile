FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04
ARG NJOBS=1
LABEL org.opencontainers.image.title="ROFT Samples Docker Image"
LABEL org.opencontainers.image.description="Infrastructure for running ROFT samples"
LABEL org.opencontainers.image.source="https://raw.githubusercontent.com/hsp-iit/roft-samples/master/dockerfiles/Dockerfile"
LABEL org.opencontainers.image.authors="Nicola A. Piga <nicola.piga@iit.it>"

# Use /bin/bash instead of /bin/sh
SHELL ["/bin/bash", "-c"]

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Set the locale
RUN apt update && \
    apt install -y -qq locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essentials
RUN apt update && \
    apt install --no-install-recommends -y -qq apt-utils build-essential ca-certificates cmake cmake-curses-gui curl emacs-nox git glmark2 gnupg2 htop iputils-ping jq libssl-dev libusb-1.0-0-dev lsb-release mesa-utils nano psmisc python3-virtualenv sudo unzip vim wget zip && \
    rm -rf /var/lib/apt/lists/*

# Install GitHub cli
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg && \
    chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
    apt update && \
    apt install --no-install-recommends -y -qq gh && \
    rm -rf /var/lib/apt/lists/*

# Install additional dependencies
RUN apt update && \
    apt install --no-install-recommends -y -qq eog libassimp-dev libconfig++-dev libglfw3-dev libglew-dev libgtk2.0-dev libglm-dev libeigen3-dev libpython3-dev libtclap-dev && \
    git clone https://github.com/robotology/robotology-superbuild && cd robotology-superbuild && bash scripts/install_apt_dependencies.sh && \
    apt remove -y libopencv* && \
    rm -rf /var/lib/apt/lists/*

# Build OpenCV
RUN git clone https://github.com/opencv/opencv && \
    git clone https://github.com/opencv/opencv_contrib && \
    cd /opencv_contrib && git checkout 4.5.2 && \
    cd /opencv && git checkout 4.5.2 && \
    mkdir build && cd build && \
    cmake -DOPENCV_ENABLE_NONFREE=ON -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -DWITH_TBB=ON -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DOPENCV_DNN_CUDA=OFF -DWITH_CUDA=ON -DWITH_CUDNN=OFF -DWITH_GTK_2_X=ON -DCUDA_GENERATION=Ampere ../ && \
    make -j$NJOBS install && \
    rm -rf /opencv && \
    rm -rf /opencv_contrib

# Build librealsense w/ CUDA
RUN git clone https://github.com/IntelRealSense/librealsense && \
    cd /librealsense && git checkout v2.52.1 && \
    mkdir build && cd build && \
    cmake -DBUILD_WITH_CUDA:BOOL=TRUE -DFORCE_RSUSB_BACKEND:BOOL=TRUE ../ && \
    make -j$NJOBS install && \
    rm -rf /librealsense

# Build ViSP
RUN git clone https://github.com/lagadic/visp && \
    cd /visp && git checkout v3.5.0 && \
    mkdir build && cd build && \
    cmake -DUSE_YARP:BOOL=OFF -DBUILD_DEMOS:BOOL=OFF -DBUILD_EXAMPLES:BOOL=OFF -DBUILD_TESTS:BOOL=OFF -DBUILD_TUTORIALS:BOOL=ON ../ && \
    make -j$NJOBS install
    # we need built tutorials for some scripts of realsense-holder-calibration, hence no removal here

# Build realsense-holder-calibration
RUN git clone https://github.com/robotology/realsense-holder-calibration && \
    cd realsense-holder-calibration && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local ../ && \
    make -j$NJOBS install && \
    rm -rf /realsense-holder-calibration

# Build robotology-superbuild
RUN git config --global user.name "user" && \
    git config --global user.email "user@email.com" && \
    cd robotology-superbuild && \
    git checkout v2022.09.0 && \
    mkdir build && cd build && \
    cmake -DROBOTOLOGY_ENABLE_CORE=ON -DROBOTOLOGY_USES_GAZEBO=OFF -DROBOTOLOGY_USES_PYTHON=ON -DROBOTOLOGY_USES_LUA=ON -DYCM_EP_ADDITIONAL_CMAKE_ARGS:STRING="-DICUB_COMPILE_BINDINGS:BOOL=ON -DCREATE_PYTHON:BOOL=ON -DENABLE_yarpmod_RGBDSensorWrapper:BOOL=ON -DENABLE_yarpmod_RGBDSensorClient:BOOL=ON -DENABLE_yarpcar_mjpeg:BOOL=ON -DENABLE_yarppm_depthimage_to_rgb:BOOL=ON" ../ && \
    make -j$NJOBS

# Build bayes-filters-lib
RUN git clone https://github.com/robotology/bayes-filters-lib && \
    cd bayes-filters-lib && git checkout devel && mkdir build && cd build && \
    cmake .. && \
    make -j$NJOBS install && \
    rm -rf /bayes-filters-lib

# Build superimpose-mesh-lib
RUN source /robotology-superbuild/build/install/share/robotology-superbuild/setup.sh && \
    git clone https://github.com/robotology/superimpose-mesh-lib && \
    cd superimpose-mesh-lib && git checkout devel && mkdir build && cd build && \
    cmake .. && \
    make -j$NJOBS install && \
    rm -rf /superimpose-mesh-lib

# Build RobotsIO
RUN source /robotology-superbuild/build/install/share/robotology-superbuild/setup.sh && \
    git clone https://github.com/xenvre/robots-io && \
    cd robots-io && mkdir build && cd build && \
    cmake -DUSE_YARP:BOOL=ON .. && \
    make -j$NJOBS install && \
    rm -rf /robots-io

# Build ROFT
RUN source /robotology-superbuild/build/install/share/robotology-superbuild/setup.sh && \
    git clone https://github.com/hsp-iit/roft && \
    cd roft && git checkout impl/online && mkdir build && cd build && \
    cmake .. && \
    make -j$NJOBS install && \
    rm -rf /roft

# Create user with passwordless sudo
RUN useradd -l -G sudo -md /home/user -s /bin/bash -p user user && \
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Switch to user
USER user

# Configure emacs
RUN echo "(setq-default indent-tabs-mode nil)" >> /home/user/.emacs.el && \
    echo "(setq-default tab-width 4)" >> /home/user/.emacs.el && \
    echo "(setq make-backup-files nil)" >> /home/user/.emacs.el && \
    echo "(setq auto-save-default nil)" >> /home/user/.emacs.el && \
    echo "(setq c-default-style \"linux\"" >> /home/user/.emacs.el && \
    echo "      c-basic-offset 4)" >> /home/user/.emacs.el && \
    echo "(global-subword-mode 1)" >> /home/user/.emacs.el && \
    echo "(add-hook 'before-save-hook 'delete-trailing-whitespace)" >> /home/user/.emacs.el && \
    echo "(custom-set-variables '(custom-enabled-themes '(tango-dark)))" >> /home/user/.emacs.el && \
    echo "(custom-set-faces)" >> /home/user/.emacs.elx

# Build roft-samples
WORKDIR /home/user
RUN source /robotology-superbuild/build/install/share/robotology-superbuild/setup.sh && \
    git clone https://github.com/xenvre/roft-samples && \
    cd /home/user/roft-samples && mkdir build && cd build && \
    cmake -DBUILD_TRACKER:BOOL=ON -DBUILD_REALSENSE:BOOL=ON -DCMAKE_INSTALL_PREFIX=/home/user/roft-samples/install .. && \
    make -j$NJOBS install

# Configure robotology-superbuild and roft-samples
RUN echo "source /robotology-superbuild/build/install/share/robotology-superbuild/setup.sh" >> /home/user/.bashrc
RUN echo "export YARP_DATA_DIRS=${YARP_DATA_DIRS}:/home/user/roft-samples/install/share/ICUBcontrib" >> /home/user/.bashrc
RUN echo "export PATH=${PATH}:/home/user/roft-samples/install/bin" >> /home/user/.bashrc
RUN mkdir /home/user/roft-samples/install/share/ICUBcontrib/applications && \
    cp /home/user/roft-samples/install/share/ICUBcontrib/templates/applications/roft.xml  /home/user/roft-samples/install/share/ICUBcontrib/applications/roft.xml

# Launch bash from /home/user
WORKDIR /home/user
CMD ["bash"]
