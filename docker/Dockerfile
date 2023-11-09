FROM pointcloudlibrary/env:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    doxygen-latex \
    dvipng \
    graphviz \
    git \
    python3-pip \
    pandoc \
    unzip \
    git \
    libpcl-dev \
    curl \
    zip \
    tar \
    libjsoncpp-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install Jinja2 sphinx sphinxcontrib-doxylink sphinx_rtd_theme requests grip

# Install OpenCV
WORKDIR /tmp
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
    unzip opencv.zip && \
    mkdir -p build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ../opencv-4.x \
    && make -j$(nproc) && make install && ldconfig \
    && rm -rf /tmp/*

# install jsoncpp
RUN git clone https://github.com/Microsoft/vcpkg.git

WORKDIR /share