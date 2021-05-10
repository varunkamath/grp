# Script to install dependencies for grp

# Updating sources

sudo apt-get -y update

# Installing OpenCV dependencies

sudo apt-get install -y build-essential cmake

sudo apt-get install -y qt5-default libvtk6-dev

sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev \
                        libopenexr-dev libgdal-dev

sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev \
                        libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm \
                        libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev

sudo apt-get install -y libtbb-dev libeigen3-dev

# Python
sudo apt-get install -y python-dev  python-tk  pylint  python-numpy  \
                        python3-dev python3-tk pylint3 python3-numpy flake8

# Documentation + getter tools
sudo apt-get install -y doxygen unzip wget

# Installing OpenCV

wget https://github.com/opencv/opencv/archive/4.2.0.zip
unzip 4.2.0.zip && rm 4.2.0.zip
mv opencv-4.2.0 OpenCV


wget https://github.com/opencv/opencv_contrib/archive/4.2.0.zip
unzip 4.2.0.zip && rm 4.2.0.zip
mv opencv_contrib-4.2.0 opencv_contrib
mv opencv_contrib OpenCV

cd OpenCV && mkdir build && cd build

cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON \
      -DWITH_XINE=ON -DENABLE_PRECOMPILED_HEADERS=OFF \
      -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ..

make -j8
sudo make install
sudo ldconfig