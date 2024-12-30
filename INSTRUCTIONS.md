```
g++ main.cpp -o main -I/usr/local/include/opencv4 -L/usr/local/lib     -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui     -lpng -ljpeg -lz -ltiff -lwebp -pthread
```

## OpenCV Build
```
cd /path/to/opencv

mkdir build && cd build

cmake -D WITH_GTK=ON -D CMAKE_BUILD_TYPE=Release ..

make -j$(nproc)

sudo make install
```

## Open3D Build

```
sudo apt install -y libeigen3-dev libglfw3-dev libglew-dev libpng-dev \
libjpeg-dev libtiff-dev libwebp-dev libopenblas-dev python3-dev \
python3-pip

git clone --recursive https://github.com/isl-org/Open3D.git

cd Open3D

mkdir build && cd build

cmake -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DBUILD_EXAMPLES=ON \
      -DBUILD_PYTHON_MODULE=OFF \
      -DBUILD_UNIT_TESTS=OFF \
      -DUSE_SYSTEM_EIGEN3=ON \
      -DUSE_SYSTEM_GLFW=ON ..

make -j$(nproc)

sudo make install

sudo ldconfig
```