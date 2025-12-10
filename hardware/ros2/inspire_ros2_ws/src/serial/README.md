Adapt to ROS2 foxy.



#### Install

Get the code:

    # open a new terminal 
    cd serial
    mkdir build

Build:

    cd build
    cmake ..
    make

Install:

    sudo make install

**like this**:

<img title="" src="file:///home/lee/serial/images/install_Serial_ubuntu.png" alt="" data-align="inline">



也可以不用 make install ,直接和你想要调用本API的ROS2 节点放在同一个 src 下编译即可。有关于本串口操作包的 API 在线文档如下：



[serial: Serial Library](http://wjwwood.io/serial/doc/1.1.0/index.html)



本共 ROS Package 修改来自：

### License

The MIT License

Copyright (c) 2012 William Woodall, John Harrison

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

### Authors

William Woodall <wjwwood@gmail.com>
John Harrison <ash.gti@gmail.com>

### Contact

William Woodall <william@osrfoundation.org>
