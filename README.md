# Azure Kinect DK Code Samples to python

This GitHub repository is forked from the [Azure Kinect DK Code Sample](https://github.com/microsoft/Azure-Kinect-Samples) of MICROSOFT. 
It implements a Socket server to send data from the simple_3D_viewer app into a Python client. (example in the [posture app repo](https://github.com/Parrotlife/posture-app)).

For more information about the Azure Kinect DK and available documentation, see [Microsoft doc](https://azure.microsoft.com/services/kinect-dk/)

## Building

Due to the samples being provided from various sources they may only build for Windows or Linux. To keep the barrier for adding new samples low we only require that the sample works in one place.

### Windows

If the project has Visual Studio solution open it and build. CMake is not currently supported for Windows, though we support the community expanding on this.

### Linux

For building with Linux, CMake is used.

Install the prerequisites
```
apt install lib4a1.2
apt install libk4abt0.9=0.9.4
apt install libxi-dev
```

From the root of the git project
```
mkdir build
cd build
cmake .. -GNinja
ninja
```

an executable is already compiled and available in build/bin.

For connecting to the posture-app, just run the ./simple_3d_viewer executable then run the [posture-app](https://github.com/Parrotlife/posture-app). 

## Microsoft Code of Conduct

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/). For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or contact opencode@microsoft.com with any additional questions or comments.

## License

[MIT License](LICENSE)
