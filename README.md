# roft-samples

A suite of applications based on ROFT

### How to build
```console
git clone https://github.com/xenvre/roft-samples
mkdir build
cd build
cmake -DBUILD_TRACKER:BOOL=ON -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> ..
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:<INSTALL_DIR>/share/ICUBcontrib
export PATH=${PATH}:<INSTALL_DIR>/bin
cp <INSTALL_DIR>/share/ICUBcontrib/templates/applications/roft.xml  <INSTALL_DIR>/share/ICUBcontrib/applications/roft.xml
```
