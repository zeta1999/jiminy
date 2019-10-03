## Installation Procedure 

In the docker environment:
```bash
# the environment does not make any sense here, put it in the default one
cd ~/wdc_workspace/build/pc/
mkdir jiminy && cd jiminy
cmake ~/wdc_workspace/src/jiminy -DCMAKE_INSTALL_PREFIX=/install
make && make install
```

