``` bash
# install west and pyelftools
python3 -m pip install west
python3 -m pip install pyelftools

cd zephyr
# fetch the zephyr using manifest and prepare the project
west init --mf west.yml build && cd build
# update source and dependencies
west update
# export zephyr environements
west zephyr-export
# install python requirements
python3 -m pip install $(west packages pip)
# install required sdk's for different cpu's, maybe get some errors with
# wget arguments you just need to modify the called script, or specifying 
# the target sdk you need instead of intalling all the supported sdk's.
west sdk install
# build the project
west build -p auto -b samd21 ../app -DBOARD_ROOT=${PWD}/../app
#flash to your board
west flash
```