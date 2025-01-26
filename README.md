``` bash
python3 -m pip install west
python3 -m pip install pyelftools

cd zephyr
west init -mf west.yml
west update
west build -p auto -b samd21 ../philly -DBOARD_ROOT=${PWD}/../philly
```