# Airbot Remote

## Install

``` bash
conda create -n frrobot python=3.10.14
conda activate frrobot
pip3 install scipy
sudo apt install libfmt-dev
sudo apt install libspdlog-dev
sudo apt install ./packages/airbot_play_2.9.0_amd64.deb
python3 python/setup.py install
```

## Start

``` bash
python3 test_encoder.py
```

