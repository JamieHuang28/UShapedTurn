# UShapedTurn
a demo for u shaped turn

typical u turn is decided by: 1, the ending point; 2, the target lane width; 3, the foward space

## setup
install conda and create python3.7 environment
```
conda create --name py37 python=3.7
conda activate py37
```
install mini-forge
```
wget -O Miniforge3.sh "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3.sh -bp "${HOME}/conda"
```
install dependencies
```
conda install --yes --file requirements.txt
```

install pybind11
```
sudo apt update && sudo apt install pybind11-dev
```

## run
```
bokeh serve main.py
```
and open the link in browser

## test
please refer to [gtest-demo](https://github.com/yecharlie/gtest-demo) when issue occurs
```
./tests/unittest
```