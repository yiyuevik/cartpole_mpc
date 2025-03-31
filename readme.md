# For MacOS

因為我手上暫時沒有mac本,所以以下代碼應該是可行，但是我無法試行一次，不保證能完全ok

#### 下載Acados

1. 自行下載，或者使用git clone：

```git clone https://github.com/acados/acados.git```

2. Terminal在acados的地址，輸入以下代碼更新submodule:

    ```git submodule update --recursive --init```

#### CMake 編譯

```
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# 可以添加更多的arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> ,我個人沒有再加，如果想要利用cpu加速，還要加上OpenMP，有需求的話可以google一下argument
make install -j4
```

#### Optional: 我看網路上論壇的推薦是用這個，不要用conda, miniconda, pycharm自帶的高級虛擬環境

```
virtualenv env --python=/usr/bin/python3
source env/bin/activate
```

documentation推薦用的這個

我因為是虛擬機上的linux，所以我自己下載了virtualenv，不知道mac需不需要自行下載這個

#### 編譯Acados

在自己的項目中編譯Acados

```
pip install -e <acados_root>/interfaces/acados_template
```

#### 編譯好後，設置lib的path

```
export DYLD_LIBRARY_PATH =$DYLD_LIBRARY_PATH :"<acados_root>/lib"
export ACADOS_SOURCE_DIR="<acados_root>"
```



#### 此時應該就可以使用了

😎



對了！裡面用了spicy，matplotlib，也記得pip install一下