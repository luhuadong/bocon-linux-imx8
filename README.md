# bocon-linux-imx8

Prepare enviroment

```shell
$ source env.sh
```

Build

```shell
make k37x_defconfig   # 重新配置
make Image            # 编译内核
make dtbs             # 编译设备树
make modules          # 编译内核模块
```
