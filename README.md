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



开发进度

- [x] 增加 CP2108 USB to Serial 驱动
- [ ] 增加自定义 GPIO 设备节点
- [x] 增加兼容 RTC 设备驱动
- [ ] 增加 SPI 及自定义协议多设备节点