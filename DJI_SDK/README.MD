### SETUP DJI Onboard SDK on RPI3

Если вы хотите развернуть DJI Onboard SDK вам потребуется следующее:

1. DJI Matrice 100/200/600
2. Raspberry PI 3
3. microSD card on 8 gb or more.

Первостепенно нам потребуется установить образ на Raspberry PI 3

[Ubiquity Robotics IMG RPI3](https://downloads.ubiquityrobotics.com/pi.html) <br />

Инструкция по развертыванию приведена на сайте.

####  Первостепенно включите swap раздел!!!

Далее необходимо подключится по SSH к Raspberry PI и произвести настройку диска. Первостепенно необходимо включить работу swap раздела.

```bash
# dd if=/dev/zero of=/mnt/sda1/swap.file bs=1M count=1024
```

Set the permissions so only root has access. This plugs a security hole.

```bash
# chmod 600 /mnt/sda1/swap.file
```

Next create a swap memory space and turn it on with these commands.

```bash
# mkswap /mnt/sda1/swap.file
# swapon /mnt/sda1/swap.file
```

To make the swap file available on every boot you need to add two lines to the fstab file. The first is the line for attaching your USB hard drive (not shown). The second will mount your swapfile. I added the following to my `/etc/fstab`

```bash
# /mnt/sda1/swap.file none swap defaults 0 0
```
После того как у нас появился swap раздел, мы сможем избежать ошибок с нехваткой оперативной памяти для компиляции DJI SDK.

#### Устанавливаем DJI SDK

Далее производим настройки и установки согласно нижеприведенной инструкции.

[Инструкция по установки](http://wiki.ros.org/dji_sdk/Tutorials/Getting%20Started)




### Links
[DJI SDK](https://developer.dji.com/onboard-sdk/documentation/introduction/homepage.html) <br />
[DJI ROS SDK](http://wiki.ros.org/dji_sdk) <br />
