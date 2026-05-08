关于网络配置在/etc/netplan/01-installer-config.yaml中
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: no
      addresses: [192.168.43.180/24]
      gateway4: 192.168.43.1
      nameservers:
        addresses: [192.168.43.1,8.8.8.8]
      access-points:
        "tritonP70":
          password: "20060819"
//这一段为ssh配置，目的是让你在pc中开发树莓派，这个是我的手机热点以及密码，然后树莓派自启动会连到这个wifi上，然后你的pc再连这个wifi就能ssh了


  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.50/24]
      nameservers:
        addresses: [8.8.8.8]
//这段是为mid360配置网址
（netplan的注释格式不是这样写的，我这个是演示）

这个yaml的格式非常变态，要求缩进，而且是两个空格
