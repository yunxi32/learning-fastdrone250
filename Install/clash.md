## CLASH FOR WINDOWS 如何在Linux上使用

### # 1. 安装clash for windows

点开链接,选择clash for windows的linux版本

[clash]: https://github.com/Fndroid/clash_for_windows_pkg/releases/tag/0.17.

![](img/QQ图片20221216185120.jpg)

### 打开clash

![](img/QQ图片20221216185124.jpg)

`cd clash`

`./cfw`

设置clah

![](../../25895336-9667850b01c82b6c.webp)

### 修改配置文件



`sodo gedit /etc/environment`

### 添加以下内容并保存

    `http_proxy=http://127.0.0.1:7890/ https_proxy=http://127.0.0.1:7890/ ftp_proxy=http://127.0.0.1:7890/ HTTP_PROXY=http://127.0.0.1:7890/ HTTPS_PROXY=http://127.0.0.1:7890/ FTP_PROXY=http://127.0.0.1:7890/`

### 保存

