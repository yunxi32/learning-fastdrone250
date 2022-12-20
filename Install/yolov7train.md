# yoloV7 训练

## 主机端训练

### 安装

#### cuda和cudnn

[CUDA Toolkit Archive | NVIDIA Developer](https://developer.nvidia.com/cuda-toolkit-archive)

这里使用的是11.6版本，高版本的cuda导致pytorch不兼容低版本代码，安装时记住安装的路径

[NVIDIA Developer Program Membership Required | NVIDIA Developer](https://developer.nvidia.com/rdp/cudnn-download)

cudnn这里下载需要注册英伟达账号，下载时注意与cuda版本对应

将cudnn的下载文件解压，找到cuda安装路径`C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA`将解压文件中bin、lib、include下的文件复制到cuda对应版本的对应文件夹中

添加以下路径到系统的环境变量中

```
C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.6
C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.6\lib\x64
```

<img src="C:\Users\20826\Documents\GitHub\learning-px4ctrl\img\屏幕截图_20221220_160458.png" alt="屏幕截图_20221220_160458" style="zoom: 80%;" />

<img src="C:\Users\20826\Documents\GitHub\learning-px4ctrl\img\屏幕截图_20221220_160509.png" alt="屏幕截图_20221220_160509" style="zoom:80%;" />

<img src="C:\Users\20826\Documents\GitHub\learning-px4ctrl\img\屏幕截图_20221220_160518.png" alt="屏幕截图_20221220_160518" style="zoom:80%;" />

**检测是否安装成功**

在该处`C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.6\extras\demo_suite`执行

```shell
.\bandwidthTest.exe
```

若结果显示pass则表示安装成功

#### pytorch

这里使用anaconda管理环境，anaconda此处下载[Anaconda | The World's Most Popular Data Science Platform](https://www.anaconda.com/)

```shell
conda create -n yolov7 python==3.8
conda activate yolov7
pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu116 -i https://pypi.tuna.tsinghua.edu.cn/simple
```

注意使用conda的时候关闭代理

**测试pytorch是否安装成功**

```python
import torch
print(torch.cuda.is_available())
print(torch.cuda.device_count())
print(torch.cuda.get_device_name(0))
print(torch.cuda.current_device())
```

#### yolov7

**数据集制作**

在[EasyData - 一站式数据处理和服务平台 (baidu.com)](https://ai.baidu.com/easydata/app/dataset/list)网站将拍摄图片标注制作成物体检测数据集（该网站能够进行多人标注）

![微信图片_20221220164434](C:\Users\20826\Documents\GitHub\learning-px4ctrl\img\微信图片_20221220164434.png)

导出为xml格式的数据集，将导出数据集上传到[Roboflow: Give your software the power to see objects in images and video](https://roboflow.com/)

选择数据的预处理以及数据增强

![屏幕截图_20221220_165448](C:\Users\20826\Documents\GitHub\learning-px4ctrl\img\屏幕截图_20221220_165448.png)

导出为yolov7格式

![屏幕截图_20221220_165620](C:\Users\20826\Documents\GitHub\learning-px4ctrl\img\屏幕截图_20221220_165620.png)

它会给你如下代码

```python
!pip install roboflow
from roboflow import Roboflow
rf = Roboflow(api_key="Wk8yTLc8l9ArmQZLpaAS")
project = rf.workspace("deng-qy-stp2x").project("shupian-test")
dataset = project.version(2).download("yolov7")
```

**安装**

克隆yolov7

```shell
git clone https://github.com/WongKinYiu/yolov7
```

安装依赖

```shell
cd yolov7
pip install -r requirements.txt
pip install roboflow
```

在上面的中端中输入python回车

将之前给的代码复制到此处

```python
from roboflow import Roboflow
rf = Roboflow(api_key="Wk8yTLc8l9ArmQZLpaAS")
project = rf.workspace("deng-qy-stp2x").project("shupian-test")
dataset = project.version(2).download("yolov7")
```

这是在yolov7的文件夹中会出现数据集名字的文件夹

下载预训练模型，在浏览器打开https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7_training.pt，将下载的文件放入yolov7文件夹下

### 训练

执行

```python
python train.py --batch 30 --epochs 150 --data {数据集地址}/data.yaml --weights 'yolov7_training.pt' --device 0 
```

参数解释

`batch`表示批量处理大小，根据gpu显存决定，值越大效果越好

`epochs`迭代次数大小,我们测试后最少150次，效果较为理想

`device`0表示使用的显卡编号

### 结果分析


