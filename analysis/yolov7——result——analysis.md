# 关于yolov7训练结果的文件解析

## 一、weights

也就是训练好的模型，一般使用best.pt去进行推理

 包括best.pt和last.pt，以及默认25epoch保存一次模型，以及保存最后5个epoch的模型

![](C:\Users\31919\Pictures\Screenshots\屏幕截图(97).png)

### 二、 confusion_matrix.png

混淆矩阵

![](C:\Users\31919\Desktop\train\exp\confusion_matrix.png)





混淆矩阵以矩阵形式将数据集中的记录按照真实的类别与分类模型预测的类别判断两个标准进行汇总。其中矩阵的行表示真实值，矩阵的列表示预测值。

TP（True Positive）: 将正类预测为正类数 即正确预测，真实为0，预测也为0

FN （False Negative）：将正类预测为负类 即错误预测，真实为0，预测为1

FP（False Positive）：将负类预测为正类数 即错误预测， 真实为1，预测为0

TN （True Negative）：将负类预测为负类数，即正确预测，真实为1，预测也为1

精确率和召回率的计算方法

精确率Precision=TP / (TP+FP), 在预测是Positive所有结果中，预测正确的比重

召回率recall=TP / (TP+FN), 在真实值为Positive的所有结果中，预测正确的比重

##  三、F1_curve.png

![](C:\Users\31919\Desktop\train\exp\F1_curve.png)

F1分数，它被定义为查准率和召回率的调和平均数

一些多分类问题的机器学习竞赛，常常将F1-score作为最终测评的方法。它是精确率和召回率的调和平均数，最大为1，最小为0。

F1-Score的值是从0到1的，1是最好，0是最差。





 这是100epoch得到的F1_curve，说明在置信度为0.4-0.6区间内得到比较好的F1分数

## 四、hyp.yaml和opt.yaml

训练时的超参数以及train.py中间的参数

## 五、P_curve.png

准确率precision和置信度confidence的关系图

![](C:\Users\31919\Desktop\train\exp\P_curve.png)

##  六、PR_curve.png

![](C:\Users\31919\Desktop\train\exp\PR_curve.png)


PR曲线中的P代表的是precision（精准率），R代表的是recall（召回率），其代表的是精准率与召回率的关系，一般情况下，将recall设置为横坐标，precision设置为纵坐标。PR曲线下围成的面积即AP，所有类别AP平均值即Map.

如果PR图的其中的一个曲线A完全包住另一个学习器的曲线B，则可断言A的性能优于B，当A和B发生交叉时，可以根据曲线下方的面积大小来进行比较。一般训练结果主要观察精度和召回率波动情况（波动不是很大则训练效果较好）

Precision和Recall往往是一对矛盾的性能度量指标；及一个的值越高另一个就低一点；
提高Precision <==> 提高二分类器预测正例门槛 <==> 使得二分类器预测的正例尽可能是真实正例；
提高Recall <==> 降低二分类器预测正例门槛 <== >使得二分类器尽可能将真实的正例挑选

## 七、R_curve.png

![](C:\Users\31919\Desktop\train\exp\R_curve.png)

召回率recall和置信度confidence之间的关系



##  八、results.png

![](C:\Users\31919\Desktop\train\exp\results.png)

Box：Box推测为GIoU损失函数均值，越小方框越准；
Objectness：推测为目标检测loss均值，越小目标检测越准；
Classification：推测为分类loss均值，越小分类越准，本实验为一类所以为0；
Precision：精度（找对的正类/所有找到的正类）；

Recall：真实为positive的准确率，即正样本有多少被找出来了（召回了多少）。

Recall从真实结果角度出发，描述了测试集中的真实正例有多少被二分类器挑选了出来，即真实的正例有多少被该二分类器召回。

val BOX:  验证集bounding box损失

val Objectness：验证集目标检测loss均值

val classification：验证集分类loss均值，本实验为一类所以为0

mAP是用Precision和Recall作为两轴作图后围成的面积，m表示平均，@后面的数表示判定iou为正负样本的阈值，@0.5:0.95表示阈值取0.5:0.05:0.95后取均值。

mAP@.5:.95（mAP@[.5:.95]）
表示在不同IoU阈值（从0.5到0.95，步长0.05）（0.5、0.55、0.6、0.65、0.7、0.75、0.8、0.85、0.9、0.95）上的平均mAP。

mAP@.5：表示阈值大于0.5的平均mAP

一般训练结果主要观察精度和召回率波动情况（波动不是很大则训练效果较好）
然后观察mAP@0.5 & mAP@0.5:0.95 评价训练结果。

## 九、results.txt

![](C:\Users\31919\Pictures\Screenshots\屏幕截图(98).png)


 ![](C:\Users\31919\Pictures\Screenshots\屏幕截图(99).png)分别的含义是训练次数、GPU消耗、训练集边界框损失、训练集目标检测损失、训练集分类损失、训练集总损失、targets目标、输入图片大小、Precision、Recall、mAP@.5、mAP@.5:.95、验证集边界框损失、验证集目标检测损失、验证机分类损失

## 十、train_batchx

我设置的batch_size为8所以一次读取8张图片



##  十一、test_batchx_labels

验证集第一轮的实际标签



## 十二、运行tensorboard

```
activate yolov7(自己所配的环境名称)
tensorboard --logdir=训练结果所在的文件夹
```

