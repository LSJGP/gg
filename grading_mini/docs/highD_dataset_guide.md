# highD 数据集获取与使用指南

## 数据集简介

highD 是由 RWTH Aachen 大学汽车工程研究所（ika）发布的高速公路无人机交通轨迹数据集。
使用无人机从空中采集，通过计算机视觉算法自动提取每辆车的完整轨迹，定位误差通常小于 10 厘米。

| 项目 | 值 |
|------|-----|
| 录制场景数 | 60 个 |
| 采集位置 | 6 段德国高速公路 |
| 帧率 | 25 Hz |
| 总录制时长 | 16.5 小时 |
| 平均单场景时长 | ~17 分钟 |
| 总车辆数 | 110,500 辆 |
| 总行驶距离 | 44,500 公里 |
| 车辆中位可见时长 | ~13.6 秒 |
| 许可 | 非商业免费（需申请） |

## 数据规模

### 帧数

| 维度 | 数值 |
|------|------|
| 全局视频帧数 | ~1,485,000 帧（16.5h × 3600s × 25fps） |
| tracks.csv 总行数 | ~37,000,000 行（110,500 辆车 × 13.6s × 25fps） |
| 单录制场景平均帧数 | ~25,500 帧 |
| 单录制场景 tracks 行数 | ~626,000 行 |
| 单辆车平均帧数 | ~340 帧 |

### 文件结构

每个录制场景包含 4 个文件：

```
XX_recordingMeta.csv   # 录制元信息（帧率、限速、车道线等）
XX_tracksMeta.csv      # 车辆摘要（最大速度、最小TTC、变道次数等）
XX_tracks.csv          # 逐帧轨迹数据（位置、速度、加速度、TTC等）
XX_highway.jpg         # 道路俯瞰图
```

## 数据字段

### tracks.csv（核心数据）

| 字段名 | 说明 | 单位 |
|--------|------|------|
| frame | 当前帧号 | - |
| id | 车辆 ID | - |
| x, y | 车辆边界框左上角坐标 | m |
| width, height | 边界框尺寸（对应车长、车宽） | m |
| xVelocity | 纵向速度（行驶方向） | m/s |
| yVelocity | 横向速度 | m/s |
| xAcceleration | 纵向加速度 | m/s² |
| yAcceleration | 横向加速度 | m/s² |
| dhw | 距前车距离（Distance Headway） | m |
| thw | 与前车的时间间隔（Time Headway） | s |
| ttc | 碰撞时间（Time-to-Collision） | s |
| precedingId | 前车 ID（同车道） | - |
| followingId | 后车 ID（同车道） | - |
| laneId | 当前车道 ID | - |
| left/rightPrecedingId | 左/右车道前方车辆 ID | - |
| left/rightAlongsideId | 左/右车道并排车辆 ID | - |
| left/rightFollowingId | 左/右车道后方车辆 ID | - |

> 注意：TTC 和 THW 为 0 表示没有前车或没有有效值。

### tracksMeta.csv（快速筛选用）

| 字段名 | 说明 | 用途 |
|--------|------|------|
| minTTC | 最小 TTC | 筛选危险驾驶（< 1s） |
| minTHW | 最小 THW | 筛选跟车过近 |
| minDHW | 最小 DHW | 筛选距离过近 |
| maxXVelocity | 最大纵向速度 | 筛选超速 |
| meanXVelocity | 平均纵向速度 | 驾驶风格分析 |
| numLaneChanges | 变道次数 | 筛选频繁变道 |
| class | 车辆类型（Car/Truck） | 分类分析 |

### recordingMeta.csv

| 字段名 | 说明 |
|--------|------|
| frameRate | 帧率（25 Hz） |
| speedLimit | 该路段限速（m/s） |
| upperLaneMarkings | 上方车道线 y 坐标（分号分隔） |
| lowerLaneMarkings | 下方车道线 y 坐标（分号分隔） |
| duration | 录制时长（秒） |

## 如何申请

1. 访问 **https://levelxdata.com/highd-dataset/**
2. 滚动到页面底部的申请表
3. 填写以下信息：
   - Full name（真实姓名）
   - Official address（单位地址）
   - Company / Institute / University（所属机构）
   - Project type（项目类型，如 Scientific Paper / Other）
   - 项目描述（需详细说明用途）
4. 勾选同意非商业使用条款
5. 提交后等待 **1-3 个工作日** 审批
6. 通过后邮件会包含下载链接

### 项目描述模板

```
I am developing an open-source evaluation/grading framework for autonomous 
driving planning systems. The highD dataset will be used to validate safety 
metrics (TTC, collision risk, speed compliance, following distance) against 
real-world naturalistic driving data from highways. The project aims to 
create a public benchmark for driving quality assessment. All results will 
be published as open-source research tools.
```

## 同系列数据集

| 数据集 | 场景 | 规模 | 网站 |
|--------|------|------|------|
| highD | 高速公路 | 110,500 辆车 | levelxdata.com/highd-dataset |
| inD | 城市交叉路口 | 13,500 道路用户 | levelxdata.com/ind-dataset |
| rounD | 环岛 | 13,740 道路用户 | levelxdata.com/round-dataset |
| exiD | 高速匝道 | - | levelxdata.com/exid-dataset |
| uniD | 校园道路 | - | levelxdata.com/unid-dataset |

> 所有数据集均可通过各自网站免费申请（学术/研究用途）。

## 用于 Grading Metric 的场景筛选策略

利用 `tracksMeta.csv` 快速筛选：

### "跑得不好"的车辆

```
minTTC < 1.0          # 危险跟车，碰撞时间不足 1 秒
maxXVelocity > speedLimit  # 超速
numLaneChanges > 3     # 频繁变道
minTHW < 0.5           # 跟车时间间隔过短
```

### "跑得好"的车辆

```
minTTC > 3.0           # 安全跟车
maxXVelocity < speedLimit × 0.95  # 合规速度
numLaneChanges <= 1    # 行驶稳定
minTHW > 1.5           # 充足的安全距离
```

## 引用

```bibtex
@inproceedings{highDdataset,
  title     = {The highD Dataset: A Drone Dataset of Naturalistic Vehicle 
               Trajectories on German Highways for Validation of Highly 
               Automated Driving Systems},
  author    = {Krajewski, Robert and Bock, Julian and Kloeker, Laurent 
               and Eckstein, Lutz},
  booktitle = {2018 21st International Conference on Intelligent 
               Transportation Systems (ITSC)},
  pages     = {2118-2125},
  year      = {2018},
  doi       = {10.1109/ITSC.2018.8569552}
}
```
