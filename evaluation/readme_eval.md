# RbRM Evaluation Code, Data and Output Files

本文档说明本仓库中评估代码、测试数据、RbRM 输入与输出文件的组织方式，以及最小 Conda 环境部署和指标评估命令。

---

## 1. 文件内容说明

本次仓库补充上传的内容主要包括评估代码、部分测试样本的 GT 真值、RbRM 输入实例分割结果，以及 RbRM 输出文件。

### 1.1 评估代码

```text
deviation_analysis/
├── batch_evaluate_sm3.py
└── evaluate_step_metrics.py
└── environment_eval.yml
└── readme_eval.md
```

说明：

- `batch_evaluate_sm3.py`：批量指标评估入口脚本，支持 `scan2cad` 和 `cad2cad` 两种评估模式；
- `evaluate_step_metrics.py`：STEP/B-Rep 读取、拓扑检查、解析几何特征参数评估等辅助模块。

### 1.2 DeepCAD 样本

本仓库补充了 50 个 DeepCAD 样本，包含 GT 真值、GT 实例分割输入以及 RbRM 输出结果。

建议目录结构如下：

```text
data/
└── DeepCAD/
    ├── gt_step/          # DeepCAD GT STEP/B-Rep 真值
    └── gt_seg/           # DeepCAD GT 实例分割结果，作为 RbRM 输入

output/
└── RbRM/
    └── DeepCAD/
        ├── stp/          # RbRM 输出的 STEP/STP 文件
        ├── stl/          # RbRM 输出的 STL 文件
        └── xlsx/         # RbRM 输出的结构化参数/统计文件
```

### 1.3 CADParser 样本

本仓库补充了 55 个 CADParser 样本，包含 GT 真值、GT 实例分割输入以及 RbRM 输出结果。

建议目录结构如下：

```text
data/
└── CADParser/
    ├── gt_step/          # CADParser GT STEP/B-Rep 真值
    └── gt_seg/           # CADParser GT 实例分割结果，作为 RbRM 输入

output/
└── RbRM/
    └── CADParser/
        ├── stp/          # RbRM 输出的 STEP/STP 文件
        ├── stl/          # RbRM 输出的 STL 文件
        └── xlsx/         # RbRM 输出的结构化参数/统计文件
```

### 1.4 Scan 样本

若使用真实扫描样本进行 `scan2cad` 评估，建议目录结构如下：

```text
data/
└── Scan/
    └── gt_pc/            # Scan GT 点云

output/
└── RbRM/
    └── Scan/
        ├── stp/          # RbRM 输出的 STEP/STP 文件，可选
        ├── stl/          # RbRM 输出的 STL 文件
        └── xlsx/         # RbRM 输出的结构化参数/统计文件，可选
```

---

## 2. 环境部署

建议使用独立 Conda 环境运行评估代码，不建议直接使用 `base` 环境。

### 2.1 创建并激活环境

```powershell
conda create -n rbrm-eval python=3.10 -y
conda activate rbrm-eval
```

### 2.2 安装 Conda 依赖

当前测试通过的最小 Conda 依赖如下：

```powershell
conda install numpy=1.26.4 pandas=2.2.2 scipy=1.11.4 tqdm=4.67.1 rtree=1.4.1 pythonocc-core=7.8.1 -y
```

### 2.3 安装 Pip 依赖

```powershell
pip install trimesh==4.6.12 open3d==0.19.0 openpyxl==3.1.5
```

---

## 3. 评估脚本使用方式

评估脚本入口为：

```powershell
python deviation_analysis\batch_evaluate_sm3.py
```

主要参数如下：

```text
--gt_dir          GT 真值目录
--pred_dir        方法输出目录
--mode            评估模式，可选 cad2cad 或 scan2cad
--unit            数据单位，默认 mm
--deflection      STEP 转 STL 的线性离散精度，默认 0.01
--geo_samples     CD/HD/NC/F-score 的表面采样点数，默认 30000
--iou_samples     Monte Carlo IoU 采样点数，默认 10000
--timeout         单个样本评估超时时间，默认 300 秒
--mp_start_method 多进程启动方式，Windows 下推荐 spawn
```

---

## 4. Scan-to-CAD 评估

适用于真实扫描点云与 RbRM 输出模型之间的评估。

使用相对路径示例：

```powershell
python deviation_analysis\batch_evaluate_sm3.py `
  --gt_dir <path_to_scan_gt_point_clouds> `
  --pred_dir <path_to_rbrm_outputs> `  # 支持stl与stp两种格式
  --mode scan2cad
```

---

## 5. DeepCAD 与 CADParser CAD-to-CAD 评估

适用于 DeepCAD 与 CADParser 数据样本 GT STEP/B-Rep 与 RbRM 输出 STEP/STP 文件之间的评估。

```powershell
python deviation_analysis\batch_evaluate_sm3.py `
  --gt_dir <path_to_gt_stp> `
  --pred_dir <path_to_rbrm_outputs>  `
  --mode cad2cad
```

---

## 6. 输出结果

评估完成后，脚本会在 `--pred_dir` 指定目录下生成 CSV 文件。

- `eval_final_*.csv`：逐样本指标结果；
- `eval_summary_*.csv`：汇总指标结果。

---
