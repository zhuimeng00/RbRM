# RbRM Evaluation Code, Data and Output Files

本文档说明本仓库中评估代码、测试数据、RbRM 输入与输出文件的组织方式，以及最小 Conda 环境部署和指标评估命令。

---

## 1. 文件内容说明

本次仓库补充上传的内容主要包括评估代码、部分测试样本的 GT 真值、RbRM 输入实例分割结果，以及 RbRM 输出文件。

### 1.1 评估代码

```text
deviation_analysis/
├── batch_evaluate_sm4.py
├── evaluate_step_metrics.py
├── environment_eval.yml
└── readme_eval.md

evaluation_v2/
├── build_gt_feature_manifest.py
├── prediction_adapters.py
├── join_feature_instances.py
├── metric_definitions.py
└── evaluate_feature_instances.py
```

说明：

- `batch_evaluate_sm4.py`：统一批量评估入口，支持 `scan2cad` 和 `cad2cad` 两种评估模式，并可在 `cad2cad` 模式下调用工程特征级评估；
- `evaluate_step_metrics.py`：提供 SM4 所需的 STEP/B-Rep 读取和拓扑辅助功能；
- `evaluation_v2/`：用于 feature-level consistency 评估的结构化工程特征评估模块。

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

统一评估脚本入口为：

```powershell
python deviation_analysis\batch_evaluate_sm4.py
```

评估结果分为三个互补部分：

1. **Feature-level consistency**
2. **Geometric fidelity**
3. **CAD-solid validity**

### 3.1 Feature-level consistency

Feature-level consistency 用于评估重建工程特征的恢复情况及其 CAD 原生几何参数一致性，包括：

- Feature Recovery (FR)
- Axis Angle Error (AAE)
- Radius Relative Error (RRE)
- Euclidean Center Distance (ECD)
- normalized Euclidean Center Distance (nECD)

其评估流程由 `evaluation_v2/` 中的模块共同完成：

- `build_gt_feature_manifest.py`：构建 CAD-derived GT feature manifest，并记录 feature identity、CAD-native parameters 和 metric eligibility；
- `prediction_adapters.py`：将结构化重建结果转换为统一的 prediction records；
- `join_feature_instances.py`：依据持久化 feature identity 建立 prediction 与 GT 的对应关系，不使用 prediction-to-GT 几何匹配或事后分配；
- `metric_definitions.py`：定义 feature-level 指标和适用性规则；
- `evaluate_feature_instances.py`：基于 identity-joined records 计算 FR、AAE、RRE、ECD 和 nECD。

对于没有唯一 CAD-native GT 参数的特征，仅将对应参数指标记为不适用；缺失的重建特征仍保留在 FR 分母中。

### 3.2 Geometric fidelity

Geometric fidelity 用于评估重建模型与参考几何之间的表面一致性，包括：

- Chamfer Distance (CD)
- Hausdorff Distance (HD)
- Normal Consistency (NC)
- F-score

### 3.3 CAD-solid validity

CAD-solid validity 用于评估输出可用性、mesh-domain 完整性以及 STEP/B-Rep CAD-solid 有效性，包括：

- Invalid Ratio (IR)
- Mesh Watertightness Rate (M-WR)
- Mesh IoU (M-IoU)
- Solid Validity Rate (S-VR)
- Solid IoU (S-IoU)

主要参数如下：

```text
--gt_dir          GT 真值目录
--pred_dir        方法输出目录
--mode            评估模式，可选 cad2cad 或 scan2cad
--unit            数据单位/坐标设置
--deflection      STEP 转 STL 的线性离散精度，默认 0.01
--geo_samples     CD/HD/NC/F-score 的表面采样点数，默认 30000
--iou_samples     Monte Carlo IoU 采样点数，默认 10000
--timeout         单个样本评估超时时间，默认 300 秒
--mp_start_method 多进程启动方式，Windows 下推荐 spawn
--enable_ev2      启用 feature-level consistency 评估
--ev2_dataset     EV2 数据集，可选 CADParser 或 DeepCAD
--ev2_gt_manifest GT feature manifest CSV
--ev2_xlsx_root   结构化工程特征输出目录
--ev2_out_dir     EV2 输出目录
```

---

## 4. Scan-to-CAD 评估

适用于真实扫描点云与 RbRM 输出模型之间的评估。

使用相对路径示例：

```powershell
python deviation_analysis\batch_evaluate_sm4.py `
  --gt_dir <path_to_scan_gt_point_clouds> `
  --pred_dir <path_to_rbrm_outputs> `
  --mode scan2cad
```

该模式主要报告 observation-domain geometric fidelity 和适用的输出有效性指标。由于真实扫描样本缺少对应 STEP-level CAD ground truth 和 CAD-derived feature annotations，不运行 feature-level consistency 或 solid-volume overlap 指标。

---

## 5. DeepCAD 与 CADParser CAD-to-CAD 评估

适用于 DeepCAD 与 CADParser 数据样本 GT STEP/B-Rep 与 RbRM 输出 STEP/STP 文件之间的统一评估。

以 CADParser 为例：

```powershell
python deviation_analysis\batch_evaluate_sm4.py `
  --gt_dir <path_to_gt_stp> `
  --pred_dir <path_to_rbrm_outputs> `
  --mode cad2cad `
  --unit mm `
  --enable_ev2 `
  --ev2_dataset CADParser `
  --ev2_gt_manifest <path_to_gt_feature_manifest.csv> `
  --ev2_xlsx_root <path_to_structured_feature_outputs> `
  --ev2_out_dir <path_to_ev2_output>
```

该命令统一生成三类评估结果：

- **Feature-level consistency**：FR、AAE、RRE、ECD、nECD；
- **Geometric fidelity**：CD、HD、NC、F-score；
- **CAD-solid validity**：IR、M-WR、M-IoU、S-VR、S-IoU。

DeepCAD 使用相同评估流程，并应保持与其官方归一化评估坐标一致的阈值设置。

---

## 6. 输出结果

评估完成后，脚本会在 `--pred_dir` 指定目录及 EV2 输出目录下生成评估文件。

主要输出包括：

- `eval_final_*.csv`：逐样本 output-level 指标结果；
- `eval_summary_*.csv`：output-level 汇总指标结果；
- `evaluation_summary.csv` / `evaluation_summary.json`：统一评估汇总；
- `run_manifest.json`：运行配置和评估来源记录；
- EV2 输出目录中的 `prediction_records_all.csv`、`identity_join/`、`per_instance_metrics.csv`、`dataset_summary.json` 和 `evaluation_audit.json`：feature-level consistency 结果及审计信息。

Feature-level evaluation 使用持久化 feature identity 和 CAD-native 参数；评价过程不会修改 RbRM 的结构化重建记录。
