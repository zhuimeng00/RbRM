import sys
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import KDTree

# PythonOCC Imports
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_SOLID, TopAbs_SHELL
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Cone, GeomAbs_Sphere, GeomAbs_Torus
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.TopoDS import topods
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCC.Core.TopExp import topexp
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
from OCC.Core.gp import gp_Pnt
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
from OCC.Core.ShapeFix import ShapeFix_Solid

class StepLoader:
    @staticmethod
    def load(filepath):
        # 增加静默模式，防止OCC读取时的控制台噪音
        reader = STEPControl_Reader()
        status = reader.ReadFile(filepath)
        if status != 1:
            raise FileNotFoundError(f"Cannot read STEP file: {filepath}")
        reader.TransferRoots()
        return reader.OneShape()

class VolumetricEvaluator:
    """
    IoU 计算器 (蒙特卡洛法)
    """
    def __init__(self, shape_gt, shape_pred):
        self.shape_gt = self._ensure_solid(shape_gt)
        self.shape_pred = self._ensure_solid(shape_pred)

    def _ensure_solid(self, shape):
        """鲁棒的实体转换逻辑"""
        if shape.IsNull(): return shape
        # 1. 已经是Solid，尝试ShapeFix修复潜在的拓扑错误（如翻转面）
        if shape.ShapeType() == TopAbs_SOLID:
            fixer = ShapeFix_Solid(topods.Solid(shape))
            fixer.Perform()
            return fixer.Solid()
        # 2. 是Shell，尝试闭合为Solid
        if shape.ShapeType() == TopAbs_SHELL:
            mk_solid = BRepBuilderAPI_MakeSolid(topods.Shell(shape))
            if mk_solid.IsDone(): return mk_solid.Solid()
        # 3. 遍历Compound寻找
        exp = TopExp_Explorer(shape, TopAbs_SOLID)
        if exp.More(): return topods.Solid(exp.Current())
        exp_sh = TopExp_Explorer(shape, TopAbs_SHELL)
        if exp_sh.More():
            mk_solid = BRepBuilderAPI_MakeSolid(topods.Shell(exp_sh.Current()))
            if mk_solid.IsDone(): return mk_solid.Solid()
        return shape

    def compute_iou(self, num_samples=50000):
        # 如果转换后还不是 Solid，无法计算体积 IoU
        if self.shape_gt.ShapeType() != TopAbs_SOLID or self.shape_pred.ShapeType() != TopAbs_SOLID:
            return 0.0

        bbox_gt = Bnd_Box(); brepbndlib.Add(self.shape_gt, bbox_gt)
        bbox_pred = Bnd_Box(); brepbndlib.Add(self.shape_pred, bbox_pred)
        
        # 如果包围盒无效（空模型），直接返回0
        if bbox_gt.IsVoid() or bbox_pred.IsVoid():
            return 0.0

        bbox_union = Bnd_Box()
        bbox_union.Add(bbox_gt)
        bbox_union.Add(bbox_pred)
        
        xmin, ymin, zmin, xmax, ymax, zmax = bbox_union.Get()
        
        # 扩大采样范围
        diag = np.sqrt((xmax-xmin)**2 + (ymax-ymin)**2 + (zmax-zmin)**2)
        margin = max(diag * 0.05, 1e-3) # 至少留一点边距
        xmin -= margin; xmax += margin
        ymin -= margin; ymax += margin
        zmin -= margin; zmax += margin

        classifier_gt = BRepClass3d_SolidClassifier(self.shape_gt)
        classifier_pred = BRepClass3d_SolidClassifier(self.shape_pred)
        
        inter_count = 0
        union_count = 0
        
        # 向量化生成随机点
        rand_pts = np.random.uniform(0, 1, (num_samples, 3))
        rand_pts[:, 0] = rand_pts[:, 0] * (xmax - xmin) + xmin
        rand_pts[:, 1] = rand_pts[:, 1] * (ymax - ymin) + ymin
        rand_pts[:, 2] = rand_pts[:, 2] * (zmax - zmin) + zmin
        
        for i in range(num_samples):
            pt = gp_Pnt(float(rand_pts[i,0]), float(rand_pts[i,1]), float(rand_pts[i,2]))
            
            # BBox 快速剔除
            in_gt = False
            if not bbox_gt.IsOut(pt): 
                classifier_gt.Perform(pt, 1e-4)
                if classifier_gt.State() in [0, 3]: in_gt = True # 0=IN, 3=ON
            
            in_pred = False
            if not bbox_pred.IsOut(pt):
                classifier_pred.Perform(pt, 1e-4)
                if classifier_pred.State() in [0, 3]: in_pred = True
            
            if in_gt or in_pred: union_count += 1
            if in_gt and in_pred: inter_count += 1

        if union_count == 0: return 0.0
        return inter_count / union_count

class TopologyEvaluator:
    def __init__(self, shape): self.shape = shape
    
    def count_faces(self):
        if self.shape.IsNull(): return 0
        explorer = TopExp_Explorer(self.shape, TopAbs_FACE)
        count = 0
        while explorer.More(): count += 1; explorer.Next()
        return count
    
    def check_watertight(self):
        if self.shape.IsNull(): return False
        # 1. OpenCascade 内置检查
        analyzer = BRepCheck_Analyzer(self.shape)
        if not analyzer.IsValid(): return False
        
        # 2. 严格检查：是否存在只有 1 个面共享的边 (自由边)
        map_edges_faces = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp.MapShapesAndAncestors(self.shape, TopAbs_EDGE, TopAbs_FACE, map_edges_faces)
        
        for i in range(1, map_edges_faces.Size() + 1):
            # 如果一条边只被 < 2 个面拥有，则是裂缝
            if map_edges_faces.FindFromIndex(i).Size() < 2: return False
        return True

class GeometryEvaluator:
    def __init__(self, shape_gt, shape_pred, num_samples=10000):
        # [改进] 动态计算网格精度
        self.deflection_gt = self._calc_deflection(shape_gt)
        self.deflection_pred = self._calc_deflection(shape_pred)
        
        self.pcd_gt = self._sample_points(shape_gt, num_samples, self.deflection_gt)
        self.pcd_pred = self._sample_points(shape_pred, num_samples, self.deflection_pred)
        
    def _calc_deflection(self, shape):
        """根据 BBox 大小动态设定网格精度，避免 CD 计算误差"""
        if shape.IsNull(): return 0.01
        bbox = Bnd_Box()
        brepbndlib.Add(shape, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        diag = np.sqrt((xmax-xmin)**2 + (ymax-ymin)**2 + (zmax-zmin)**2)
        # 设定为对角线的 1/1000，但设有上下限
        return np.clip(diag * 0.001, 1e-4, 0.1)

    def _sample_points(self, shape, num_samples, deflection):
        if shape.IsNull(): return np.zeros((0, 3))
        # 使用动态精度
        mesh = BRepMesh_IncrementalMesh(shape, deflection)
        mesh.Perform()
        
        points = []
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while explorer.More():
            face = topods.Face(explorer.Current())
            loc = TopLoc_Location()
            triangulation = BRep_Tool.Triangulation(face, loc)
            if triangulation:
                transf = loc.Transformation()
                for i in range(1, triangulation.NbNodes() + 1):
                    pnt = triangulation.Node(i)
                    pnt.Transform(transf)
                    points.append([pnt.X(), pnt.Y(), pnt.Z()])
            explorer.Next()
        
        points = np.array(points)
        if len(points) == 0: return np.zeros((num_samples, 3))
        
        # 采样/补齐
        if len(points) >= num_samples:
            return points[np.random.choice(len(points), num_samples, replace=False)]
        else:
            return points[np.random.choice(len(points), num_samples, replace=True)]

    def compute_distances(self):
        # 如果任意一方采样失败（空模型），返回 None
        if len(self.pcd_gt) == 0 or len(self.pcd_pred) == 0: return None, None
        
        tree_gt = KDTree(self.pcd_gt)
        tree_pred = KDTree(self.pcd_pred)
        
        # pred 到 gt 的距离 (Accuracy)
        dist_p_g, _ = tree_gt.query(self.pcd_pred)
        # gt 到 pred 的距离 (Completeness)
        dist_g_p, _ = tree_pred.query(self.pcd_gt)
        
        # CD = (Mean(d_p->g) + Mean(d_g->p)) / 2 (可选除以2，这里保持累加)
        cd = np.mean(dist_p_g) + np.mean(dist_g_p)
        # HD = Max(Max(d_p->g), Max(d_g->p))
        hd = max(np.max(dist_p_g), np.max(dist_g_p))
        
        return cd, hd

class FeatureParamEvaluator:
    def __init__(self, shape_gt, shape_pred):
        self.feats_gt = self._extract_features(shape_gt)
        self.feats_pred = self._extract_features(shape_pred)
        self.matches = []
        
    def _get_axis_vec(self, axis_obj):
        d = axis_obj.Direction()
        v = np.array([d.X(), d.Y(), d.Z()])
        norm = np.linalg.norm(v)
        return v / (norm + 1e-9)

    def _extract_features(self, shape):
        features = []
        if shape.IsNull(): return features
        
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        idx = 0
        while explorer.More():
            face = topods.Face(explorer.Current())
            surf = BRepAdaptor_Surface(face)
            stype = surf.GetType()
            
            # 计算几何中心
            props = GProp_GProps()
            brepgprop.SurfaceProperties(face, props)
            c = props.CentreOfMass()
            center = np.array([c.X(), c.Y(), c.Z()])
            
            feat = None
            if stype == GeomAbs_Cylinder:
                obj = surf.Cylinder()
                feat = {'type': 'Cylinder', 'radius': obj.Radius(), 'axis': self._get_axis_vec(obj.Axis())}
            elif stype == GeomAbs_Plane:
                obj = surf.Plane()
                feat = {'type': 'Plane', 'radius': 0.0, 'axis': self._get_axis_vec(obj.Axis())}
            elif stype == GeomAbs_Cone:
                obj = surf.Cone()
                feat = {'type': 'Cone', 'radius': obj.RefRadius(), 'axis': self._get_axis_vec(obj.Axis()), 'angle': obj.SemiAngle()}
            elif stype == GeomAbs_Sphere:
                obj = surf.Sphere()
                feat = {'type': 'Sphere', 'radius': obj.Radius(), 'axis': None}
            elif stype == GeomAbs_Torus:
                obj = surf.Torus()
                feat = {'type': 'Torus', 'radius': obj.MajorRadius(), 'axis': self._get_axis_vec(obj.Axis()), 'minor_radius': obj.MinorRadius()}

            if feat:
                feat['id'] = idx
                feat['center'] = center
                features.append(feat)
                idx += 1
            explorer.Next()
        return features

    def compute_metrics(self):
        types = ['Cylinder', 'Plane', 'Cone', 'Sphere', 'Torus']
        all_aae, all_rre, all_ecd = [], [], [] # [新增] ECD: Euclidean Center Distance
        
        total_gt = 0
        total_match = 0
        
        for t in types:
            gts = [f for f in self.feats_gt if f['type'] == t]
            preds = [f for f in self.feats_pred if f['type'] == t]
            
            total_gt += len(gts)
            
            if not gts or not preds: continue
            
            # 构建代价矩阵
            cost_mat = np.zeros((len(gts), len(preds)))
            for i, g in enumerate(gts):
                for j, p in enumerate(preds):
                    dist = np.linalg.norm(g['center'] - p['center'])
                    rad_diff = abs(g['radius'] - p['radius'])
                    
                    # 匹配权重：位置优先，半径其次
                    cost = dist * 1.0 + rad_diff * 5.0
                    if t == 'Cone': cost += abs(g['angle'] - p['angle']) * 10.0
                    cost_mat[i, j] = cost

            row_ind, col_ind = linear_sum_assignment(cost_mat)
            thresh = 20.0 # 匹配阈值 (单位同模型单位，建议根据数据集尺度调整)
            
            for r, c in zip(row_ind, col_ind):
                if cost_mat[r, c] < thresh:
                    total_match += 1
                    f_g, f_p = gts[r], preds[c]
                    
                    # 1. ECD (Center Distance)
                    ecd = np.linalg.norm(f_g['center'] - f_p['center'])
                    all_ecd.append(ecd)

                    # 2. AAE (Axis Alignment) - Sphere 无轴
                    if f_g['axis'] is not None and f_p['axis'] is not None:
                        dot = np.abs(np.dot(f_g['axis'], f_p['axis']))
                        angle = np.degrees(np.arccos(np.clip(dot, 0, 1)))
                        all_aae.append(angle)
                    
                    # 3. RRE (Relative Radius) - Plane 无半径
                    if t != 'Plane' and f_g['radius'] > 1e-6:
                        rre = abs(f_p['radius'] - f_g['radius']) / f_g['radius']
                        all_rre.append(rre)

        # 汇总
        metrics = {
            "AAE": np.mean(all_aae) if all_aae else None,
            "RRE": np.mean(all_rre) if all_rre else None,
            "ECD": np.mean(all_ecd) if all_ecd else None,
            "Match_Count": total_match,
            "GT_Count": total_gt,
            "Pred_Count": len(self.feats_pred)
        }
        return metrics
