from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape


class StepLoader:
    """Load STEP/B-Rep files with OpenCascade."""

    @staticmethod
    def load(filepath):
        reader = STEPControl_Reader()
        status = reader.ReadFile(filepath)
        if status != 1:
            raise FileNotFoundError(f"Cannot read STEP file: {filepath}")
        reader.TransferRoots()
        return reader.OneShape()


class TopologyEvaluator:
    """Basic STEP/B-Rep topology utilities used by the batch evaluator."""

    def __init__(self, shape):
        self.shape = shape

    def count_faces(self):
        if self.shape.IsNull():
            return 0

        explorer = TopExp_Explorer(self.shape, TopAbs_FACE)
        count = 0
        while explorer.More():
            count += 1
            explorer.Next()
        return count

    def check_watertight(self):
        if self.shape.IsNull():
            return False

        analyzer = BRepCheck_Analyzer(self.shape)
        if not analyzer.IsValid():
            return False

        map_edges_faces = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp.MapShapesAndAncestors(
            self.shape,
            TopAbs_EDGE,
            TopAbs_FACE,
            map_edges_faces,
        )

        for i in range(1, map_edges_faces.Size() + 1):
            if map_edges_faces.FindFromIndex(i).Size() < 2:
                return False

        return True

