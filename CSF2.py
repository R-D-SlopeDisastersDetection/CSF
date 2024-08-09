import string

import laspy
import numpy as np
import CSF
import open3d as o3d


class CSF2(object):

    def __init__(self, inputfile: string, outputfile: string, filetype: string,
                 bSloopSmooth: bool = False, cloth_resolution: float = 0.5, rigidness: float = 3,
                 time_step: float = 0.65, class_threshold: float = 0.03, interations: int = 500):
        self.csf = CSF.CSF()
        self.inputfile = inputfile
        self.outputfile = outputfile
        self.filetype = filetype
        self.bSloopSmooth = bSloopSmooth
        self.cloth_resolution = cloth_resolution
        self.rigidness = rigidness
        self.time_step = time_step
        self.class_threshold = class_threshold
        self.interations = interations
        self.csf.params.bSloopSmooth = self.bSloopSmooth
        self.csf.params.cloth_resolution = self.cloth_resolution
        self.csf.params.rigidness = self.rigidness
        self.csf.params.time_step = self.time_step
        self.csf.params.class_threshold = self.class_threshold
        self.csf.params.interations = self.interations
        self.ground = CSF.VecInt()
        self.non_ground = CSF.VecInt()
        
    def process(self):
        if self.filetype == 'las':
            self.las_process()


    def view_cloud(self):
        if self.filetype=='las':
            self.las_view_cloud()


    def las_process(self):
        inFile = laspy.read(self.inputfile)
        points = inFile.points
        xyz = np.vstack((inFile.x, inFile.y, inFile.z)).transpose()

        self.csf.setPointCloud(xyz)
        self.csf.do_filtering(self.ground, self.non_ground)

        outFile = laspy.LasData(inFile.header)
        outFile.points = points[np.array(self.ground)]
        outFile.write(self.outputfile)


    def las_view_cloud(self):
        las = laspy.read(self.outputfile)
        points = np.stack([las.x, las.y, las.z]).transpose()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)


        if las.red.max() >= 256: las.red >>= 8
        if las.green.max() >= 256: las.green >>= 8
        if las.blue.max() >= 256: las.blue >>= 8
        colors = np.stack([las.red * 256 / 65535, las.green * 256 / 65535, las.blue * 256 / 65535], axis=0).transpose(
            (1, 0))
        pcd.colors = o3d.utility.Vector3dVector(colors)

        o3d.visualization.draw_geometries([pcd])


    def set_inputfile(self, inputfile: str):
        self.inputfile = inputfile

    def get_inputfile(self):
        return self.inputfile

    def set_outputfile(self, outputfile: str):
        self.outputfile = outputfile

    def get_outputfile(self):
        return self.outputfile

    def set_filetype(self, filetype):
        self.filetype = filetype

    def get_filetype(self):
        return self.filetype

    def set_bSloopSmooth(self, bSloopSmooth: bool):
        self.bSloopSmooth = bSloopSmooth

    def get_bSloopSmooth(self):
        return self.bSloopSmooth

    def set_cloth_resolution(self, cloth_resolution: float):
        self.cloth_resolution = cloth_resolution

    def get_cloth_resolution(self):
        return self.cloth_resolution

    def set_rigidness(self, rigidness: float):
        self.rigidness = rigidness

    def get_rigidness(self):
        return self.rigidness

    def set_time_step(self, time_step: float):
        self.time_step = time_step

    def get_time_step(self):
        return self.time_step

    def set_class_threshold(self, class_threshold: float):
        self.class_threshold = class_threshold

    def get_class_threshold(self):
        return self.class_threshold

    def set_interations(self, interations: int):
        self.interations = interations

    def get_interations(self):
        return self.interations