import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2
import ros_numpy

def getSampleArrayFromPointCloud_pcd_from_file (PointCloud, sample_path, layers):
    # Load point cloud from pcd
    cloud = pcl.load_XYZI(sample_path)
    cloud = cloud.to_array()    
    return getSampleArrayFromPointCloud_pcd (PointCloud, cloud, layers)
    
def getSampleArrayFromPointCloud_pcd (PointCloud, cloud, layers):
    # Get points and intensity (remission) from pcd point cloud
    points = cloud[:, 0:3]    # get xyz
    remissions = cloud[:, 3]/255 #normal remissions
    # Set points and remission to PointCloud
    PointCloud.set_points(points, remissions)
    xyz = PointCloud.proj_xyz.copy()
    
    # Get intensity and range from point cloud
    Intensity = PointCloud.proj_remission.copy()
    Range = PointCloud.proj_range.copy()
    
    # Expand dimensions to enable concatenate
    Intensity = np.expand_dims(Intensity,axis=2)
    Range = np.expand_dims(Range,axis=2)
    
    # Concatenate intensity and range according to given layers
    if (layers == 'xyzi'):
        xyzi = np.concatenate((xyz, Intensity),axis=2)
        xyzi = np.swapaxes(xyzi,0,1)
        return xyzi
    if (layers == 'xyzir'):
        xyzir = np.concatenate((xyz, Intensity, Range),axis=2)
        xyzir = np.swapaxes(xyzir,0,1)
        return xyzir
    if (layers == 'ir'):
        ir = np.concatenate((Intensity, Range),axis=2)
        ir = np.swapaxes(ir,0,1)
        return ir
    if (layers == 'xyz'):
        xyz = np.swapaxes(xyz,0,1)
        return xyz

    raise NameError('ERROR - Unknown layer type. Aborting!')

def array_to_PointCloud2(cloud, header):
    data = np.zeros(cloud.shape[0], dtype=[
      ('x', np.float32),
      ('y', np.float32),
      ('z', np.float32),
      ('intensity', np.float32)
    ])
    data['x'] = cloud[:,0]
    data['y'] = cloud[:,1]
    data['z'] = cloud[:,2]
    data['intensity'] = cloud[:,3]
    
    cloud_msg = ros_numpy.msgify(PointCloud2, data)
    cloud_msg.header = header
    
    return cloud_msg