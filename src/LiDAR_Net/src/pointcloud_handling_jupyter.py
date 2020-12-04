import numpy as np
import tensorflow as tf

def getSampleArrayFromPointCloud (PointCloud, sample_path, layers):
    # Load point cloud from bin
    PointCloud.open_scan(sample_path)
    
    xyz = PointCloud.proj_xyz.copy()
    xyz[:,:,2] = xyz[:,:,2] + 0 # TODO - adjust to correct kitti height (1.73)

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

def getLabelArrayFromPointCloud(PointCloud, label_path, KittiToProject_LUT, maxvalue):
    # Load point cloud from bin
    PointCloud.open_label(label_path)
    
    # Get labels from point cloud
    Labels = PointCloud.proj_sem_label.copy()
    
    # Map kitti classes to project classes
    Labels = KittiToProject_LUT[Labels]
    Labels = np.swapaxes(Labels,0,1)
    Labels = tf.keras.utils.to_categorical(Labels,num_classes=maxvalue+1)
    return Labels
    