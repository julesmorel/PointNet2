from path import Path
import tempfile
import logging
import pdal
import numpy as np
import subprocess
import glob

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        # logging.FileHandler("debug.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('slim')


def bbox_to_pdal_bounds(bbox):
    bbox = np.array(bbox)
    return f'([{bbox[0, 0]}, {bbox[1, 0]}],[{bbox[0, 1]}, {bbox[1, 1]}])'

def classify(las_list, dtm_tif, tree_list, limit_z=0.1, distance_wood=.03, bbox=[[-20,-20],[20, 20]]):
    '''Assign class=2 to the ground points, class=3 to the wood points and class=1 to the remaining point in the .laz files provided

    Parameters
    ----------
    las_list : list of string
        the LAZ files to classify
    dtm_tif : string
        The DTM raster in the tif format
    tree_list : list of string
        the ASCII files containing every trees point clouds
    limit_z : float
        the limit along the Z axis for the ground classification
    distance_wood : float
        the distance threshold for the wood classification    
    bbox : list of list of float
        bounding box considered for the crop, in the format [[xmin,ymin],[xmax, ymax]]                
    '''

    bounds = bbox_to_pdal_bounds(bbox)
    las_list = [Path(f) for f in las_list]
    tree_list = [Path(f) for f in tree_list]

    with tempfile.TemporaryDirectory() as tempdir:
        tmpdir = Path(tempdir)

        #Concatenate all the files containing the tree point clouds
        all_trees = tmpdir / 'alltrees.xyz'
        with open(all_trees, "wb") as outfile:
            for f in tree_list:
                with open(f, "rb") as infile:
                    outfile.write(infile.read())

        for file in las_list:

            #Assign class=2 to every point within 10cm to the DTM, class=1 otherwise
            ground_classified_file = tmpdir / file.name.stripext() + '_ground.laz'
            crop_stages = [pdal.Reader.las(file),
            pdal.Filter.crop(bounds=bounds),
            pdal.Filter.hag_dem(raster=dtm_tif),
            pdal.Filter.assign(value=["Classification = 2 WHERE HeightAboveGround < 0.1","Classification = 1 WHERE HeightAboveGround >= 0.1"]),
            pdal.Writer.las(ground_classified_file)]
            pdal.Pipeline(crop_stages).execute()

            #Assing class=3 to the points close to the trees points
            classified_file = tmpdir / file.name.stripext() + '_classified.laz'
            cmd = ['./classification/classification', ground_classified_file, all_trees, classified_file, str(distance_wood)]
            print(' '.join(cmd))
            subprocess.check_call(' '.join(cmd), shell=True)




