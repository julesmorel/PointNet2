from path import Path
import tempfile
import logging
import pdal
import numpy as np
import subprocess
import click
from ast import literal_eval

#customizeation of click parser to retrieve list of args
#https://stackoverflow.com/questions/48391777/nargs-equivalent-for-options-in-click
class OptionListArgs(click.Option):

    def __init__(self, *args, **kwargs):
        self.save_other_options = kwargs.pop('save_other_options', True)
        nargs = kwargs.pop('nargs', -1)
        assert nargs == -1, 'nargs, if set, must be -1 not {}'.format(nargs)
        super(OptionListArgs, self).__init__(*args, **kwargs)
        self._previous_parser_process = None
        self._eat_all_parser = None

    def add_to_parser(self, parser, ctx):

        def parser_process(value, state):
            # method to hook to the parser.process
            done = False
            value = [value]
            if self.save_other_options:
                # grab everything up to the next option
                while state.rargs and not done:
                    for prefix in self._eat_all_parser.prefixes:
                        if state.rargs[0].startswith(prefix):
                            done = True
                    if not done:
                        value.append(state.rargs.pop(0))
            else:
                # grab everything remaining
                value += state.rargs
                state.rargs[:] = []
            value = tuple(value)

            # call the actual process
            self._previous_parser_process(value, state)

        retval = super(OptionListArgs, self).add_to_parser(parser, ctx)
        for name in self.opts:
            our_parser = parser._long_opt.get(name) or parser._short_opt.get(name)
            if our_parser:
                self._eat_all_parser = our_parser
                self._previous_parser_process = our_parser.process
                our_parser.process = parser_process
                break
        return retval

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        # logging.FileHandler("debug.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('slim')

@click.command()
@click.option('--las_list', required=True, cls=OptionListArgs, help='the LAZ files to classify.')
@click.option('--dtm_tif', required=True, type=click.Path(exists=True, file_okay=True, dir_okay=False), help='The DTM raster in the tif format.')
@click.option('--tree_list', required=True, cls=OptionListArgs, help='The ASCII files containing every trees point clouds.')
@click.option('--limit_z', type=float, default=0.1, help='The limit along the Z axis for the ground classification')
@click.option('--distance_wood', type=float, default=0.03, help='The distance threshold for the wood classification')
@click.option('--bbox', type=str, default="([-20,20],[-20,20])", help='The bounding box considered for the crop in the format: ([XMIN,XMAX],[YMIN,YMAX])')
def classify(las_list, dtm_tif, tree_list, limit_z, distance_wood, bbox):
    """Assign class=2 to the ground points, class=3 to the wood points and class=1 to the remaining point in the .laz files provided"""

    las_list = [Path(f) for f in literal_eval(las_list)]
    tree_list = [Path(f) for f in literal_eval(tree_list)]

    with tempfile.TemporaryDirectory() as tempdir:
        tmpdir = Path(tempdir)

        #Concatenate all the files containing the tree point clouds
        all_trees = tmpdir / 'alltrees.xyz'
        with open(all_trees, "wb") as outfile:
            for f in tree_list:
                print("f "+str(f))
                with open(f, "rb") as infile:
                    outfile.write(infile.read())

        for file in las_list:

            #Assign class=2 to every point within 10cm to the DTM, class=1 otherwise
            ground_classified_file = tmpdir / file.name.stripext() + '_ground.laz'
            crop_stages = [pdal.Reader.las(file),
            pdal.Filter.crop(bounds=bbox),
            pdal.Filter.hag_dem(raster=dtm_tif),
            pdal.Filter.assign(value=["Classification = 2 WHERE HeightAboveGround < "+str(limit_z),"Classification = 1 WHERE HeightAboveGround >= "+str(limit_z)]),
            pdal.Writer.las(ground_classified_file)]
            pdal.Pipeline(crop_stages).execute()

            #Assing class=3 to the points close to the trees points
            classified_file = tmpdir / file.name.stripext() + '_classified.laz'
            cmd = ['./classification/classification', ground_classified_file, all_trees, classified_file, str(distance_wood)]
            print(' '.join(cmd))
            subprocess.check_call(' '.join(cmd), shell=True)

if __name__ == '__main__':
    classify()    

