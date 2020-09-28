# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%
import os
import meshlabxml as mlx
import sys

file_in = './test_data/duck.ply'
file_out = './data/out.ply'

if len(sys.argv) > 1:
    file_in = sys.argv[1]
    if len(sys.argv) > 2:
        file_out = sys.argv[2]

MESHLABSERVER_PATH = 'C:\\Program Files\\VCG\\MeshLab'
os.environ['PATH'] += os.pathsep + MESHLABSERVER_PATH


# %%
meshlab_script_1 = './meshlab_files/meshlab_pointcloud_downsampling.mlx'
meshlab_script_2 = './meshlab_files/meshlab_mesh_cleanup.mlx'

meshlab_log_1 = './meshlab_files/meshlab_pointcloud_downsampling.log'
meshlab_log_2 = './meshlab_files/meshlab_mesh_cleanup.log'

file_tmp = './data/tmp.ply'

print('Reading file from: %s' % file_in)
print('File will be stored to: %s' % file_out)
print('\n')

file_in_abspath = os.path.join(os.path.dirname(os.path.realpath(file_in)), file_in.split('/')[-1])
file_tmp_abspath = os.path.join(os.path.dirname(os.path.realpath(file_tmp)), file_tmp.split('/')[-1])
file_out_abspath = os.path.join(os.path.dirname(os.path.realpath(file_out)), file_out.split('/')[-1])

mlx.run(file_in=file_in_abspath, file_out=file_tmp_abspath, output_mask='-m fc vc vn', script=meshlab_script_1, log=meshlab_log_1)
mlx.run(file_in=file_tmp_abspath, file_out=file_out_abspath, script=meshlab_script_2, log=meshlab_log_2)

print('Mesh written to: %s' % file_out_abspath)