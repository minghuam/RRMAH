''' Calls sampler, gracut and training exe in a batch manner.
'''
import sys
import os
import shutil
import subprocess

def mkdir_safe(dir_name):
	dst = os.path.abspath(dir_name)
	if not os.path.exists(dst):
		os.makedirs(dst)

def mkdir_safe_empty(dir_name):
	dst = os.path.abspath(dir_name)
	if os.path.exists(dst):
		shutil.rmtree(dst)
	os.makedirs(dst)

def back_up(dir_name, backup_root):
	# create back up folder if not exists
	if not os.path.exists(backup_root):
		try:
			print 'Creating backup directory..'
			os.makedirs(backup_root)
		except:
			print 'Failed to create backup directory!'
			return -1
	# delete back up
	dst_path = os.path.join(backup_root, dir_name)
	if os.path.exists(dst_path):
		print 'Deleting old backups..'
		shutil.rmtree(dst_path)

	# copy data files
	print 'Copying backups..'
	src_path = os.path.abspath(dir_name)
	shutil.copytree(dir_name, dst_path)
	return 0

if __name__ == '__main__':
	if len(sys.argv) < 2:
		print 'missing arguments: dir_name'
		sys.exit(0)

	dir_name = sys.argv[1]
	backup_root = '.backup'
	sampler_exe = 'SampleMask.exe'
	sampler_img_dir = '.sampler_img'
	sampler_msk_dir = '.sampler_msk'
	sampler_depth_dir = '.sampler_depth'
	grabcut_exe = 'GrabCut.exe'
	training_exe = 'TrainObjectDetection.exe'

	# create directoires for sampling
	mkdir_safe_empty(sampler_img_dir);
	mkdir_safe_empty(sampler_msk_dir);
	mkdir_safe_empty(sampler_depth_dir);
	# call sampler exe
	print 'Start sampling...'
	subprocess.call([sampler_exe, sampler_img_dir, sampler_msk_dir, sampler_depth_dir], shell=True)

	# delete masks
	shutil.rmtree(sampler_msk_dir)
	mkdir_safe_empty(sampler_msk_dir)

	# call grab cut exe
	print 'Start grabcut...'
	subprocess.call([grabcut_exe, sampler_img_dir, sampler_msk_dir], shell=True)

	# backup
	print 'Backup old files...'
	back_up(dir_name, backup_root)

	# copy new images/masks
	print 'Copy new files...'
	dst_path = os.path.abspath(dir_name)
	# remove old files
	shutil.rmtree(dst_path)
	# copy new files
	mkdir_safe_empty(dst_path)
	mkdir_safe_empty(os.path.join(dst_path, 'model'))
	shutil.copytree(sampler_img_dir, os.path.join(dst_path, 'image'))
	shutil.copytree(sampler_msk_dir, os.path.join(dst_path, 'mask'))

	# call training 
	print 'Start training...'
	subprocess.call([training_exe, dir_name])