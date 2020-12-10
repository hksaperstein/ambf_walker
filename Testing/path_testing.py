from os.path import dirname, join

project_root = dirname(dirname(__file__))
output_path = join(project_root, 'subfolder1')
print(project_root)
print(output_path)