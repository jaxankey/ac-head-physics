import zipfile, os, subprocess

# All the files that are needed
files = ['cockpit.lua', 'manifest.ini', 'settings.ini']

# Get the version number from the manifest
f = open('manifest.ini', 'r')
for line in f:
    s = line.split('=')
    if s[0].strip() == 'VERSION': 
        version = s[1].strip()
        print('VERSION='+version)
f.close()

# Get the base path
base_path = os.path.join('ac-head-physics')

# Open the zip file
zip_name = 'ac-head-physics v'+version+'.zip'
print(zip_name)
zip_file = zipfile.ZipFile(zip_name, 'w')

# Write the files
for file in files: 
    print(' ', file)
    zip_file.write(file, os.path.join(base_path, file))

# Installation
zip_file.write('INSTALLATION INSTRUCTIONS.txt')

# Close it.
zip_file.close()
print('ALL DONE!')

subprocess.Popen(r'explorer /select, "'+os.path.join(os.getcwd(),zip_name)+'"')