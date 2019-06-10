from setuptools import setup, find_packages

setup(name = 'exo_simu_py',
    version = '0.1',
    description = 'Package containing python-native methods.',
    author = 'Alexis Duburcq',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    zip_safe = False)
