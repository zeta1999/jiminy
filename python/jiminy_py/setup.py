from setuptools import setup, find_packages

setup(name = 'jiminy_py',
      version = '0.6',
      description = 'Package containing python-native methods.',
      author = 'Alexis Duburcq',
      packages=find_packages('src'),
      package_dir={'': 'src'},
      zip_safe = False)
