import os

import pip
from setuptools import setup, find_packages


def require(requirement_file):
    requires = []
    requirements = pip.req.parse_requirements(requirement_file, session=pip.download.PipSession())

    for item in requirements:
        if item.req:
            requires.append(str(item.req))
    return requires


# include the non python files
def package_files(directory, strip_leading):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            package_file = os.path.join(path, filename)
            paths.append(package_file[len(strip_leading):])
    return paths


car_templates = ['templates/*']
web_controller_html = package_files('donkeycar/parts/web_controller/templates', 'donkeycar/')

extra_files = car_templates + web_controller_html
print('extra_files', extra_files)

setup(name='donkeycar',
      version='2.2.1',
      description='Self driving library for python.',
      url='https://github.com/wroscoe/donkey',
      download_url='https://github.com/wroscoe/donkey/archive/2.1.5.tar.gz',
      author='Will Roscoe',
      author_email='wroscoe@gmail.com',
      license='MIT',
      entry_points={
          'console_scripts': [
              'donkey=donkeycar.management.base:execute_from_command_line',
          ],
      },
      install_requires=require('requirements.txt'),
      tests_require=require('requirements-dev.txt'),
      extras_require={
          'pi': require('requirements-pi.txt'),
          'learning': require('requirements-learning.txt'),
          'tests': require('requirements-dev.txt'),
      },
      package_data={
          'donkeycar': extra_files,
      },

      include_package_data=True,

      classifiers=[
          # How mature is this project? Common values are
          #   3 - Alpha
          #   4 - Beta
          #   5 - Production/Stable
          'Development Status :: 3 - Alpha',

          # Indicate who your project is intended for
          'Intended Audience :: Developers',
          'Topic :: Scientific/Engineering :: Artificial Intelligence',

          # Pick your license as you wish (should match "license" above)
          'License :: OSI Approved :: MIT License',

          # Specify the Python versions you support here. In particular, ensure
          # that you indicate whether you support Python 2, Python 3 or both.

          'Programming Language :: Python :: 3.4',
          'Programming Language :: Python :: 3.5',
          'Programming Language :: Python :: 3.6',
      ],
      keywords='selfdriving cars drive',

      packages=find_packages(exclude=(['tests', 'docs', 'site', 'env'])),
      )
