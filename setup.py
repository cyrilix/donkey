import os

from setuptools import setup, find_packages


# include the non python files
def package_files(directory, strip_leading):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            package_file = os.path.join(path, filename)
            paths.append(package_file[len(strip_leading):])
    return paths


tests_require = ['pytest',
                 'matplotlib',
                 'docker-compose',
                 'docker']

car_templates = ['templates/*']
web_controller_html = package_files('donkeycar/parts/web_controller/templates', 'donkeycar/')

extra_files = car_templates + web_controller_html

setup(name='donkeycar',
      version='2.2.1',
      description='Self driving library for python.',
      url='https://github.com/cyrilix/donkey',
      license='MIT',
      entry_points={
          'console_scripts': [
              'donkey=donkeycar.management.base:execute_from_command_line',
          ],
      },
      setup_requires=['pytest-runner'],
      install_requires=['numpy',
                        'pillow',
                        'docopt',
                        'tornado==4.5.3',
                        'requests',
                        'python-socketio',
                        'flask',
                        'eventlet',
                        'moviepy',
                        'opencv-python',
                        'imutils',
                        'pandas',
                        'pyserial',
                        'paho-mqtt'],
      tests_require=tests_require,
      extras_require={
          'pi': ['picamera',
                 'Adafruit_PCA9685',
                 'RPi.GPIO',
                 'smbus-cffi',
                 'wiringpi',
                 'Pygame'],
          'learning': ['keras',
                       'tensorflow>=1.1',
                       'h5py'],
          'tests': tests_require
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
