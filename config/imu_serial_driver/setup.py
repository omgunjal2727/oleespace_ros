from setuptools import setup
from setuptools.command.install import install
import os
from glob import glob
import shutil

package_name = 'imu_serial_driver'

class CustomInstall(install):
    def run(self):
        install.run(self)
        # Copy executable from bin to lib after installation
        bin_dir = os.path.join(self.install_base, 'bin')
        lib_dir = os.path.join(self.install_base, 'lib', package_name)
        os.makedirs(lib_dir, exist_ok=True)
        
        exe_name = 'imu_serial_node'
        src = os.path.join(bin_dir, exe_name)
        dst = os.path.join(lib_dir, exe_name)
        
        if os.path.exists(src):
            shutil.copy2(src, dst)
            os.chmod(dst, 0o755)

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OleeSpace Team',
    maintainer_email='your@email.com',
    description='ROS2 serial driver for ISM330DHCX IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_serial_node = imu_serial_driver.imu_serial_node:main',
        ],
    },
    cmdclass={
        'install': CustomInstall,
    },
)