from setuptools import setup
from glob import glob

package_name = 'ball_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name +'/launch',['launch/ball_detection.launch.py']),
        ('lib/'+package_name,[package_name+'/config_and_weights/ball_detector_config.py']),
        ('lib/'+package_name+'/pytorchyolo',[package_name+'/pytorchyolo/detect.py']),
        ('lib/'+package_name+'/pytorchyolo',[package_name+'/pytorchyolo/models.py']),
        ('lib/'+package_name+'/pytorchyolo',[package_name+'/pytorchyolo/test.py']),
        ('lib/'+package_name+'/pytorchyolo',[package_name+'/pytorchyolo/train.py']),
        ('lib/'+package_name+'/pytorchyolo/utils',[package_name+'/pytorchyolo/utils/augmentations.py']),
        ('lib/'+package_name+'/pytorchyolo/utils',[package_name+'/pytorchyolo/utils/datasets.py']),
        ('lib/'+package_name+'/pytorchyolo/utils',[package_name+'/pytorchyolo/utils/logger.py']),
        ('lib/'+package_name+'/pytorchyolo/utils',[package_name+'/pytorchyolo/utils/loss.py']),
        ('lib/'+package_name+'/pytorchyolo/utils',[package_name+'/pytorchyolo/utils/parse_config.py']),
        ('lib/'+package_name+'/pytorchyolo/utils',[package_name+'/pytorchyolo/utils/transforms.py']),
        ('lib/'+package_name+'/pytorchyolo/utils',[package_name+'/pytorchyolo/utils/utils.py']),
        ('lib/'+package_name+'/pytorchyolo/utils',[package_name+'/pytorchyolo/utils/utils2.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='19schleid',
    maintainer_email='florian.schleid@uni-hamburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'YoloWrapper = ball_detection.YoloWrapper:main'
        ],
    },
)
