from setuptools import find_packages, setup

package_name = 'ros2_chat_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prudhviraj',
    maintainer_email='prudhvirajchalapaka07@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ai_bot = ros2_chat_bot.ai_bot:main',
            'user_client = ros2_chat_bot.user_node:main',
            'gui_client = ros2_chat_bot.gui_client:main',
        ],
    },
)
