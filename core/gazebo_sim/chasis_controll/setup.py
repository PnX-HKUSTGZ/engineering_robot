from setuptools import setup
import os # <-- 确保导入 os 模块

package_name = 'chasis_controll' # 定义包名变量

setup(
    name=package_name,
    version='0.0.0',
    packages=[], # 你的包不使用 find_packages，这是正确的
    py_modules=[package_name], # 你的主要 Python 模块名称就是包名 'chasis_controll'
    install_requires=['setuptools'],
    # *** 添加 data_files 来安装资源标记 ***
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), # 这行是关键！
        ('share/' + package_name, ['package.xml']),
        # 如果你的包还有其他文件（如 Launch 文件、配置文件等），也需要在这里添加
        # 例如：('share/' + package_name + '/launch', ['launch/your_launch_file.launch.py']),
        # 例如：(os.path.join('share', package_name, 'config'), ['config/your_config.yaml']),
    ],
    maintainer='Rohan Agrawal',
    maintainer_email='rohan@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleop from the keyboard.'
    ),
    entry_points={
        'console_scripts': [
            f'{package_name} = {package_name}:main' # 使用 f-string 更通用
        ],
    },
)