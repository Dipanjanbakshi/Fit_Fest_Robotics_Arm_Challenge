from setuptools import find_packages, setup

package_name = 'pymoveit2'

setup(
    name=package_name,
    version='0.0.0',
    # no package_dir: code lives under this package's folder (pymoveit2/pymoveit2)
    packages=find_packages(include=['pymoveit2', 'pymoveit2.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        "console_scripts": [
            "pick_and_place = pymoveit2.pick_and_place:main",
            "ex_joint_goal = pymoveit2.ex_joint_goal:main",
            "ex_pose_goal = pymoveit2.ex_pose_goal:main",
            "ex_gripper = pymoveit2.ex_gripper:main",
            "ex_fk = pymoveit2.ex_fk:main",
            "ex_ik = pymoveit2.ex_ik:main",
        ]
    },
)
