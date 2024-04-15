from setuptools import setup

package_name = 'maze_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tune65',
    maintainer_email='thommandra65@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'talker = maze_bot.publisher_member_function:main',
            'driving_node = maze_bot.driving_node:main',
            'go_to_goal_node = maze_bot.go_to_goal:main',
            'video_recoder = maze_bot.video_saver:main',
            'maze_solver_node = maze_bot.maze_solver:main',
            'maze_solver_run = maze_bot.maze_bot_run:main',    
        ],
    },
)
