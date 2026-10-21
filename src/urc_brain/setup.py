from setuptools import find_packages, setup

setup(
    name='urc_brain', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/urc_brain']),
                ('share/urc_brain', ['package.xml'])],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='Adriel', maintainer_email='294955227+adrielsta@users.noreply.github.com',
    description='URC rover simulation-first autonomy', license='Apache-2.0',
    entry_points={'console_scripts': [
        'supervisor = urc_brain.supervisor:main',
        'mission = urc_brain.mission:main',
        'terrain_scan = urc_brain.terrain_scan:main',
        'practice_referee = urc_brain.practice_referee:main',
    ]},
)
