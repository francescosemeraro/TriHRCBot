from setuptools import setup

package_name = 'non_dyadic_ai'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='francesco',
    maintainer_email='francesco.semeraro@manchester.ac.uk',
    description='Package for the non-dyadic system that processes the 3D skeleton poses acquired and extracts the joint activites of the pair of people. It also computes on the run the mediated pose between the two users, according to the joint action being performed.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'full_pose_calculator = non_dyadic_ai.full_compute_pose:main',
            'partial_pose_calculator = non_dyadic_ai.partial_compute_pose:main',
            'tf_classifier = non_dyadic_ai.classifier:main',
        ],
    },
)
