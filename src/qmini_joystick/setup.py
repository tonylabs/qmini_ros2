from setuptools import find_packages, setup

package_name = "qmini_joystick"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tony Wang",
    maintainer_email="okay1984@gmail.com",
    description="PS4 DualShock joystick parser. Publishes qmini_msgs/JoystickCommand.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Mapper node lands when joystick milestone arrives.
            # "ps4_mapper = qmini_joystick.ps4_mapper:main",
        ],
    },
)
