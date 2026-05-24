from setuptools import find_packages, setup

package_name = "qmini_imu"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", []),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tony Wang",
    maintainer_email="okay1984@gmail.com",
    description="Wheeltec N100 IMU wrapper with stale-message watchdog.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Watchdog node lands in M3.
            # "imu_watchdog = qmini_imu.imu_watchdog:main",
        ],
    },
)
