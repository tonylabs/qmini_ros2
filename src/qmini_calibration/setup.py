from setuptools import find_packages, setup

package_name = "qmini_calibration"

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
    description="Calibration nodes + offline analyzers (Jupyter) for sim-to-real diffing.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Individual calibration nodes land per M4 measurement.
            # "joint_zero_calib = qmini_calibration.joint_zero_calib:main",
            # "imu_noise_calib = qmini_calibration.imu_noise_calib:main",
            # "actuator_latency_calib = qmini_calibration.actuator_latency_calib:main",
        ],
    },
)
