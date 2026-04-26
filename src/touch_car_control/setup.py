from setuptools import find_packages, setup

package_name = "touch_car_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "INFO.txt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ws_car_1 user",
    maintainer_email="user@example.com",
    description="Reactive bumper controller for the Gazebo touch car.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bumper_driver = touch_car_control.bumper_driver:main",
        ],
    },
)
