import os
from glob import glob

from setuptools import setup

package_name = "kerala_bot_sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="siddharth",
    maintainer_email="siddharth@e-yantra.org",
    description="Stepper+belt trajectory feasibility simulator with RViz playback.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "rviz_player = kerala_bot_sim.rviz_player:main",
            "state_listener = kerala_bot_sim.state_listener:main",
            "run_demo = kerala_bot_sim.run_demo:main",
        ],
    },
)
