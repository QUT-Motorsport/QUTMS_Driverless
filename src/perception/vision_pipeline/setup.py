from glob import glob
import os

from setuptools import setup

from typing import List, Tuple

package_name = "vision_pipeline"


def generate_yolov5_data_files() -> List[Tuple[str, str]]:
    data_files: List[Tuple[str, str]] = []
    install_base = os.path.join("share", package_name)
    for root, dirs, files in os.walk("yolov5"):
        if root.startswith("."):
            continue

        install = os.path.join(install_base, root)
        sources = [os.path.join(root, f) for f in files if not f.startswith(".")]
        data_files.append((install, sources))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ]
    + generate_yolov5_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alistair English, Alastair Bradford",
    maintainer_email="team@qutmotorsport.com",
    description="Pipeline for managing visual detection of cones.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "torch_detector = vision_pipeline.node_detector:main_torch",
            "v8_torch_detector = vision_pipeline.node_detector:main_v8_torch",
            "annotator = vision_pipeline.node_annotator:main",
            "image_saver = vision_pipeline.node_image_saver:main",
        ],
    },
)
