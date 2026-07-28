from setuptools import setup

package_name = "parameter_identify"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/identify.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RC2026 Robotic Arm Team",
    maintainer_email="todo@example.com",
    description="Offline dynamic parameter identification tools for the robotic arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "identify_arm = parameter_identify.identify_arm:main",
            "generate_trajectory = parameter_identify.generate_trajectory:main",
        ],
    },
)
