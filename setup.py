#!/usr/bin/env python3
from setuptools import setup, find_packages
import os

# Read the README file for long description
def read_readme():
    readme_path = os.path.join(os.path.dirname(__file__), 'README.md')
    if os.path.exists(readme_path):
        with open(readme_path, 'r', encoding='utf-8') as f:
            return f.read()
    return "MuJoCo AR Viewer - AR visualization for MuJoCo simulations"

# Read requirements
def read_requirements():
    req_path = os.path.join(os.path.dirname(__file__), 'requirements.txt')
    requirements = []
    if os.path.exists(req_path):
        with open(req_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    requirements.append(line)
    return requirements

setup(
    name='mujoco-ar-viewer',
    version='0.1.0',
    description='AR visualization for MuJoCo physics simulations',
    long_description=read_readme(),
    long_description_content_type='text/markdown',
    author='Improbable AI',
    author_email='',
    url='https://github.com/Improbable-AI/mujocoARViewer',
    packages=find_packages(exclude=['tests*', 'scenes*', 'Source*']),
    package_data={
        'mujoco_arviewer': ['generated/*.py'],
    },
    include_package_data=True,
    install_requires=read_requirements(),
    python_requires='>=3.8',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Scientific/Engineering :: Physics',
        'Topic :: Scientific/Engineering :: Visualization',
    ],
    keywords='mujoco ar vr visualization physics simulation',
    project_urls={
        'Bug Reports': 'https://github.com/Improbable-AI/mujocoARViewer/issues',
        'Source': 'https://github.com/Improbable-AI/mujocoARViewer',
    },
)