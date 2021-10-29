#!/usr/bin/env python3

import os
from setuptools import setup, find_packages

directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(directory, 'README.md'), encoding='utf-8') as f:
  long_description = f.read()

setup(name='cassandra',
      version='0.1',
      description='A comprehensive open-source python simulator for TVC model rockets',
      author='Gaston Maffei',
      license='MIT',
      long_description=long_description,
      long_description_content_type='text/markdown',
      packages = find_packages(),
      classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License"
      ],
      install_requires=['numpy', 'matplotlib', 'plotext'],
      python_requires='>=3.7',
      include_package_data=True)

