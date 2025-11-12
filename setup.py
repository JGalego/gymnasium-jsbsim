from setuptools import setup, find_packages


setup(
      name='gymnasium_jsbsim',
      version='0.1',
      description='✈️ A reinforcement learning environment for aircraft control using the JSBSim flight dynamics model',
      url='https://github.com/JGalego/gymnasium-jsbsim',
      author='JGalego',
      license='MIT',
      install_requires=[
            'numpy>=2.3.4',
            'gymnasium>=1.2.2',
            'matplotlib>=3.10.7',
            'jsbsim>=1.2.3',
      ],
      packages=find_packages(),
      classifiers=[
            'License :: OSI Approved :: MIT License',
            'Development Status :: 2 - Pre-Alpha',
            'Intended Audience :: Science/Research',
            'Programming Language :: Python :: 3.10',
            'Topic :: Scientific/Engineering :: Artificial Intelligence',
      ],
      python_requires='>=3.10',
      include_package_data=True,
      zip_safe=False
)
