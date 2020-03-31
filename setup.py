import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='pydobot',
    packages=['pydobot'],
    version='1.2.0',
    description='Python library for Dobot Magician',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Luis Mesas',
    author_email='luismesas@gmail.com',
    url='https://github.com/luismesas/pydobot',
    download_url='https://github.com/luismesas/pydobot/archive/v1.2.0.tar.gz',
    keywords=['dobot', 'magician', 'robotics'],
    classifiers=[],
    install_requires=[
        'pyserial==3.4'
    ]
)
