import os
from distutils.core import setup

build_num = os.environ['CIRCLE_BUILD_NUM']

setup(
    name='pydobot',
    packages=['pydobot'],
    version='0.2.%s' % build_num,
    description='Python 3 library for Dobot Magician',
    author='Luis Mesas',
    author_email='luismesas@gmail.com',
    url='https://github.com/luismesas/pydobot',
    download_url='https://github.com/luismesas/pydobot/archive/0.2.%s.tar.gz' % build_num,
    keywords=['dobot', 'magician', 'robotics'],
    classifiers=[],
    install_requires=[
        'pyserial'
    ]
)
