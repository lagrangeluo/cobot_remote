import os
import sysconfig
import sys
import shutil
import zipfile
from hashlib import sha256
from base64 import urlsafe_b64encode
import re

from wheel.wheelfile import WheelFile


def distutils_dir_name(dname):
    """Returns the name of a distutils build directory"""
    f = "{dirname}.{platform}-{version[0]}.{version[1]}"
    return os.path.join('build', f.format(dirname=dname,
                                          platform=sysconfig.get_platform(),
                                          version=sys.version_info))


def convert_to_manylinux(name, version):
    """
    Modifies the arch metadata of a pip package linux_x86_64=>manylinux1_x86_64
    :param name:
    :param version:
    :return:
    """
    # Get python version as XY (27, 35, 36, etc)
    python_version = str(sys.version_info.major) + str(sys.version_info.minor)
    name_version = '{}-{}'.format(name.replace('-', '_'), version)

    # linux wheel package
    dist_zip = '{0}-cp{1}-cp{1}m-linux_x86_64.whl'.format(name_version,
                                                          python_version)
    dist_zip_path = os.path.join('dist', dist_zip)
    if not os.path.exists(dist_zip_path):
        print('Wheel not found: {}'.format(dist_zip_path))
        return

    unzip_dir = 'dist/unzip/{}'.format(dist_zip)
    os.makedirs(unzip_dir, exist_ok=True)
    with zipfile.ZipFile(dist_zip_path, 'r') as zip_ref:
        zip_ref.extractall(unzip_dir)

    wheel_file = '{}.dist-info/WHEEL'.format(name_version)
    new_wheel_str = ''
    with open(os.path.join(unzip_dir, wheel_file)) as f:
        for line in f.readlines():
            if line.startswith('Tag'):
                # Replace the linux tag
                new_wheel_str += line.replace('linux', 'manylinux1')
            else:
                new_wheel_str += line

    # compute hash & size of the new WHEEL file
    # Follows https://www.python.org/dev/peps/pep-0376/#record
    m = sha256()
    m.update(new_wheel_str.encode('utf-8'))
    hash = urlsafe_b64encode(m.digest()).decode('utf-8')
    hash = hash.replace('=', '')

    with open(os.path.join(unzip_dir, wheel_file), 'w') as f:
        f.write(new_wheel_str)
    statinfo = os.stat(os.path.join(unzip_dir, wheel_file))
    byte_size = statinfo.st_size

    record_file = os.path.join(unzip_dir,
                               '{}.dist-info/RECORD'.format(name_version))
    new_record_str = ''
    with open(record_file) as f:
        for line in f.readlines():
            if line.startswith(wheel_file):
                # Update the record for the WHEEL file
                new_record_str += '{},sha256={},{}'.format(wheel_file, hash,
                                                           str(byte_size))
                new_record_str += os.linesep
            else:
                new_record_str += line

    with open(record_file, 'w') as f:
        f.write(new_record_str)

    def zipdir(path, ziph):
        for root, dirs, files in os.walk(path):
            for file in files:
                ziph.write(os.path.join(root, file),
                           os.path.join(root, file).replace(path, ''))

    new_zip_name = dist_zip.replace('linux', 'manylinux1')
    print('Generating new zip {}...'.format(new_zip_name))
    zipf = zipfile.ZipFile(os.path.join('dist', new_zip_name),
                           'w', zipfile.ZIP_DEFLATED)
    zipdir(unzip_dir, zipf)
    zipf.close()

    shutil.rmtree(unzip_dir, ignore_errors=True)
    os.remove(dist_zip_path)


def rename_package_wheel(wheel_file, new_package_name='', new_version=''):
    from os.path import join
    if not new_package_name and not new_version:
        raise Exception('Must specify either new_package_name or new_version')
    wheel_filename = os.path.basename(wheel_file)

    with WheelFile(wheel_file) as wf:
        old_name = wf.parsed_filename.group('name')
        old_version = wf.parsed_filename.group('ver')
        old_name_ver = wf.parsed_filename.group('namever')
        name = new_package_name if new_package_name else old_name
        version = new_version if new_version else old_version
        name_ver = '{}-{}'.format(name, version)
        extracted_dir = name_ver
        if os.path.exists(extracted_dir):
            shutil.rmtree(extracted_dir, ignore_errors=True)
        wf.extractall(extracted_dir)

    shutil.move(join(extracted_dir, '{}.dist-info'.format(old_name_ver)),
                join(extracted_dir, '{}.dist-info'.format(name_ver)))
    if os.path.exists(join(extracted_dir, '{}.data'.format(old_name_ver))):
        shutil.move(join(extracted_dir, '{}.data'.format(old_name_ver)),
                    join(extracted_dir, '{}.data'.format(name_ver)))
    metadata_file = join(extracted_dir, '{}.dist-info'.format(name_ver),
                         'METADATA')
    with open(metadata_file, 'r', encoding='utf8') as f:
        metadata_text = f.read()
    metadata_text = re.sub(r'^Name: .+',
                           'Name: {}'.format(new_package_name),
                           metadata_text, flags=re.MULTILINE)
    metadata_text = re.sub(r'^Version: .+',
                           'Version: {}'.format(version),
                           metadata_text, flags=re.MULTILINE)
    with open(metadata_file, 'w', encoding='utf-8') as f:
        f.write(metadata_text)

    wheel_filename = wheel_filename.replace(old_name_ver, name_ver)
    with WheelFile(wheel_filename, 'w') as wf:
        wf.write_files(extracted_dir)
    shutil.rmtree(extracted_dir, ignore_errors=True)
    return wheel_filename


def rename_package_wheel_main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('wheel_file',
                        help='The existing wheel to copy and modify')
    parser.add_argument('--name', '-n',
                        help='The new name for the wheel package', default=None)
    parser.add_argument('--version', '-v',
                        help='The new version for the wheel package',
                        default=None)
    ns = parser.parse_args()
    wheel_filename = rename_package_wheel(ns.wheel_file, ns.name, ns.version)
    print('Created {}'.format(wheel_filename))


__all__ = ['distutils_dir_name', 'convert_to_manylinux', 'rename_package_wheel']
