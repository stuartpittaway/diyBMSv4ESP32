import re
import json
import glob
import shutil
import gzip
import os
import errno
import sys
import importlib
Import('env')


def load_minifier():
    # Only reach for pip when the minifier is not already usable.  Installing on every build
    # costs a round trip to the package index each time, and an offline build on a machine
    # that already has it has no reason to go looking.
    #
    # htmlmin2 is a maintained fork of htmlmin, which has had no release since 2020 and no
    # longer installs at all: it imports "cgi", removed from the standard library in Python
    # 3.13 (PEP 594).  The fork keeps the same module name.
    for last_attempt in (False, True):
        try:
            import htmlmin
            return htmlmin.minify
        except (ImportError, AttributeError) as e:
            if last_attempt:
                # Carry on with unminified pages - a build without a network, or on a machine
                # where pip is locked down, should still produce working firmware.  Say so
                # loudly though: this used to happen silently, and the pages are not small.
                print('  WARNING: cannot use htmlmin (%s)' % e)
                print('  WARNING: pages will be embedded unminified, costing ~20kB of flash')
                return None

            print('  htmlmin not usable (%s), installing htmlmin2' % e)

            # --force-reinstall also repairs the one messy case: both packages ship the same
            # "htmlmin" package directory, so on a machine that had the original, removing it
            # afterwards takes the shared files with it and leaves an importable but empty
            # module.  A plain install would report "already satisfied" and change nothing.
            env.Execute('"$PYTHONEXE" -m pip install --upgrade --force-reinstall htmlmin2')

            # That empty module is already in sys.modules, so drop it before trying again.
            sys.modules.pop('htmlmin', None)
            importlib.invalidate_caches()


def prepare_www_files():
    # WARNING -  this script will DELETE your 'data' dir and recreate an empty one to copy/gzip files from 'data_src'
    #           so make sure to edit your files in 'data_src' folder as changes madt to files in 'data' will be LOST
    #
    #           If 'data_src' dir doesn't exist, and 'data' dir is found, the script will automatically
    #           rename 'data' to 'data_src

    # add filetypes (extensions only) to be gzipped before uploading. Everything else will be copied directly
    filetypes_to_gzip = ['js', 'css', 'ico']
    filetypes_to_minify = ['htm']

    print('[COPY/GZIP DATA FILES]')

    #data_dir = env.get('PROJECTDATA_DIR')

    data_dir = os.path.join(env.get('PROJECT_DIR'), 'web_temp')
    data_src_dir = os.path.join(env.get('PROJECT_DIR'), 'web_src')

    if(os.path.exists(data_dir)):
        print('  Deleting data dir ' + data_dir)
        shutil.rmtree(data_dir)

    print('  Re-creating empty data dir ' + data_dir)
    os.mkdir(data_dir)

    files_to_gzip = []
    for extension in filetypes_to_gzip:
        files_to_gzip.extend(
            glob.glob(os.path.join(data_src_dir, '*.' + extension)))
    print('  files to gzip: ' + str(files_to_gzip))

    files_to_minify = []
    for extension in filetypes_to_minify:
        files_to_minify.extend(
            glob.glob(os.path.join(data_src_dir, '*.' + extension)))
    print('  files to minify: ' + str(files_to_minify))

    all_files = glob.glob(os.path.join(data_src_dir, '*.*'))
    files_to_copy = list(
        set(all_files) - set(files_to_gzip) - set(files_to_minify))
    print('  files to copy: ' + str(files_to_copy))

    for file in files_to_copy:
        print('  Copying file: ' + file + ' to data dir')
        shutil.copy(file, data_dir)

    # Call htmlmin through its module rather than its command line.  pip puts the console
    # script next to whichever Python is running PlatformIO, and nothing adds that directory
    # to PATH for the commands SCons runs, so "htmlmin ..." only resolved when PlatformIO
    # happened to be installed into a Python already on PATH.
    minify = load_minifier() if files_to_minify else None

    for file in files_to_minify:
        destination = os.path.join(data_dir, os.path.basename(file))

        if minify is None:
            print('  Copying file UNMINIFIED: ' + file + ' to data dir')
            shutil.copy(file, destination)
            continue

        print('  Minify file: ' + file + ' to data dir')

        with open(file, 'r', encoding='utf-8') as f_in:
            content = f_in.read()

        # remove_optional_attribute_quotes=False is the old --keep-optional-attribute-quotes.
        with open(destination, 'w', encoding='utf-8') as f_out:
            f_out.write(minify(content, remove_optional_attribute_quotes=False))

    for file in files_to_gzip:
        print('  GZipping file: ' + file + ' to data dir')
        with open(file, 'rb') as f_in, gzip.GzipFile(filename=os.path.join(data_dir, os.path.basename(file) + '.gz'), mode='w', compresslevel=9) as f_out:
            shutil.copyfileobj(f_in, f_out)

    print('[/COPY/GZIP DATA FILES]')


def get_build_flag_value(flag_name):
    build_flags = env.ParseFlags(env['BUILD_FLAGS'])
    flags_with_value_list = [build_flag for build_flag in build_flags.get(
        'CPPDEFINES') if type(build_flag) == list]
    defines = {k: v for (k, v) in flags_with_value_list}
    return str(defines.get(flag_name))


prepare_www_files()

# https://www.andiamo.co.uk/resources/iso-language-codes/
