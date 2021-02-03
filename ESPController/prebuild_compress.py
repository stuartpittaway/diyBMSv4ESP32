import re
import json
import glob
import shutil
import gzip
import os
import errno
Import('env')

env.Execute("$PYTHONEXE -m pip install -v htmlmin")
env.Execute("$PYTHONEXE -m pip install --upgrade htmlmin")


def prepare_www_files():
    # WARNING -  this script will DELETE your 'data' dir and recreate an empty one to copy/gzip files from 'data_src'
    #           so make sure to edit your files in 'data_src' folder as changes madt to files in 'data' woll be LOST
    #
    #           If 'data_src' dir doesn't exist, and 'data' dir is found, the script will autimatically
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

    for file in files_to_minify:
        print('  Minify file: ' + file + ' to data dir')
        shutil.copy(file, os.path.join(data_dir, os.path.basename(file)))
        env.Execute("htmlmin  --keep-optional-attribute-quotes " +
                    file+" "+os.path.join(data_dir, os.path.basename(file)))

    for file in files_to_gzip:
        print('  GZipping file: ' + file + ' to data dir')
        with open(file, 'rb') as f_in, gzip.open(os.path.join(data_dir, os.path.basename(file) + '.gz'), 'wb') as f_out:
            shutil.copyfileobj(f_in, f_out)

    print('[/COPY/GZIP DATA FILES]')


def translate_file(xsltfilename, filename):
    # Provide basic translation of input files using regular expressions
    # ideally I would do this using XSLT parser, but XSLT2 support in Python is poor

    print('[TRANSLATE]', xsltfilename, filename)
    sourcefile = os.path.join(env.get('PROJECT_DIR'), 'web_temp', filename)
    if(not os.path.exists(sourcefile)):
        raise FileNotFoundError(
            errno.ENOENT, os.strerror(errno.ENOENT), sourcefile)

    jsonfile = os.path.join(env.get('PROJECT_DIR'),
                            'translation', xsltfilename)
    if(not os.path.exists(jsonfile)):
        raise FileNotFoundError(
            errno.ENOENT, os.strerror(errno.ENOENT), jsonfile)

    with open(sourcefile, 'r', encoding='utf8') as source_file:
        source = str(source_file.read())

    # template automatically builds template.json file with the extracted data
    # which is useful when creating new translations
    template = {"match": []}
    data = {}
    with open(jsonfile, 'r', encoding='utf8') as json_file:
        data = json.load(json_file)

    for x in data["match"]:
        matches = re.search(pattern=str(
            x["pattern"]), string=source, flags=re.MULTILINE | re.UNICODE | re.DOTALL)

        if not matches:
            raise Exception("Regular expression not found "+str(x["pattern"]))

        t = {"pattern": str(x["pattern"]),
             "replacement": str(source[matches.start(1):matches.end(1)])}

        print("Replacing '", source[matches.start(1):matches.end(
            1)], "' with '", x["replacement"], "'")
        source = source[:matches.start(
            1)] + x["replacement"] + source[matches.end(1):]

        template["match"].append(t)

    print("Saving", sourcefile)
    with open(sourcefile, 'w', encoding='utf8') as source_file:
        source_file.write(source)

    print("Saving blank template", sourcefile)
    templatefile = os.path.join(
        env.get('PROJECT_DIR'), 'translation', 'template.json')
    with open(templatefile, 'w', encoding='utf8') as template_file:
        json.dump(template, template_file, indent=4)


def get_build_flag_value(flag_name):
    build_flags = env.ParseFlags(env['BUILD_FLAGS'])
    flags_with_value_list = [build_flag for build_flag in build_flags.get(
        'CPPDEFINES') if type(build_flag) == list]
    defines = {k: v for (k, v) in flags_with_value_list}
    return str(defines.get(flag_name))


prepare_www_files()

# https://www.andiamo.co.uk/resources/iso-language-codes/
# English (United Kingdom) en-gb
# Spanish es
translate_file(get_build_flag_value("LANG")+".json", "default.htm")

#raise Exception("STOP")
