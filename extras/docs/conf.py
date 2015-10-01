import sys
import os
import shlex
import subprocess

read_the_docs_build = os.environ.get('READTHEDOCS', None) == 'True'

if read_the_docs_build:
    subprocess.call('doxygen', shell=True)

extensions = ['breathe']
breathe_projects = { 'Nanoshield_IMU': 'xml' }
breathe_default_project = "Nanoshield_IMU"
templates_path = ['_templates']
source_suffix = '.rst'
master_doc = 'index'
project = u'Nanoshield_IMU'
copyright = u'2015, Nanoshield_IMU'
author = u'Nanoshield_IMU'
version = '1.0'
release = '1.0'
language = None
exclude_patterns = ['_build']
pygments_style = 'sphinx'
todo_include_todos = False
html_static_path = ['_static']
htmlhelp_basename = 'Nanoshield_IMUdoc'
latex_elements = {
}
latex_documents = [
  (master_doc, 'Nanoshield_IMU.tex', u'Nanoshield\_IMU Documentation',
   u'Nanoshield\_IMU', 'manual'),
]
