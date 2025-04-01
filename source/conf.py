# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
import os
import subprocess 
import sys
sys.path.insert(0, os.path.abspath('../'))
sys.path.insert(0, os.path.abspath('../scripts/'))

# Mock imports for modules that can't be installed in the documentation environment
import mock
 
MOCK_MODULES = ['rospy', 'actionlib', 'nav_msgs', 'nav_msgs.msg', 'assignment_2_2024', 
                'assignment_2_2024.action', 'assignment_2_2024.msg', 'assignment_2_2024.srv']
for mod_name in MOCK_MODULES:
    sys.modules[mod_name] = mock.Mock()

show_authors=True
# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Assignment1'
copyright = '2025, Mazen_Madbouly'
author = 'mazen_madbouly'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc', 
    'sphinx.ext.doctest', 
    'sphinx.ext.intersphinx', 
    'sphinx.ext.todo', 
    'sphinx.ext.coverage', 
    'sphinx.ext.mathjax', 
    'sphinx.ext.ifconfig', 
    'sphinx.ext.viewcode', 
    'sphinx.ext.githubpages', 
    "sphinx.ext.napoleon",
    'sphinx.ext.inheritance_diagram',
    'breathe'
]

templates_path = ['_templates']
exclude_patterns = []

highlight_language = 'c++' 
source_suffix = '.rst' 
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']

# Extension configuration
# -- Options for intersphinx extension ---------------------------------------
# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'python': ('https://docs.python.org/3', None)}

# -- Options for todo extension ----------------------------------------------
# If true, `todo` and `todoList` produce output, else they produce nothing. 
todo_include_todos = True