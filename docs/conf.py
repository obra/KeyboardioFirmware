# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('..'))
sys.path.append('_sphinx')



# -- Project information -----------------------------------------------------

project = 'Kaleidoscope'
copyright = '2013-2025, Keyboardio & Friends'
author = 'Keyboardio & Friends'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    #'breathe',
    #'exhale',
    'recommonmark',
    'sphinx_markdown_tables'
]

extensions.append('copy-examples')
extensions.append('copy-plugin-readmes')


# Setup the breathe extension
breathe_projects = {
    "Kaleidoscope": "./doxyoutput/xml"
}
breathe_default_project = "Kaleidoscope"

# Setup the exhale extension
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",
    "rootFileName":          "library_root.rst",
    "rootFileTitle":         "Kaleidoscope",
    "doxygenStripFromPath":  "..",
    # Suggested optional arguments
    "createTreeView":        True,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    "exhaleExecutesDoxygen": True,
    "exhaleUseDoxyfile": True,
    "verboseBuild": True
}

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'


cpp_id_attributes = ["aligned","packed","weak","always_inline","noinline","no-unroll-loops","__attribute__((optimize(3)))"]
cpp_paren_attributes = ["optimize","__aligned__","section","deprecated"]

source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'restructuredtext',
}




# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', '_sphinx', 'Thumbs.db', '.DS_Store', 'etc', 'requirements.txt']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
html_theme_options = {
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    'vcs_pageview_mode': '',
    'style_nav_header_background': 'white',
    "display_github": False, # Add 'Edit on Github' link instead of 'View page source'

    # Toc options

    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 8,
    'includehidden': False,
    'titles_only': True
}

#

# on_rtd is whether we are on readthedocs.org, this line of code grabbed from docs.readthedocs.org
on_rtd = os.environ.get('READTHEDOCS', None) == 'True'

if not on_rtd:  # only import and set the theme if we're building docs locally
    import sphinx_rtd_theme
    html_theme = 'sphinx_rtd_theme'
    html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']


master_doc = 'index'
