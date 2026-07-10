project = 'libbno055-linux'
copyright = '2026, lazytatzv'
author = 'lazytatzv'
release = '1.0.0'

extensions = [
    'myst_parser',
    'sphinxcontrib.mermaid',
]

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

html_theme = 'furo'
