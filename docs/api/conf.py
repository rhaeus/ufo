project = "UFO"
author = "Daniel Duberg"
copyright = "2025, Daniel Duberg"

extensions = ["breathe"]

# -- Breathe configuration ---------------------------------------------------

breathe_projects = {"UFO": "../../build/docs/xml"}
breathe_default_project = "UFO"

# -- HTML output -------------------------------------------------------------

html_theme = "furo"
html_title = "UFO C++ API"

html_theme_options = {
    "light_css_variables": {
        "color-brand-primary": "#2962ff",
        "color-brand-content": "#2962ff",
    },
    "dark_css_variables": {
        "color-brand-primary": "#82b1ff",
        "color-brand-content": "#82b1ff",
    },
}
