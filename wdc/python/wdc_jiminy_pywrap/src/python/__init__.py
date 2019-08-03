###############################################################################
## @brief             Entry point for wdc_jiminy_pywrap python module.
##
## @copyright         Wandercraft
###############################################################################

import os as _os # private import
import sys as _sys

if (_sys.version_info > (3, 0)):
    from contextlib import redirect_stderr as _redirect_stderr
    with open(_os.devnull, 'w') as stderr, _redirect_stderr(stderr):
        from jiminy import *  # Load jiminy module since wdc_jiminy is built on top of it
        from .libwdc_jiminy_pywrap import *
else:
    with open(_os.devnull, 'w') as stderr:
        old_target = _sys.stderr
        _sys.stderr = stderr

        from jiminy import *
        from .libwdc_jiminy_pywrap import *

        _sys.stderr = old_target