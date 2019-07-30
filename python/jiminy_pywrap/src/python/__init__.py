###############################################################################
## @brief             Entry point for jiminy_pywrap python module.
###############################################################################

import os as _os # private import
import sys as _sys
import pinocchio as _pnc
_sys.path.append(_os.path.dirname(_pnc.__file__)) # Required to be able to find libpinocchio_pywrap.so
import libpinocchio_pywrap as _pin # Preload the dynamic library Python bindings

from wdc.redirect_stdio import RedirectStdErr as _RedirectStdErr
with _RedirectStdErr():
    from .libjiminy_pywrap import *

# We remove the NONBLOCK flag which is set on stdin file by the logger when importing a wdc C++ library.
from wdc.redirect_stdio import reset_stdin_flag as _reset_stdin_flag
_reset_stdin_flag()
